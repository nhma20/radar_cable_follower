/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>
#include <px4_msgs/msg/rc_channels.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include <std_msgs/msg/int32.hpp>

#include <nav_msgs/msg/path.hpp>

#include <radar_cable_follower_msgs/msg/tracked_powerlines.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <stdint.h>
#include <math.h>  
#include <limits>
#include <mutex>
#include <chrono>
#include <iostream>

#include "geometry.h"


#define PI 3.14159265
#define NAN_ std::numeric_limits<double>::quiet_NaN()

using namespace std::chrono;
using namespace std::chrono_literals;
// using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node {
public:
	OffboardControl() : Node("offboard_control") {
		_offboard_control_mode_publisher =
			this->create_publisher<px4_msgs::msg::OffboardControlMode>("fmu/offboard_control_mode/in", 10);
		_trajectory_setpoint_publisher =
			this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("fmu/trajectory_setpoint/in", 10);
		_vehicle_command_publisher =
			this->create_publisher<px4_msgs::msg::VehicleCommand>("fmu/vehicle_command/in", 10);

		this->declare_parameter<float>("yaw_frac", 0.25);
		this->declare_parameter<float>("pos_frac", 0.5);
		this->declare_parameter<float>("powerline_following_distance", 5.0);
		this->get_parameter("powerline_following_distance", _following_distance);
		this->declare_parameter<float>("powerline_following_speed", 1.5);
		this->get_parameter("powerline_following_speed", _follow_speed);
		this->declare_parameter<int>("powerline_following_ID", -1);

		this->declare_parameter<int>("launch_with_debug", 1);
		this->get_parameter("launch_with_debug", _launch_with_debug);

		this->declare_parameter<float>("take_off_to_height", 0.0);
		this->get_parameter("take_off_to_height", _takeoff_height);


		// VehicleStatus: https://github.com/PX4/px4_msgs/blob/master/msg/VehicleStatus.msg
		_vehicle_status_sub = create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/vehicle_status/out", 10,
            [this](px4_msgs::msg::VehicleStatus::ConstSharedPtr msg) {
              _arming_state = msg->arming_state;
              _nav_state = msg->nav_state;
			});


		_rc_channels_sub = this->create_subscription<px4_msgs::msg::RcChannels>(
			"/fmu/rc_channels/out",	10,
            [this](px4_msgs::msg::RcChannels::ConstSharedPtr msg) {
              _rc_misc_state = msg->channels[7];
			  _rc_height_state = msg->channels[6];
			  
			//   RCLCPP_INFO(this->get_logger(),  "\nRC MISC state: %f", _rc_misc_state);
			});


		_powerline_pose_sub = this->create_subscription<radar_cable_follower_msgs::msg::TrackedPowerlines>(
			"/tracked_powerlines",	10,
			std::bind(&OffboardControl::update_alignment_pose, this, std::placeholders::_1));


		_selected_id_sub = this->create_subscription<std_msgs::msg::Int32>(
			"/selected_id",	10,
            [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
				_id_mutex.lock(); {
					_selected_ID = msg->data;
				} _id_mutex.unlock();
			});


		if(_launch_with_debug > 0)
		{
			_follow_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/follow_pose", 10);
			_manual_path_pub = this->create_publisher<nav_msgs::msg::Path>("/manual_path", 10);
			_offboard_path_pub = this->create_publisher<nav_msgs::msg::Path>("/offboard_path", 10);
		}

		tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
		transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


		// get common timestamp
		_timesync_sub =	this->create_subscription<px4_msgs::msg::Timesync>("fmu/timesync/out", 10,
				[this](const px4_msgs::msg::Timesync::UniquePtr msg) {
					_timestamp.store(msg->timestamp);
				});


		timer_ = this->create_wall_timer(100ms, 
				std::bind(&OffboardControl::flight_state_machine, this));

		_mission_timer = this->create_wall_timer(250ms, 
				std::bind(&OffboardControl::mission_state_machine, this));

		_path_timer = this->create_wall_timer(500ms, 
				std::bind(&OffboardControl::publish_path, this));
		
	}


	~OffboardControl() {
		RCLCPP_INFO(this->get_logger(),  "Shutting down offboard control, landing..");
		publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND); 
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
		
	}

	void arm() const;
	void disarm() const;
	


private:
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr _mission_timer;
	rclcpp::TimerBase::SharedPtr _path_timer;

	rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_control_mode_publisher;
	rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_setpoint_publisher;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_publisher;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _follow_pose_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _manual_path_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr _offboard_path_pub;

	rclcpp::Subscription<radar_cable_follower_msgs::msg::TrackedPowerlines>::SharedPtr _powerline_pose_sub;
	rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _vehicle_status_sub;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr _selected_id_sub;
	rclcpp::Subscription<px4_msgs::msg::RcChannels>::SharedPtr _rc_channels_sub;

	std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
	std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
	std::atomic<uint64_t> _timestamp;   //!< common synced timestamped
	int _nav_state, _old_nav_state = 0;
	int _arming_state;
	geometry_msgs::msg::PoseArray::SharedPtr _powerline_array_msg; // auto?
	int _counter = 0;
	float _following_distance;
	float _follow_speed;
	int _selected_ID = -1;
	int _launch_with_debug;
	float _takeoff_height;

	bool _new_takeoff = true;
	float _rc_misc_state = -1;
	float _prev_rc_misc_state = -2;

	float _rc_height_state = -1;
	float _prev_rc_height_state = -1;

    bool _printed_offboard = false;

	bool _in_offboard = false;

	std::mutex _id_mutex;
	std::mutex _drone_pose_mutex;
	std::mutex _powerline_mutex;

	pose_t _drone_pose; // in world coordinates North-West-Up
	orientation_t _drone_orientation; // RPY
	pose_t _alignment_pose;

	float _hover_height = 2;

	void publish_path();
	void mission_state_machine();
	void flight_state_machine();
	void update_drone_pose();
	void update_alignment_pose(radar_cable_follower_msgs::msg::TrackedPowerlines::SharedPtr msg);
	void publish_offboard_control_mode() const;
	void publish_takeoff_setpoint();
	void publish_tracking_setpoint();
	void publish_hold_setpoint() const;
	void publish_setpoint(px4_msgs::msg::TrajectorySetpoint msg) const;
	void publish_vehicle_command(uint16_t command, float param1 = 0.0,
				     float param2 = 0.0) const;
	void world_to_points();
};


void OffboardControl::publish_path() {
	// limit length
	// offboard and manual paths

	static auto manual_path_msg = nav_msgs::msg::Path();
	manual_path_msg.header.stamp = this->now();
	manual_path_msg.header.frame_id = "world";

	static auto offboard_path_msg = nav_msgs::msg::Path();
	offboard_path_msg.header.stamp = this->now();
	offboard_path_msg.header.frame_id = "world";

	auto pose_msg = geometry_msgs::msg::PoseStamped();
	pose_msg.header.stamp = this->now();
	pose_msg.header.frame_id = "world";

	_drone_pose_mutex.lock(); {

		pose_msg.pose.position.x = _drone_pose.position(0);
		pose_msg.pose.position.y = _drone_pose.position(1);
		pose_msg.pose.position.z = _drone_pose.position(2);

		pose_msg.pose.orientation.x = _drone_pose.quaternion(0);
		pose_msg.pose.orientation.y = _drone_pose.quaternion(1);
		pose_msg.pose.orientation.z = _drone_pose.quaternion(2);
		pose_msg.pose.orientation.w = _drone_pose.quaternion(3);

	} _drone_pose_mutex.unlock();


	if (_in_offboard)
	{
		float dist = 99999.9;

		if (offboard_path_msg.poses.size() > 0 )
		{		
			float x_diff = offboard_path_msg.poses.back().pose.position.x - pose_msg.pose.position.x;
			float y_diff = offboard_path_msg.poses.back().pose.position.y - pose_msg.pose.position.y;
			float z_diff = offboard_path_msg.poses.back().pose.position.z - pose_msg.pose.position.z;
			dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
		}		

		if(dist > 1.0)
		{
			offboard_path_msg.poses.push_back(pose_msg);
			_offboard_path_pub->publish(offboard_path_msg);
		}
		manual_path_msg.poses.clear();
	}
	else
	{
		float dist = 99999.9;

		if (manual_path_msg.poses.size() > 0 )
		{	
			float x_diff = manual_path_msg.poses.back().pose.position.x - pose_msg.pose.position.x;
			float y_diff = manual_path_msg.poses.back().pose.position.y - pose_msg.pose.position.y;
			float z_diff = manual_path_msg.poses.back().pose.position.z - pose_msg.pose.position.z;
			dist = sqrt(pow(x_diff, 2) + pow(y_diff, 2) + pow(z_diff, 2));
		}	

		if(dist > 1.0)
		{
			manual_path_msg.poses.push_back(pose_msg);
			_manual_path_pub->publish(manual_path_msg);
		}
		offboard_path_msg.poses.clear();
	}
		
}


void OffboardControl::mission_state_machine() {

	//if (! _in_offboard)
	//{
	//	return;
	//}

	// if (_rc_misc_state < -0.5)
	// {
	// 	RCLCPP_INFO(this->get_logger(),  "\nOriginal distance and speed\n");
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_speed", _follow_speed));
	// }

	// if (_prev_rc_misc_state < -0.5 && _rc_misc_state > -0.5 && _rc_misc_state < 0.5)
	// {
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance+7.5));
	// 	RCLCPP_INFO(this->get_logger(),  "\nIncreasing following distance\n");
	// }

	// if (_prev_rc_misc_state > -0.5 && _prev_rc_misc_state < 0.5 && _rc_misc_state > 0.5)
	// {
	// 	// this->set_parameter(rclcpp::Parameter("powerline_following_ID", 0));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_speed", _follow_speed*2));
	// 	RCLCPP_INFO(this->get_logger(),  "\nIncreasing following speed\n");
	// }

	if (_prev_rc_misc_state != _rc_misc_state)
	{
	
		if (_rc_misc_state < -0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\nForward direction\n");
			_follow_speed = abs(_follow_speed);
		}

		if (_rc_misc_state > -0.5 && _rc_misc_state < 0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\nStop\n");
			_follow_speed = 0.0;
		}

		if (_rc_misc_state > 0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\nReverse direction\n");
			_follow_speed = -abs(_follow_speed);
		}

		this->set_parameter(rclcpp::Parameter("powerline_following_speed", _follow_speed));

		_prev_rc_misc_state = _rc_misc_state;
	}
	

	if (_prev_rc_height_state != _rc_height_state)
	{

		if (_rc_height_state < -0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\n-1m following height\n");
			_following_distance = _following_distance - 1.0;
		}

		if (_rc_height_state > -0.5 && _rc_height_state < 0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\nSame following height\n");
		}

		if (_rc_height_state > 0.5)
		{
			RCLCPP_INFO(this->get_logger(),  "\n+1m following height\n");
			_following_distance = _following_distance + 1.0;
		}

		this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance));

		_prev_rc_height_state = _rc_height_state;
	}

	// static int callback_count = 0;

	// if (callback_count == 0)
	// {
	// 	RCLCPP_INFO(this->get_logger(),  "\nOriginal ID, original direction\n");
	// }

	// if (callback_count == 1)
	// {
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_ID", 0));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_speed", -_follow_speed*2));
	// 	RCLCPP_INFO(this->get_logger(),  "\nNew ID, reversing direction\n");
	// }

	// if (callback_count == 2)
	// {
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_ID", 0));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_distance", _following_distance+7.5));
	// 	this->set_parameter(rclcpp::Parameter("powerline_following_speed", _follow_speed*2));
	// 	RCLCPP_INFO(this->get_logger(),  "\nGreater distance, increased speed\n");
	// }
	

	// callback_count++;
}


void OffboardControl::flight_state_machine() {

	OffboardControl::update_drone_pose();

	// If drone not armed (from external controller) and put in offboard mode, do nothing
	if (_nav_state != 14) 
	{
		if (_old_nav_state != _nav_state && _nav_state != 14 && _launch_with_debug > 0)
		{				
			RCLCPP_INFO(this->get_logger(), "nav_state: %d", _nav_state);
			RCLCPP_INFO(this->get_logger(), "\n \nWaiting for offboard mode\n");
		}

		if (_old_nav_state != _nav_state && _nav_state == 14 && _launch_with_debug > 0)
		{				
			RCLCPP_INFO(this->get_logger(), "nav_state: %d", _nav_state);
			RCLCPP_INFO(this->get_logger(), "\n \nOffboard mode enabled\n");
		}

		publish_hold_setpoint();
		_in_offboard = false;
		_new_takeoff = true;
		_old_nav_state = _nav_state;
		_counter = 0;
		return;
	}

	_in_offboard = true;

	if (!_printed_offboard)
	{
		RCLCPP_INFO(this->get_logger(), "\n \nEntering offboard control \n");
		_printed_offboard = true;
		this->arm();
	}

	this->get_parameter("take_off_to_height", _takeoff_height);
	if(_takeoff_height > 1){
		static bool takeoff_print = false;
		if(takeoff_print == false){
			takeoff_print = true;
			RCLCPP_INFO(this->get_logger(), "\n \nTaking off to %f meters\n", _takeoff_height);
		}
		publish_takeoff_setpoint();
		return;
	}

	else if(_counter < 1000000000){
		if(_counter == 10 && _launch_with_debug > 0){
			RCLCPP_INFO(this->get_logger(), "\n \nBeginning alignment \n");
		}
		publish_tracking_setpoint();

		// RCLCPP_INFO(this->get_logger(), "Alignment pose:\n X %f \n Y: %f \n Z: %f",
		// 	_alignment_pose.position(0), _alignment_pose.position(1), _alignment_pose.position(2));	
			
		// RCLCPP_INFO(this->get_logger(), "Drone pose:\n X %f \n Y: %f \n Z: %f",		
		// 	_drone_pose.position(0), _drone_pose.position(1), _drone_pose.position(2));	

	}

	_counter++;

}


void OffboardControl::update_drone_pose() {

	geometry_msgs::msg::TransformStamped t;

	try {
		if (tf_buffer_->canTransform("world", "drone", tf2::TimePointZero))	{
			t = tf_buffer_->lookupTransform("world","drone",tf2::TimePointZero);
		}
		else {
			RCLCPP_INFO(this->get_logger(), "Can not transform");
			return;
		}
	} catch (const tf2::TransformException & ex) {
		RCLCPP_INFO(this->get_logger(), "Could not transform: %s", ex.what());
		return;
	}

	_drone_pose_mutex.lock(); {

		_drone_pose.position(0) = t.transform.translation.x;
		_drone_pose.position(1) = t.transform.translation.y;
		_drone_pose.position(2) = t.transform.translation.z;
		
		_drone_pose.quaternion(0) = t.transform.rotation.x;
		_drone_pose.quaternion(1) = t.transform.rotation.y;
		_drone_pose.quaternion(2) = t.transform.rotation.z;
		_drone_pose.quaternion(3) = t.transform.rotation.w;

		// RCLCPP_INFO(this->get_logger(), "Yaw: %f", _drone_orientation(2));

	} _drone_pose_mutex.unlock();

}


void OffboardControl::update_alignment_pose(radar_cable_follower_msgs::msg::TrackedPowerlines::SharedPtr msg) {		
		
	if (msg->poses.size() < 1)
	{
		return;
	}


	float current_highest = 0;
	size_t highest_index = 0;
	int id = -1;

	_id_mutex.lock(); {
		id = _selected_ID;
	} _id_mutex.unlock();

	this->get_parameter("powerline_following_ID", id);

	for (size_t i = 0; i < msg->poses.size(); i++)
	{
		// find powerline corresponding to selected ID
		if (msg->ids[i] == id)
		{
			current_highest = msg->poses[i].position.z;
			highest_index = i;
			break;
		}
		
		// else find highest powerline
		if ( msg->poses[i].position.z > current_highest )
		{
			current_highest = msg->poses[i].position.z;
			highest_index = i;
		}
	}

	float tmp_follow_dist;
	this->get_parameter("powerline_following_distance", tmp_follow_dist);	

	_powerline_mutex.lock(); {	

		_alignment_pose.position(0) = msg->poses[highest_index].position.x;
		_alignment_pose.position(1) = msg->poses[highest_index].position.y;
		_alignment_pose.position(2) = msg->poses[highest_index].position.z + (float)tmp_follow_dist;

		_alignment_pose.quaternion(0) = msg->poses[highest_index].orientation.x;
		_alignment_pose.quaternion(1) = msg->poses[highest_index].orientation.y;
		_alignment_pose.quaternion(2) = msg->poses[highest_index].orientation.z;
		_alignment_pose.quaternion(3) = msg->poses[highest_index].orientation.w;

		// RCLCPP_INFO(this->get_logger(), "Alignment pose:\n X %f \n Y: %f \n Z: %f",
		// 	_alignment_pose.position(0), _alignment_pose.position(1), _alignment_pose.position(2));		

	} _powerline_mutex.unlock();

}


/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

	RCLCPP_INFO(this->get_logger(), "Arm command send\n");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm() const {
	publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

	RCLCPP_INFO(this->get_logger(), "Disarm command send\n");
}


/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode() const {
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = _timestamp.load();
	msg.position = true;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;	
	_offboard_control_mode_publisher->publish(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Should go to align with cable of choice there
 */
void OffboardControl::publish_tracking_setpoint() {

	if (_alignment_pose.position(0) == 0 && 
		_alignment_pose.position(1) == 0 && 
		_alignment_pose.position(2) == 0)
	{
		RCLCPP_INFO(this->get_logger(), "Nothing to align with - holding position");
		OffboardControl::publish_hold_setpoint();
		return;
	}
	

	float pos_frac;
	this->get_parameter("pos_frac", pos_frac);

	float yaw_frac;
	this->get_parameter("yaw_frac", yaw_frac);
	static float tmp_follow_speed;
	this->get_parameter("powerline_following_speed", tmp_follow_speed);

	orientation_t target_yaw_eul;

	pose_eul_t publish_pose;

	_powerline_mutex.lock(); {
	_drone_pose_mutex.lock(); {
		
		// calculate fractional yaw positions (basic porportional control)
		target_yaw_eul = quatToEul(_alignment_pose.quaternion);

		float cur_yaw = quatToEul(_drone_pose.quaternion)(2); 
		float target_yaw = target_yaw_eul(2);
		
		if ( abs(cur_yaw - target_yaw) <= (float)M_PI )
		{
			target_yaw = cur_yaw + (target_yaw - cur_yaw)*yaw_frac;			
		}
		else 
		{
			float diff = (2*M_PI - target_yaw) + cur_yaw;

			target_yaw = cur_yaw - diff*yaw_frac;			
		}


		publish_pose.position(0) = _drone_pose.position(0) + (_alignment_pose.position(0) - _drone_pose.position(0))*pos_frac;
		publish_pose.position(1) = _drone_pose.position(1) + (_alignment_pose.position(1) - _drone_pose.position(1))*pos_frac;
		publish_pose.position(2) = _drone_pose.position(2) + (_alignment_pose.position(2) - _drone_pose.position(2))*pos_frac;		

		publish_pose.orientation(0) = 0.0;
		publish_pose.orientation(1) = 0.0;
		publish_pose.orientation(2) = target_yaw;


	} _drone_pose_mutex.unlock();
	} _powerline_mutex.unlock();

	point_t unit_x(
		1.0 * tmp_follow_speed,
		0.0,
		0.0
	);


	publish_pose = pose_NWU_to_NED(publish_pose);
	
	// rotate unit x (1,0,0) velocity to align with powerline direction
	rotation_matrix_t rot_mat = quatToMat(_alignment_pose.quaternion);
	point_t unit_velocity = rotateVector(rot_mat, unit_x);

	// rotate powerline direction velocity from NWU to NED frame
	static rotation_matrix_t R_NWU_to_NED = eulToR(orientation_t(-M_PI, 0, 0));
	unit_velocity = rotateVector(R_NWU_to_NED, unit_velocity);

	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = publish_pose.position(0); // in meters NED
	msg.y = publish_pose.position(1); // in meters NED
	msg.z = publish_pose.position(2); // in meters NED
	msg.yaw = publish_pose.orientation(2); // rotation around z NED in radians
	msg.vx = unit_velocity(0); // m/s NED
	msg.vy = unit_velocity(1); // m/s NED
	msg.vz = unit_velocity(2); // m/s NED

	// RCLCPP_INFO(this->get_logger(), "Xv:%f Yv:%f Zv:%f", msg.velocity[0], msg.velocity[1], msg.velocity[2]);

	OffboardControl::publish_setpoint(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Drone should take off to _takeoff_height
 */
void OffboardControl::publish_takeoff_setpoint() {

	static pose_eul_t NWU_to_NED_pose;	

	if (_new_takeoff == true)
	{	
		// freeze takeoff setpoint
		_new_takeoff = false;
		NWU_to_NED_pose.position = _drone_pose.position; 
		NWU_to_NED_pose.orientation = quatToEul(_drone_pose.quaternion);
		NWU_to_NED_pose = pose_NWU_to_NED(NWU_to_NED_pose);
	}

	px4_msgs::msg::TrajectorySetpoint msg{};
	msg.timestamp = _timestamp.load();
	msg.x = NWU_to_NED_pose.position(0); 		// in meters NED
	msg.y = NWU_to_NED_pose.position(1);
	msg.z = - _takeoff_height;
	// YAW is cropped to 0-PI for some reason, uncrop to 0-2PI based on if ROLL is 0 or PI
	msg.yaw = (float)NWU_to_NED_pose.orientation(2);

	OffboardControl::publish_setpoint(msg);
}


/**
 * @brief Publish a trajectory setpoint
 *        Drone should holds it pose
 */
void OffboardControl::publish_hold_setpoint() const {

	pose_eul_t NWU_to_NED_pose;
	NWU_to_NED_pose.position = _drone_pose.position; 
	NWU_to_NED_pose.orientation = quatToEul(_drone_pose.quaternion);	

	NWU_to_NED_pose = pose_NWU_to_NED(NWU_to_NED_pose);

	px4_msgs::msg::TrajectorySetpoint msg{}; // in meters NED
	msg.timestamp = _timestamp.load();
	msg.x = NWU_to_NED_pose.position(0); 		
	msg.y = NWU_to_NED_pose.position(1);
	msg.z = NWU_to_NED_pose.position(2);
	//YAW is cropped to 0-PI for some reason, uncrop to 0-2PI based on if ROLL is 0 or PI
	msg.yaw = (float)NWU_to_NED_pose.orientation(2);// + (float)NWU_to_NED_pose.orientation(0);

	// RCLCPP_INFO(this->get_logger(), "DRONE EUL:\n R:%f P:%f Y:%f ", NWU_to_NED_pose.orientation(0), NWU_to_NED_pose.orientation(1), NWU_to_NED_pose.orientation(2));

	OffboardControl::publish_setpoint(msg);
}


/**
 * @brief Publish a trajectory setpoint
 * and pose message
 */
void OffboardControl::publish_setpoint(px4_msgs::msg::TrajectorySetpoint msg) const {

	publish_offboard_control_mode();

	orientation_t eul (
		0.0,
		0.0,
		-msg.yaw // NED to NWU
	);

	quat_t quat = eulToQuat(eul);

	if (_launch_with_debug > 0)
	{	
		auto pose_msg = geometry_msgs::msg::PoseStamped();
		pose_msg.header.stamp = this->now();
		pose_msg.header.frame_id = "world";
		pose_msg.pose.orientation.x = quat(0);
		pose_msg.pose.orientation.y = quat(1);
		pose_msg.pose.orientation.z = quat(2);
		pose_msg.pose.orientation.w = quat(3);
		pose_msg.pose.position.x = msg.x;
		pose_msg.pose.position.y = -msg.y; // NED to NWU
		pose_msg.pose.position.z = -msg.z; // NED to NWU

		_follow_pose_pub->publish(pose_msg);
	}

	_trajectory_setpoint_publisher->publish(msg);
}


/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1,
					      float param2) const {
	px4_msgs::msg::VehicleCommand msg{};
	msg.timestamp = _timestamp.load();
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;

	_vehicle_command_publisher->publish(msg);
}

int main(int argc, char* argv[]) {
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
