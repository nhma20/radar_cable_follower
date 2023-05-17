// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include "geometry.h"
#include "radar_cable_follower_msgs/msg/tracked_powerlines.hpp"

// Debug
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

 // MISC includes
#include <cstdlib>
#include <stdlib.h> 
#include <chrono>
#include <math.h> 
#include <cmath> 
#include <vector>
#include <deque>
#include <string>
#include <numeric>
#include <mutex>

// PCL includes
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/common/angles.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>





#define DEG_PER_RAD 57.2957795
#define RAD_PER_DEG 0.01745329
#define PI 3.14159265

using namespace std::chrono_literals;

//creates a RadarPCLFilter class that subclasses the generic rclcpp::Node base class.
class RadarPCLFilter : public rclcpp::Node
{
	public:
		RadarPCLFilter() : Node("radar_pcl_filter_node") {
			
			// Params
			this->declare_parameter<int>("concat_size", 200);
			this->get_parameter("leaf_size", _concat_size);

			this->declare_parameter<float>("leaf_size", 0.75);
			this->get_parameter("leaf_size", _leaf_size);

			this->declare_parameter<float>("model_thresh", 2.0);
			this->get_parameter("model_thresh", _model_thresh);

			this->declare_parameter<float>("ground_threshold", 1.5);
			this->get_parameter("ground_threshold", _ground_threshold);

			this->declare_parameter<float>("drone_threshold", 0.5);
			this->get_parameter("drone_threshold", _drone_threshold);

			this->declare_parameter<float>("cluster_crop_radius", 20);
			this->get_parameter("cluster_crop_radius", _cluster_crop_radius);

			this->declare_parameter<float>("line_model_parallel_angle_threshold", 15.0);
			this->get_parameter("line_model_parallel_angle_threshold", _line_model_parallel_angle_threshold);

			this->declare_parameter<float>("line_model_distance_threshold", 1.5);
			this->get_parameter("line_model_distance_threshold", _line_model_distance_thresh);

			this->declare_parameter<float>("line_model_inlier_threshold", 10.0);
			this->get_parameter("line_model_inlier_threshold", _line_model_inlier_thresh);

			this->declare_parameter<float>("line_model_pitch_threshold", 0.35);
			this->get_parameter("line_model_pitch_threshold", _line_model_pitch_thresh);

			this->declare_parameter<std::string>("voxel_or_time_concat", "voxel");
			this->get_parameter("voxel_or_time_concat", _voxel_or_time_concat);

			this->declare_parameter<std::string>("line_or_point_follow", "line");
			this->get_parameter("line_or_point_follow", _line_or_point_follow);

			this->declare_parameter<int>("point_follow_outlier_filter", 1);
			this->get_parameter("point_follow_outlier_filter", _point_follow_outlier_filter);

			this->declare_parameter<std::string>("sensor_upwards_or_downwards", "downwards");
			this->get_parameter("sensor_upwards_or_downwards", _sensor_upwards_or_downwards);

			this->declare_parameter<int>("add_crop_downsample_rate", 10);
			this->get_parameter("add_crop_downsample_rate", _add_crop_downsample_rate);

			this->declare_parameter<float>("tracking_update_ratio", 0.01);
			this->get_parameter("tracking_update_ratio", _tracking_update_ratio);

			this->declare_parameter<float>("tracking_update_euclid_dist", 1.5);
			this->get_parameter("tracking_update_euclid_dist", _tracking_update_euclid_dist);

			this->declare_parameter<int>("launch_with_debug", 1);
			this->get_parameter("launch_with_debug", _launch_with_debug);

			this->declare_parameter<float>("2d_direction_bin_threshold", 10.0);
			this->get_parameter("2d_direction_bin_threshold", _2d_direction_bin_threshold);

			this->declare_parameter<float>("hough_rho", 1);
			this->get_parameter("hough_rho", _hough_rho);

			this->declare_parameter<float>("hough_theta", PI/180);
			this->get_parameter("hough_theta", _hough_theta);

			this->declare_parameter<int>("hough_pixel_diameter", 4);
			this->get_parameter("hough_pixel_diameter", _hough_pixel_size);

			this->declare_parameter<int>("hough_minimum_inliers", 20);
			this->get_parameter("hough_minimum_inliers", _hough_minimum_inliers);

			this->declare_parameter<int>("hough_minimum_length", 40);
			this->get_parameter("hough_minimum_length", _hough_minimum_length);

			this->declare_parameter<int>("hough_maximum_gap", 20);
			this->get_parameter("hough_maximum_gap", _hough_maximum_gap);

			this->declare_parameter<std::string>("sliced_hough_highest_or_cluster", "highest");
			this->get_parameter("sliced_hough_highest_or_cluster", _sliced_Hough_highest_or_cluster);

			this->declare_parameter<float>("radar_elevation_fov", 40.0);
			this->get_parameter("radar_elevation_fov", _radar_elevation_fov);
			
			this->declare_parameter<float>("radar_azimuth_fov", 120.0);
			this->get_parameter("radar_azimuth_fov", _radar_azimuth_fov);

			this->declare_parameter<bool>("reset_global_point_cloud", false);
			this->get_parameter("reset_global_point_cloud", _reset_global_point_cloud);
			
			this->declare_parameter<int>("plus_or_minus_yaw", 1);
			this->get_parameter("plus_or_minus_yaw", _plus_or_minus_yaw);
			
			


			raw_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/iwr6843_pcl",	10,
			std::bind(&RadarPCLFilter::add_new_radar_pointcloud, this, std::placeholders::_1));

			output_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world_pcl", 10);

			input_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/input_pcl", 10);

			hough_line_pub = this->create_publisher<sensor_msgs::msg::Image>("/hough_line_img", 10);
			
			vis_tracked_powerlines_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/vis_powerlines_array", 10);

			tracked_powerlines_pub = this->create_publisher<radar_cable_follower_msgs::msg::TrackedPowerlines>("/tracked_powerlines", 10);

			debug_point = this->create_publisher<geometry_msgs::msg::PointStamped>("/debug_point", 10);
			debug_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("/debug_pose", 10);

			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

			geometry_msgs::msg::TransformStamped drone_tf;

			_timer_pl = this->create_wall_timer(33ms, 
				std::bind(&RadarPCLFilter::update_powerline_poses, this));

			_timer_tf = this->create_wall_timer(33ms, 
				std::bind(&RadarPCLFilter::update_tf, this));

			_timer_pcl = this->create_wall_timer(33ms, 
				std::bind(&RadarPCLFilter::powerline_detection, this));


			while(true) {

				try {

					drone_tf = tf_buffer_->lookupTransform("iwr6843_frame", "world", tf2::TimePointZero);

					RCLCPP_INFO(this->get_logger(), "Found transform drone->world");
					break;

				} catch(tf2::TransformException & ex) {

					RCLCPP_INFO(this->get_logger(), "Could not get transform world->drone, trying again...");
					_t_tries++;

					if( _t_tries > 100) {
						RCLCPP_FATAL(this->get_logger(), "Failed to get transform after 1000 tries.");
						throw std::exception();
					}
				}

				std::this_thread::sleep_for(std::chrono::milliseconds(50));

				pcl::PointCloud<pcl::PointXYZ>::Ptr tmp1_cloud (new pcl::PointCloud<pcl::PointXYZ>);
				_concat_cloud = tmp1_cloud;

				pcl::PointCloud<pcl::PointXYZ>::Ptr tmp2_cloud (new pcl::PointCloud<pcl::PointXYZ>);
				_pl_search_cloud = tmp2_cloud;

			}
		}

		~RadarPCLFilter() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down radar_pointcloud_filter node..");
		}

	private:
		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

		rclcpp::TimerBase::SharedPtr _timer_pl;
		rclcpp::TimerBase::SharedPtr _timer_tf;
		rclcpp::TimerBase::SharedPtr _timer_pcl;

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pointcloud_pub;

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr input_pointcloud_pub;

		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr hough_line_pub;
		rclcpp::Publisher<radar_cable_follower_msgs::msg::TrackedPowerlines>::SharedPtr tracked_powerlines_pub;
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr vis_tracked_powerlines_pub;

		rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr debug_point;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr debug_pose;

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pcl_subscription_;

		std::mutex _concat_cloud_mutex;
		std::mutex _drone_xyz_mutex;

		float _ground_threshold;
		float _drone_threshold;
		float _height_above_ground = 0;
		int _concat_size; 
		std::string _voxel_or_time_concat;
		std::string _line_or_point_follow;
		int _point_follow_outlier_filter;
		float _leaf_size;
		float _model_thresh;
		float _cluster_crop_radius;
		float _line_model_parallel_angle_threshold;
		float _line_model_distance_thresh;
		float _line_model_inlier_thresh;
		float _line_model_pitch_thresh;
		std::string _sensor_upwards_or_downwards;
		int _add_crop_downsample_rate;
		float _tracking_update_ratio;
		float _tracking_update_euclid_dist;
		int _launch_with_debug;
		float _2d_direction_bin_threshold;
		float _hough_theta;
		float _hough_rho;
		int _hough_pixel_size;
		int _hough_minimum_inliers;
		int _hough_maximum_gap;
		int _hough_minimum_length;
		std::string _sliced_Hough_highest_or_cluster;
		float _radar_elevation_fov;
		float _radar_azimuth_fov;
		bool _reset_global_point_cloud;
		int _plus_or_minus_yaw;

		float _global_powerline_2d_angle = 0.0;

		int _t_tries = 0;

		bool _first_message = false;

		float _powerline_world_yaw; // +90 to -90 deg relative to x-axis

		vector_t _t_xyz;
		quat_t _t_rot;

		std::deque<int> _concat_history; 

		pcl::PointCloud<pcl::PointXYZ>::Ptr _concat_cloud;
		pcl::PointCloud<pcl::PointXYZ>::Ptr _pl_search_cloud;

		std::vector<line_model_t> _line_models;

		void add_new_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		

		void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void filter_pointcloud(float ground_threshold, float drone_threshold, 
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void concatenate_poincloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
										pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points);

		void downsample_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
									pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_points);

		void fixed_size_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
										pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points);

		void crop_distant_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
									pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped);

		void direction_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
									pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

		void direction_extraction_2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
										Eigen::Vector3f &dir_axis);

		void direction_extraction_25D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
												Eigen::Vector3f &dir_axis);

		void direction_extraction_3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
												Eigen::Vector3f &dir_axis);

		std::vector<line_model_t> line_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);		


		std::vector<line_model_t> parallel_line_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
																Eigen::Vector3f axis);

		std::vector<line_model_t> follow_point_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
																float powerline_2d_angle);																						

		void create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::msg::PointCloud2 * pcl_msg);

		void powerline_detection();

		void update_powerline_poses();

		void update_tf();
};


void RadarPCLFilter::update_tf() {

	geometry_msgs::msg::TransformStamped t;

	try {
		if (tf_buffer_->canTransform("world", "iwr6843_frame", tf2::TimePointZero))	{
			t = tf_buffer_->lookupTransform("world","iwr6843_frame",tf2::TimePointZero);
		}
		else {
			RCLCPP_INFO(this->get_logger(), "Can not transform");
			return;
		}
	} catch (const tf2::TransformException & ex) {
		RCLCPP_INFO(this->get_logger(), "Could not transform: %s", ex.what());
		return;
	}

	_drone_xyz_mutex.lock(); {

		_t_xyz(0) = t.transform.translation.x;
		_t_xyz(1) = t.transform.translation.y;
		_t_xyz(2) = t.transform.translation.z;
		
	} _drone_xyz_mutex.unlock();

	_t_rot(0) = t.transform.rotation.x;
	_t_rot(1) = t.transform.rotation.y;
	_t_rot(2) = t.transform.rotation.z;
	_t_rot(3) = t.transform.rotation.w;

}


void RadarPCLFilter::update_powerline_poses() {

	if (_line_models.size() > 0)
	{

		static std::vector<point_t> new_point_vec;
		static std::vector<quat_t> new_quat_vec;
		static std::vector<id_point_t> prev_point_vec;
		static std::vector<quat_t> prev_quat_vec;
		static int id_count = 0;
		static point_t prev_drone_xyz;
		static point_t new_drone_xyz;

		auto pose_array_msg = geometry_msgs::msg::PoseArray();
		pose_array_msg.header = std_msgs::msg::Header();
		pose_array_msg.header.stamp = this->now();
		pose_array_msg.header.frame_id = "world";

		prev_drone_xyz = new_drone_xyz;

		plane_t proj_plane;

		// create plane from drone position and estimated powerline direction
		_drone_xyz_mutex.lock(); {
	

			new_drone_xyz(0) = _t_xyz(0);
			new_drone_xyz(1) = _t_xyz(1);
			new_drone_xyz(2) = -_t_xyz(2);


			proj_plane = create_plane(_line_models.at(0).quaternion, _t_xyz);

		} _drone_xyz_mutex.unlock();

		// project estimated poses onto projection plane
		for (size_t i = 0; i < _line_models.size(); i++)
		{	

			point_t proj_pl_point = projectPointOnPlane(_line_models.at(i).position, proj_plane);
			
			new_point_vec.push_back(proj_pl_point);

			new_quat_vec.push_back(_line_models.at(i).quaternion);

		}


		point_t drone_xyz_delta = new_drone_xyz - prev_drone_xyz;

		proj_plane = create_plane(_line_models.at(0).quaternion, drone_xyz_delta);

		point_t zero_point;
		zero_point(0) = 0.0;
		zero_point(1) = 0.0;
		zero_point(2) = 0.0;

		point_t proj_delta = projectPointOnPlane(zero_point, proj_plane);

		// RCLCPP_INFO(this->get_logger(),  "\nProj delta: \n X %f\t Y %f\t Z %f\n", 
		// 			proj_delta(0), proj_delta(1), proj_delta(2));


		this->get_parameter("tracking_update_euclid_dist", _tracking_update_euclid_dist);
		this->get_parameter("tracking_update_ratio", _tracking_update_ratio);
		float one_minus_ratio = 1.0 - _tracking_update_ratio;

		// look for matches to merge new and existing point
		for (size_t i = 0; i < new_point_vec.size(); i++)
		{
			bool merged = false;
			
			for (size_t j = 0; j < prev_point_vec.size(); j++)
			{				
				float dist_x = abs( (prev_point_vec.at(j).point(0) + proj_delta(0)) - new_point_vec.at(i)(0) );
				float dist_y = abs( (prev_point_vec.at(j).point(1) + proj_delta(1)) - new_point_vec.at(i)(1) );
				float dist_z = abs( (prev_point_vec.at(j).point(2) + proj_delta(2)) - new_point_vec.at(i)(2) );
				float euclid_dist = sqrt( pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_z, 2) );
				
				if (euclid_dist < _tracking_update_euclid_dist)
				{
					prev_point_vec.at(j).point(0) = _tracking_update_ratio * new_point_vec.at(i)(0) + one_minus_ratio * (prev_point_vec.at(j).point(0) + proj_delta(0)); //( new_point_vec.at(i)(0) + prev_point_vec.at(j).point(0) ) / 2.0f; //new_point_vec.at(i)(0);
					prev_point_vec.at(j).point(1) = _tracking_update_ratio * new_point_vec.at(i)(1) + one_minus_ratio * (prev_point_vec.at(j).point(1) + proj_delta(1)); //( new_point_vec.at(i)(1) + prev_point_vec.at(j).point(1) ) / 2.0f; //new_point_vec.at(i)(1);
					prev_point_vec.at(j).point(2) = _tracking_update_ratio * new_point_vec.at(i)(2) + one_minus_ratio * (prev_point_vec.at(j).point(2) + proj_delta(2)); //( new_point_vec.at(i)(2) + prev_point_vec.at(j).point(2) ) / 2.0f; //new_point_vec.at(i)(2);
					
					prev_quat_vec.at(j) = new_quat_vec.at(i);

					if (prev_point_vec.at(j).alive_count < 500) // param this
					{
						prev_point_vec.at(j).alive_count = prev_point_vec.at(j).alive_count + 2;
					}
					
					merged = true;

					break;
				}
			}
			
			// add new point if no match found
			if (merged == false)
			{
				id_point_t new_id_point;

				new_id_point.point(0) = new_point_vec.at(i)(0);
				new_id_point.point(1) = new_point_vec.at(i)(1);
				new_id_point.point(2) = new_point_vec.at(i)(2);
				new_id_point.id = id_count++;
				new_id_point.alive_count = 100;

				prev_point_vec.push_back(new_id_point);

				prev_quat_vec.push_back(new_quat_vec.at(i));
			}
		}

		// decrement all alive counters and remove ones at 0
		for (size_t i = prev_point_vec.size(); i > 0; i--)
		{
			int idx = i-1;
			prev_point_vec.at(idx).alive_count--;

			if (prev_point_vec.at(idx).alive_count < 1)
			{
				prev_point_vec.erase(prev_point_vec.begin()+idx);
				prev_quat_vec.erase(prev_quat_vec.begin()+idx);
			}
		}
		
		// publish message
		if (prev_point_vec.size() > 0)
		{		
			auto track_pose_array_msg = geometry_msgs::msg::PoseArray();
			track_pose_array_msg.header = std_msgs::msg::Header();
			track_pose_array_msg.header.stamp = this->now();
			track_pose_array_msg.header.frame_id = "world";

			auto tracked_powerlines_msg = radar_cable_follower_msgs::msg::TrackedPowerlines();
			tracked_powerlines_msg.header = std_msgs::msg::Header();
			tracked_powerlines_msg.header.stamp = this->now();
			tracked_powerlines_msg.header.frame_id = "world";

			int tracked_count = 0;

			for (size_t i = 0; i < prev_point_vec.size(); i++)
			{

				if (prev_point_vec.at(i).alive_count < 250)
				{
					continue;
				}

				tracked_count++;
				
				// if (_launch_with_debug)
				// {				
				// 	RCLCPP_INFO(this->get_logger(),  "\nPoint %d: \n X %f\n Y %f\n Z %f\n ID %d\n ALIVE %d\n", 
				// 		i, prev_point_vec.at(i).point(0), prev_point_vec.at(i).point(1), prev_point_vec.at(i).point(2), 
				// 		prev_point_vec.at(i).id, prev_point_vec.at(i).alive_count);
				// }
			
			
				auto track_pose_msg = geometry_msgs::msg::Pose();
				track_pose_msg.orientation.x = prev_quat_vec.at(i)(0);
				track_pose_msg.orientation.y = prev_quat_vec.at(i)(1);
				track_pose_msg.orientation.z = prev_quat_vec.at(i)(2);
				track_pose_msg.orientation.w = prev_quat_vec.at(i)(3);
				track_pose_msg.position.x = prev_point_vec.at(i).point(0);
				track_pose_msg.position.y = prev_point_vec.at(i).point(1);
				track_pose_msg.position.z = prev_point_vec.at(i).point(2);
				
				track_pose_array_msg.poses.push_back(track_pose_msg);

				tracked_powerlines_msg.poses.push_back(track_pose_msg);
				tracked_powerlines_msg.ids.push_back(prev_point_vec.at(i).id);
			}

			tracked_powerlines_msg.count = tracked_count;

			tracked_powerlines_pub->publish(tracked_powerlines_msg);

			if (_launch_with_debug > 0)
			{			
				vis_tracked_powerlines_pub->publish(track_pose_array_msg);
			}
		}
		
		
		new_point_vec.clear();
		new_quat_vec.clear();

	}
}


void RadarPCLFilter::add_new_radar_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

		if (msg->width < 1)
	{
		return;
	}

	homog_transform_t world_to_drone;

	// make transform drone->world
	_drone_xyz_mutex.lock(); {

		_height_above_ground = _t_xyz(2); ///t.transform.translation.z;

		world_to_drone = getTransformMatrix(_t_xyz, _t_rot);

	} _drone_xyz_mutex.unlock();

	// transform points in pointcloud

	pcl::PointCloud<pcl::PointXYZ>::Ptr local_points (new pcl::PointCloud<pcl::PointXYZ>);

	RadarPCLFilter::read_pointcloud(msg, local_points);

	pcl::PointCloud<pcl::PointXYZ>::Ptr world_points (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::transformPointCloud (*local_points, *world_points, world_to_drone);

	// filter ground and drone points 
	this->get_parameter("ground_threshold", _ground_threshold);
	this->get_parameter("drone_threshold", _drone_threshold);
	RadarPCLFilter::filter_pointcloud(_ground_threshold, _drone_threshold, world_points);

	if (world_points->size() < 1)
	{
		return;
	}

	auto pcl_msg = sensor_msgs::msg::PointCloud2();
	RadarPCLFilter::create_pointcloud_msg(world_points, &pcl_msg);
	input_pointcloud_pub->publish(pcl_msg);  

	_concat_cloud_mutex.lock(); {

        RadarPCLFilter::concatenate_poincloud(world_points, _concat_cloud);

    } _concat_cloud_mutex.unlock();

}


void RadarPCLFilter::create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, sensor_msgs::msg::PointCloud2 * pcl_msg) {

  // create PointCloud2 msg
	//https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	int pcl_size = cloud->size();
	auto pcl2_msg = sensor_msgs::msg::PointCloud2();
	const uint32_t POINT_STEP = 12;

	pcl_msg->header = std_msgs::msg::Header();
	pcl_msg->header.stamp = this->now();
	std::string frameID = "world";
	pcl_msg->header.frame_id = frameID;
	pcl_msg->fields.resize(3);
	pcl_msg->fields[0].name = 'x';
	pcl_msg->fields[0].offset = 0;
	pcl_msg->fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[0].count = 1;
	pcl_msg->fields[1].name = 'y';
	pcl_msg->fields[1].offset = 4;
	pcl_msg->fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[1].count = 1;
	pcl_msg->fields[2].name = 'z';
	pcl_msg->fields[2].offset = 8;
	pcl_msg->fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
	pcl_msg->fields[2].count = 1;

	if(pcl_size > 0){
		pcl_msg->data.resize(std::max((size_t)1, (size_t)pcl_size) * POINT_STEP, 0x00);
	} else {
        return;
    }

	pcl_msg->point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	pcl_msg->row_step = pcl_msg->data.size();//pcl_msg->point_step * pcl_msg->width; // only 1 row because unordered
	pcl_msg->height = 1;  // because unordered cloud
	pcl_msg->width = pcl_msg->row_step / POINT_STEP; // number of points in cloud
	pcl_msg->is_dense = false; // there may be invalid points

	// fill PointCloud2 msg data
	uint8_t *ptr = pcl_msg->data.data();

	for (size_t i = 0; i < (size_t)pcl_size; i++)
	{
		pcl::PointXYZ point = (*cloud)[i];

        *(reinterpret_cast<float*>(ptr + 0)) = point.x;
        *(reinterpret_cast<float*>(ptr + 4)) = point.y;
        *(reinterpret_cast<float*>(ptr + 8)) = point.z;
        ptr += POINT_STEP;
	}
	
}


std::vector<line_model_t> RadarPCLFilter::line_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in)  {

	std::vector<line_model_t> line_models;

	if ( cloud_in->size() < 2 ) {
		return line_models;
	}

	this->get_parameter("line_model_distance_threshold", _line_model_distance_thresh);
	this->get_parameter("line_model_inlier_threshold", _line_model_inlier_thresh);
	this->get_parameter("line_model_pitch_threshold", _line_model_pitch_thresh);

	std::vector<float> yaw_list;

	static pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	static pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	static pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_LINE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold ((float)_line_model_distance_thresh);

	pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::copyPointCloud(*cloud_in, *reduced_cloud);

	seg.setInputCloud (reduced_cloud);
	seg.segment (*inliers, *coefficients);


	line_model_t line_model;

	int count = 0;

	// Continue line extraction if first line model has pitch below threshold and inliers above threshold
	while (abs(coefficients->values[5]) < _line_model_pitch_thresh && 
			(int)inliers->indices.size() > (int)_line_model_inlier_thresh)
	{
		// scale factor for X and Y to compensate ignoring Z
		float z_factor = 1 / sqrt( pow(coefficients->values[3],2) + pow(coefficients->values[4],2) );
		// calculate yaw in world frame (+90 to -90 deg relative to X direction)
		float tmp_powerline_world_yaw = -1 * (abs(coefficients->values[3]) / coefficients->values[3]) * acos(abs(coefficients->values[3])) * z_factor;
		
		// break if difference between current and previous yaw is above 45 degrees (0.7854 rads)
		if ( count++ > 0 && abs(tmp_powerline_world_yaw - yaw_list.back()) > 0.7854 )
		{
			break;
		}

		yaw_list.push_back(tmp_powerline_world_yaw);
		
		_powerline_world_yaw = tmp_powerline_world_yaw;
		// RCLCPP_INFO(this->get_logger(),  "Powerline yaw: %f", (_powerline_world_yaw*DEG_PER_RAD));
		
		point_t pl_position(
			coefficients->values[0],
			coefficients->values[1], 
			coefficients->values[2]
		);

		orientation_t temp_eul(
			0,
			0,
			tmp_powerline_world_yaw
		);

		quat_t pl_quat = eulToQuat(temp_eul);

        line_model.position = pl_position;
        line_model.quaternion = pl_quat;

		line_models.push_back(line_model);

		extract.setInputCloud(reduced_cloud);

		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*reduced_cloud); 
		

		if ((int)reduced_cloud->size() < (int)_line_model_inlier_thresh)
		{
			break;
		}
		
		seg.setInputCloud (reduced_cloud);
		seg.segment (*inliers, *coefficients);
	}		

	if (line_models.size() > 0)
	{
		return line_models;
	} else {
		return _line_models; // to not remove last direction if none is found
	}
	
		

}


std::vector<line_model_t> RadarPCLFilter::parallel_line_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
																	Eigen::Vector3f axis)  {

	std::vector<line_model_t> line_models;

	if ( cloud_in->size() < 2 ) {
		return line_models;
	}

	this->get_parameter("line_model_parallel_angle_threshold", _line_model_parallel_angle_threshold);
	this->get_parameter("line_model_distance_threshold", _line_model_distance_thresh);
	this->get_parameter("line_model_inlier_threshold", _line_model_inlier_thresh);

	std::vector<float> yaw_list;

	static pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	static pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	static pcl::SACSegmentation<pcl::PointXYZ> seg;

	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_PARALLEL_LINE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold ((float)_line_model_distance_thresh);
	seg.setAxis( (-1 * axis ) ); // needs to be negated for some reason?

	seg.setEpsAngle(pcl::deg2rad((double)_line_model_parallel_angle_threshold));//(_line_model_parallel_angle_threshold/DEG_PER_RAD)); //90/DEG_PER_RAD);//

	pcl::PointCloud<pcl::PointXYZ>::Ptr reduced_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	pcl::copyPointCloud(*cloud_in, *reduced_cloud);

	seg.setInputCloud (reduced_cloud);
	seg.segment (*inliers, *coefficients);

	if ((int)inliers->indices.size() < (int)_line_model_inlier_thresh)
	{
		return _line_models;
	}
	
	line_model_t line_model;
	int count = 0;

	// Continue line extraction if first line model has pitch below threshold and inliers above threshold
	while (abs(coefficients->values[5]) < _line_model_pitch_thresh && 
			(int)inliers->indices.size() > (int)_line_model_inlier_thresh)
	{

		// RCLCPP_INFO(this->get_logger(),  "Axis: \n X %f\n Y %f\n Z %f\n", coefficients->values[3], coefficients->values[4], coefficients->values[5]);

		// scale factor for X and Y to compensate ignoring Z
		float z_factor = 1 / sqrt( pow(coefficients->values[3],2) + pow(coefficients->values[4],2) );
		// calculate yaw in world frame (+90 to -90 deg relative to X direction)
		float tmp_powerline_world_yaw;
		if(_sensor_upwards_or_downwards == "downwards")
		{
			tmp_powerline_world_yaw = -1 * (abs(coefficients->values[3]) / coefficients->values[3]) * acos(abs(coefficients->values[3])) * z_factor;
		}
		else
		{	
			tmp_powerline_world_yaw = (abs(coefficients->values[3]) / coefficients->values[3]) * acos(abs(coefficients->values[3])) * z_factor;
		}
		// break if difference between current and previous yaw is above 45 degrees (0.7854 rads)
		if ( count++ > 0 && abs(tmp_powerline_world_yaw - yaw_list.back()) > 0.7854 )
		{
			break;
		}

		yaw_list.push_back(tmp_powerline_world_yaw);
		
		_powerline_world_yaw = tmp_powerline_world_yaw;
		// RCLCPP_INFO(this->get_logger(),  "Powerline yaw: %f", (_powerline_world_yaw*DEG_PER_RAD));
		
		point_t pl_position(
			coefficients->values[0],
			coefficients->values[1], 
			coefficients->values[2]
		);

		this->get_parameter("plus_or_minus_yaw", _plus_or_minus_yaw);

		orientation_t temp_eul(
			0,
			0,
			tmp_powerline_world_yaw * _plus_or_minus_yaw
		);

		quat_t pl_quat = eulToQuat(temp_eul);

	    line_model.position = pl_position;
        line_model.quaternion = pl_quat;

		line_models.push_back(line_model);

		extract.setInputCloud(reduced_cloud);

		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*reduced_cloud); 
		

		if ((int)reduced_cloud->size() < _line_model_inlier_thresh)
		{
			break;
		}
		
		seg.setInputCloud (reduced_cloud);
		seg.segment (*inliers, *coefficients);
	}		

	if (line_models.size() > 0)
	{
		return line_models;
	} else {
		return _line_models; // to not remove last direction if none is found
	}
	
		

}

std::vector<line_model_t> RadarPCLFilter::follow_point_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
																	float powerline_2d_angle) {
	
	if (cloud_in->size() > 10)
	{
		this->get_parameter("point_follow_outlier_filter", _point_follow_outlier_filter);

		pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);

		if (_point_follow_outlier_filter > 0)
		{
		
			// filter cloud for outliers
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud (cloud_in);
			sor.setMeanK (5); // param this
			sor.setStddevMulThresh (6.0); // and this?
			sor.filter (*filtered_cloud);

		}

		// find highest point
		pcl::PointXYZ max_z_point;
		pcl::PointXYZ second_max_z_point;

		for (size_t i = 0; i < filtered_cloud->size (); ++i)
		{
			// Check if the point is invalid
			if (!std::isfinite ((*filtered_cloud)[i].x) || 
				!std::isfinite ((*filtered_cloud)[i].y) || 
				!std::isfinite ((*filtered_cloud)[i].z))
				continue;

			if ((*filtered_cloud)[i].z > max_z_point.z) {
				second_max_z_point = max_z_point;
				max_z_point = (*filtered_cloud)[i];
			}
		}

	// create line model from hough angle and second highest point XYZ
	orientation_t hough_angle (
		0,
		0,
		powerline_2d_angle
	);

	std::vector<line_model_t> line_model_vec;
	line_model_t line_model;

	line_model.position(0) = second_max_z_point.x;
	line_model.position(1) = second_max_z_point.y;
	line_model.position(2) = second_max_z_point.z;

	line_model.quaternion = eulToQuat(hough_angle);

	line_model_vec.push_back(line_model);

	_line_models = line_model_vec;

	}

	return _line_models; // only update if new data

}

void RadarPCLFilter::direction_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
											pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_LINE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold ((float)_model_thresh);

	seg.setInputCloud (cloud_in);
	seg.segment (*inliers, *coefficients);

	pcl::copyPointCloud (*cloud_in, *inliers, *cloud_filtered);

	if ( abs(coefficients->values[5]) < 0.25)
	{
		float z_factor = 1 / sqrt( pow(coefficients->values[3],2) + pow(coefficients->values[4],2) );

		_powerline_world_yaw = -1 * (abs(coefficients->values[3]) / coefficients->values[3]) * acos(abs(coefficients->values[3])) * z_factor;

		// RCLCPP_INFO(this->get_logger(),  "Powerline yaw: %f", (_powerline_world_yaw*DEG_PER_RAD));
	
		orientation_t yaw_eul (
			0.0,
			0.0,
			_powerline_world_yaw
		);

		quat_t yaw_quat = eulToQuat(yaw_eul);


		auto track_pose_array_msg = geometry_msgs::msg::PoseArray();
		track_pose_array_msg.header = std_msgs::msg::Header();
		track_pose_array_msg.header.stamp = this->now();
		track_pose_array_msg.header.frame_id = "world";

		auto track_pose_msg = geometry_msgs::msg::Pose();
		track_pose_msg.orientation.x = yaw_quat(0);
		track_pose_msg.orientation.y = yaw_quat(1);
		track_pose_msg.orientation.z = yaw_quat(2);
		track_pose_msg.orientation.w = yaw_quat(3);

		_drone_xyz_mutex.lock(); {

			track_pose_msg.position.x = _t_xyz(0);
			track_pose_msg.position.y = _t_xyz(1);
			track_pose_msg.position.z = _t_xyz(2);
			
		} _drone_xyz_mutex.unlock();
		
		track_pose_array_msg.poses.push_back(track_pose_msg);

		if (_launch_with_debug > 0)
		{			
			vis_tracked_powerlines_pub->publish(track_pose_array_msg);
		}

	}	
}

void RadarPCLFilter::direction_extraction_2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
												Eigen::Vector3f &dir_axis) {


	int img_size = _cluster_crop_radius*10*2;
	cv::Mat img(img_size, img_size, CV_8UC1, cv::Scalar(0));

	this->get_parameter("hough_pixel_diameter", _hough_pixel_size);

	// project point cloud onto ground and create mat from points
	for (size_t i = 0; i < cloud_in->size(); i++)
	{
		pcl::PointXYZ point = (*cloud_in)[i];

		float y_tmp;
		float x_tmp;

		// x and y swapped to align image with world x y (up = +x, left = +y)
		_drone_xyz_mutex.lock(); {
			y_tmp = roundf( img_size/2 - 10.0*(point.x - _t_xyz(0)) );
			x_tmp = roundf( img_size/2 - 10.0*(point.y - _t_xyz(1)) );
		} _drone_xyz_mutex.unlock();

		if (_hough_pixel_size > 1)
		{
			cv::circle(img, cv::Point(x_tmp,y_tmp),round(_hough_pixel_size/2), cv::Scalar(255,255,255), -1, 8,0);
		}
		else
		{
			img.at<uchar>(y_tmp, x_tmp) = 255;
		}
	}

	// Probabilistic Line Transform
    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
	std::vector<float> tmp_angles;

	this->get_parameter("hough_rho", _hough_rho);
	this->get_parameter("hough_theta", _hough_theta);
	this->get_parameter("hough_minimum_inliers", _hough_minimum_inliers);
	this->get_parameter("hough_maximum_gap", _hough_maximum_gap);
	this->get_parameter("hough_minimum_length", _hough_minimum_length);

	// cv::GaussianBlur(img, img, cv::Size(9, 9), 2, 2 );
    cv::HoughLinesP(img, linesP, _hough_rho, _hough_theta, _hough_minimum_inliers, _hough_maximum_gap, _hough_minimum_length); // rho res pixels, theta res rads, min intersections, min line length, max line gap
    
	float tmp_pl_world_yaw = -0.0;

    for( size_t i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
		// Draw the lines
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(127,127,127), 2, cv::LINE_AA);

		// calculate angle (+-90 deg around world X-axis)
		float diff_x = (float)l[0] - (float)l[2];
		float diff_y = (float)l[3] - (float)l[1];

		float tmp_atan2_swapped = atan2f(diff_x, diff_y); // args swapped to rot 90deg

		// flip sign to align with right hand rule direction
		tmp_pl_world_yaw = -1 * tmp_atan2_swapped;
		
		// flip angle if outside +-90 deg range
		if (tmp_pl_world_yaw > PI/2)
		{
			tmp_pl_world_yaw = tmp_pl_world_yaw - PI;
		}
		else if (tmp_pl_world_yaw < -PI/2)
		{
			tmp_pl_world_yaw = tmp_pl_world_yaw + PI;
		}

		// check if angle value is broken
		if (isnan(tmp_pl_world_yaw) || tmp_pl_world_yaw == 0.0 || tmp_pl_world_yaw != tmp_pl_world_yaw)
		{
			continue;
		}


		tmp_angles.push_back(tmp_pl_world_yaw);
	}

	// RCLCPP_INFO(this->get_logger(),  "Angles: %d\n", tmp_angles.size());

	static float powerline_2d_angle = 0.0; 

	// find most popular hough line angle
	if (tmp_angles.size() > 1)
	{
		this->get_parameter("2d_direction_bin_threshold", _2d_direction_bin_threshold);

		sort(tmp_angles.begin(), tmp_angles.end());

		std::vector<std::vector<float>> clusters;
		static float eps = (float)pcl::deg2rad((double)_2d_direction_bin_threshold); // threshold between bins
		float curr_angle = tmp_angles.at(0);

		int over_85_count = 0;
		int under_minus_85_count = 0;

		if (curr_angle > 1.48352986)
		{
			over_85_count++;
		}
		else if (curr_angle < -1.48352986)
		{
			under_minus_85_count++;
		}		

		std::vector<float> curr_cluster;
		curr_cluster.push_back(curr_angle);

		// divide found hough angles into clusters
		for (size_t i = 1; i < tmp_angles.size(); i++)
		{		
			if (tmp_angles.at(i) > 1.48352986) // 85 deg
			{
				over_85_count++;
			}
			else if (tmp_angles.at(i) < -1.48352986) // -85 deg
			{
				under_minus_85_count++;
			}
				
			if( abs(tmp_angles.at(i) - curr_angle) <= eps ) 
			{
				curr_cluster.push_back(tmp_angles.at(i));
			} 
			else 
			{
				clusters.push_back(curr_cluster);
				curr_cluster.clear();
				curr_cluster.push_back(tmp_angles.at(i));
			}
			curr_angle = tmp_angles.at(i);
		}
		clusters.push_back(curr_cluster);


		// if both >85 and <-85 measurements exist, combine into one bin to fix jump
		if (under_minus_85_count > 0 && over_85_count > 0)
		{
			if (under_minus_85_count > over_85_count)
			{
				if(clusters.at((clusters.size()-1)).size() > (size_t)over_85_count)
				{
					for (size_t i = 0; i < (size_t)over_85_count; i++)
					{
						clusters.at(((int)clusters.size()-1)).erase((clusters.at(((int)clusters.size()-1)).end()-1));
						clusters.at(0).push_back(-1.56905099754); // -89.99 deg
					}
				} 
				else
				{
					clusters.erase(clusters.end());

					for (int i = 0; i < over_85_count; i++)
					{
						clusters.at(0).push_back(-1.56905099754); // -89.99 deg
					}
				}
			}

			if (under_minus_85_count <= over_85_count)
			{
				if(clusters.at(0).size() > (size_t)under_minus_85_count)
				{
					for (size_t i = 0; i < (size_t)under_minus_85_count; i++)
					{
						clusters.at(0).erase((clusters.at(0).begin()));
						clusters.at((clusters.size()-1)).push_back(1.56905099754); // 89.99 deg
					}
				} 
				else
				{
					clusters.erase(clusters.begin());

					for (int i = 0; i < under_minus_85_count; i++)
					{
						clusters.at((clusters.size()-1)).push_back(1.56905099754); // 89.99 deg
					}
				}
			}
		}
		
		
		// print clusters
		RCLCPP_INFO(this->get_logger(),  "Angle values:");
		for (size_t i = 0; i < clusters.size(); i++)
		{
			RCLCPP_INFO(this->get_logger(),  "Cluster %d:", i);
			for (size_t j = 0; j < clusters.at(i).size(); j++)
			{
				RCLCPP_INFO(this->get_logger(),  "%f \t", clusters.at(i).at(j));
			}
		}
		RCLCPP_INFO(this->get_logger(),  "over 85: %d", over_85_count);
		RCLCPP_INFO(this->get_logger(),  "under -85: %d", under_minus_85_count);


		// find biggest cluster
		int biggest_cluster = -1;
		int biggest_cluster_idx = -1;
		int second_biggest_cluster = -2;
		for (size_t i = 0; i < clusters.size(); i++)
		{

			if ((int)clusters.at(i).size() > biggest_cluster)
			{
				biggest_cluster = (int)clusters.at(i).size();
				biggest_cluster_idx = (int)i;
			}
			else 
			{
				if ((int)clusters.at(i).size() > second_biggest_cluster)
				{
					second_biggest_cluster = (int)clusters.at(i).size();
				}
			}
		}


		if (biggest_cluster == second_biggest_cluster || biggest_cluster_idx < 0)
		{
			// empty or tie, no clear hough direction = keep previous powerline angle
			powerline_2d_angle = powerline_2d_angle;
		}
		else
		{
			// average cluster with most votes
			float count = (float)clusters.at(biggest_cluster_idx).size();
			float sum = 0.0;

			for (size_t i = 0; i < clusters.at(biggest_cluster_idx).size(); i++)
			{
				sum += clusters.at(biggest_cluster_idx).at(i);
			}

			powerline_2d_angle = 0.8*powerline_2d_angle + 0.2*(sum / count); // simple low pass filter
		}
	}


	// update direction axis that is used for 3D parallel line fit
	dir_axis(0) = cos(powerline_2d_angle);
	dir_axis(1) = sin(powerline_2d_angle);
	dir_axis(2) = 0;

	if (_launch_with_debug > 0)
	{	
		std::string txt_angle = std::to_string((int)roundf(powerline_2d_angle*DEG_PER_RAD))+"*";
	
		cv::putText(img, //target image
				txt_angle, //text
				cv::Point((img.cols / 2)-15, img.rows-25), //bottom-center position
				cv::FONT_HERSHEY_DUPLEX,
				1.0,
				cv::Scalar(255,255,255), //font color
				2);

		sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img).toImageMsg();

		hough_line_pub->publish(*msg.get());
	}
}

void RadarPCLFilter::direction_extraction_25D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
												Eigen::Vector3f &dir_axis) {

	if (cloud_in->size() < 2)
	{
		return;
	}
	

	pcl::RandomSample<pcl::PointXYZ> sampler;
	sampler.setInputCloud (cloud_in);
	sampler.setSeed (std::rand ());
	sampler.setSample((unsigned int)(100));
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	sampler.filter(*sample_cloud);

	std::vector<float> z_coords;

	for (size_t i = 0; i < sample_cloud->size(); i++)
	{
		z_coords.push_back(sample_cloud->at(i).z);
	}

	sort(z_coords.begin(), z_coords.end());

	static float avg_height = 0.0;

	this->get_parameter("sliced_hough_highest_or_cluster", _sliced_Hough_highest_or_cluster);

	if (_sliced_Hough_highest_or_cluster == "highest")
	{
		float second_highest_z = z_coords.at((z_coords.size()-2));
		avg_height = 0.5*avg_height + 0.5*second_highest_z; // simple low pass filter
	}
	else
	{
		std::vector<std::vector<float>> clusters;
		static float eps = 0.25; // threshold between bins
		float curr_angle = z_coords.at(0);

		std::vector<float> curr_cluster;
		curr_cluster.push_back(curr_angle);

		// divide sampled Z coordinates into clusters
		for (size_t i = 1; i < z_coords.size(); i++)
		{				
			if( abs(z_coords.at(i) - curr_angle) <= eps ) 
			{
				curr_cluster.push_back(z_coords.at(i));
			} 
			else 
			{
				clusters.push_back(curr_cluster);
				curr_cluster.clear();
				curr_cluster.push_back(z_coords.at(i));
			}
			curr_angle = z_coords.at(i);
		}
		clusters.push_back(curr_cluster);

			
		// // print clusters
		// RCLCPP_INFO(this->get_logger(),  "Angle values:");
		// for (size_t i = 0; i < clusters.size(); i++)
		// {
		// 	RCLCPP_INFO(this->get_logger(),  "Cluster %d:", i);
		// 	for (size_t j = 0; j < clusters.at(i).size(); j++)
		// 	{
		// 		RCLCPP_INFO(this->get_logger(),  "%f \t", clusters.at(i).at(j));
		// 	}
		// }


		// find highest cluster
		int highest_cluster_idx = -1;
		float highest_z = -1;
		// int second_biggest_cluster = -2;
		for (size_t i = 0; i < clusters.size(); i++)
		{
			if ((int)clusters.at(i).size() > 2 && clusters.at(i).at(clusters.at(i).size()-1) > highest_z)
			{
				highest_cluster_idx = (int)i;
				highest_z = clusters.at(i).at(clusters.at(i).size()-1);
			}
		}

		if(highest_cluster_idx == -1)
		{
			return;
		}

		// average heighest cluster
		float count = (float)clusters.at(highest_cluster_idx).size();
		float sum = 0.0;
		for (size_t i = 0; i < clusters.at(highest_cluster_idx).size(); i++)
		{
			sum += clusters.at(highest_cluster_idx).at(i);
		}

		avg_height = 0.5*avg_height + 0.5*(sum / count); // simple low pass filter
	}


	// RCLCPP_INFO(this->get_logger(),  "Highest cluster: %d", highest_cluster_idx);
	// RCLCPP_INFO(this->get_logger(),  "Highest cluster size: %d", highest_cluster);
	// RCLCPP_INFO(this->get_logger(),  "Highest cluster height: %f", avg_height);

	// make big enough to clear everything
	float minX = -10000;
	float minY = -10000;
	float maxX = 10000;
	float maxY = 10000;

	// filter away points higher and lower than highest cluster average
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(minX, minY, (avg_height-1.5), 1.0));
	boxFilter.setMax(Eigen::Vector4f(maxX, maxY, (avg_height+1.5), 1.0));
	boxFilter.setInputCloud(cloud_in);
	boxFilter.filter(*cloud_cropped);

	
	int img_size = _cluster_crop_radius*10*2;
	cv::Mat img(img_size, img_size, CV_8UC1, cv::Scalar(0));

	this->get_parameter("hough_pixel_diameter", _hough_pixel_size);

	// project point cloud onto ground and create mat from points
	for (size_t i = 0; i < cloud_cropped->size(); i++)
	{
		pcl::PointXYZ point = (*cloud_cropped)[i];

		float y_tmp;
		float x_tmp;

		// x and y swapped to align image with world x y (up = +x, left = +y)
		_drone_xyz_mutex.lock(); {
			y_tmp = roundf( img_size/2 - 10.0*(point.x - _t_xyz(0)) );
			x_tmp = roundf( img_size/2 - 10.0*(point.y - _t_xyz(1)) );
		} _drone_xyz_mutex.unlock();

		if (_hough_pixel_size > 1)
		{
			cv::circle(img, cv::Point(x_tmp,y_tmp),round(_hough_pixel_size/2), cv::Scalar(255,255,255), -1, 8,0);
		}
		else
		{
			img.at<uchar>(y_tmp, x_tmp) = 255;
		}
	}


	// Probabilistic Line Transform
    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
	std::vector<float> tmp_angles;

	this->get_parameter("hough_rho", _hough_rho);
	this->get_parameter("hough_theta", _hough_theta);
	this->get_parameter("hough_minimum_inliers", _hough_minimum_inliers);
	this->get_parameter("hough_maximum_gap", _hough_maximum_gap);
	this->get_parameter("hough_minimum_length", _hough_minimum_length);

	// cv::GaussianBlur(img, img, cv::Size(9, 9), 2, 2 );
    cv::HoughLinesP(img, linesP, _hough_rho, _hough_theta, _hough_minimum_inliers, _hough_maximum_gap, _hough_minimum_length); // rho res pixels, theta res rads, min intersections, min line length, max line gap
    
	float tmp_pl_world_yaw = -0.0;

    for( size_t i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
		// Draw the lines
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(127,127,127), 2, cv::LINE_AA);

		// calculate angle (+-90 deg around world X-axis)
		float diff_x = (float)l[0] - (float)l[2];
		float diff_y = (float)l[3] - (float)l[1];

		float tmp_atan2_swapped = atan2f(diff_x, diff_y); // args swapped to rot 90deg

		// flip sign to align with right hand rule direction
		tmp_pl_world_yaw = -1 * tmp_atan2_swapped;
		
		// flip angle if outside +-90 deg range
		if (tmp_pl_world_yaw > PI/2)
		{
			tmp_pl_world_yaw = tmp_pl_world_yaw - PI;
		}
		else if (tmp_pl_world_yaw < -PI/2)
		{
			tmp_pl_world_yaw = tmp_pl_world_yaw + PI;
		}

		// check if angle value is broken
		if (isnan(tmp_pl_world_yaw) || tmp_pl_world_yaw == 0.0 || tmp_pl_world_yaw != tmp_pl_world_yaw)
		{
			continue;
		}


		tmp_angles.push_back(tmp_pl_world_yaw);
	}

	// RCLCPP_INFO(this->get_logger(),  "Angles: %d\n", tmp_angles.size());

	static float powerline_2d_angle = 0.0; 

	// find most popular hough line angle
	if (tmp_angles.size() > 1)
	{
		this->get_parameter("2d_direction_bin_threshold", _2d_direction_bin_threshold);

		sort(tmp_angles.begin(), tmp_angles.end());

		std::vector<std::vector<float>> clusters;
		static float eps = (float)pcl::deg2rad((double)_2d_direction_bin_threshold); // threshold between bins
		float curr_angle = tmp_angles.at(0);

		int over_85_count = 0;
		int under_minus_85_count = 0;

		if (curr_angle > 1.48352986)
		{
			over_85_count++;
		}
		else if (curr_angle < -1.48352986)
		{
			under_minus_85_count++;
		}		

		std::vector<float> curr_cluster;
		curr_cluster.push_back(curr_angle);

		// divide found hough angles into clusters
		for (size_t i = 1; i < tmp_angles.size(); i++)
		{		
			if (tmp_angles.at(i) > 1.48352986) // 85 deg
			{
				over_85_count++;
			}
			else if (tmp_angles.at(i) < -1.48352986) // -85 deg
			{
				under_minus_85_count++;
			}
				
			if( abs(tmp_angles.at(i) - curr_angle) <= eps ) 
			{
				curr_cluster.push_back(tmp_angles.at(i));
			} 
			else 
			{
				clusters.push_back(curr_cluster);
				curr_cluster.clear();
				curr_cluster.push_back(tmp_angles.at(i));
			}
			curr_angle = tmp_angles.at(i);
		}
		clusters.push_back(curr_cluster);


		// if both >85 and <-85 measurements exist, combine into one bin to fix jump
		if (under_minus_85_count > 0 && over_85_count > 0)
		{
			if (under_minus_85_count > over_85_count)
			{
				if(clusters.at((clusters.size()-1)).size() > (size_t)over_85_count)
				{
					for (size_t i = 0; i < (size_t)over_85_count; i++)
					{
						clusters.at(((int)clusters.size()-1)).erase((clusters.at(((int)clusters.size()-1)).end()-1));
						clusters.at(0).push_back(-1.56905099754); // -89.99 deg
					}
				} 
				else
				{
					clusters.erase(clusters.end());

					for (int i = 0; i < over_85_count; i++)
					{
						clusters.at(0).push_back(-1.56905099754); // -89.99 deg
					}
				}
			}

			if (under_minus_85_count <= over_85_count)
			{
				if(clusters.at(0).size() > (size_t)under_minus_85_count)
				{
					for (size_t i = 0; i < (size_t)under_minus_85_count; i++)
					{
						clusters.at(0).erase((clusters.at(0).begin()));
						clusters.at((clusters.size()-1)).push_back(1.56905099754); // 89.99 deg
					}
				} 
				else
				{
					clusters.erase(clusters.begin());

					for (int i = 0; i < under_minus_85_count; i++)
					{
						clusters.at((clusters.size()-1)).push_back(1.56905099754); // 89.99 deg
					}
				}
			}
		}
		
		
		// // print clusters
		// RCLCPP_INFO(this->get_logger(),  "Angle values:");
		// for (size_t i = 0; i < clusters.size(); i++)
		// {
		// 	RCLCPP_INFO(this->get_logger(),  "Cluster %d:", i);
		// 	for (size_t j = 0; j < clusters.at(i).size(); j++)
		// 	{
		// 		RCLCPP_INFO(this->get_logger(),  "%f \t", clusters.at(i).at(j));
		// 	}
		// }
		// RCLCPP_INFO(this->get_logger(),  "over 85: %d", over_85_count);
		// RCLCPP_INFO(this->get_logger(),  "under -85: %d", under_minus_85_count);


		// find biggest cluster
		int biggest_cluster = -1;
		int biggest_cluster_idx = -1;
		int second_biggest_cluster = -2;
		for (size_t i = 0; i < clusters.size(); i++)
		{

			if ((int)clusters.at(i).size() > biggest_cluster)
			{
				biggest_cluster = (int)clusters.at(i).size();
				biggest_cluster_idx = (int)i;
			}
			else 
			{
				if ((int)clusters.at(i).size() > second_biggest_cluster)
				{
					second_biggest_cluster = (int)clusters.at(i).size();
				}
			}
		}


		if (biggest_cluster == second_biggest_cluster || biggest_cluster_idx < 0)
		{
			// empty or tie, no clear hough direction = keep previous powerline angle
			powerline_2d_angle = powerline_2d_angle;
		}
		else
		{
			// average cluster with most votes
			float count = (float)clusters.at(biggest_cluster_idx).size();
			float sum = 0.0;

			for (size_t i = 0; i < clusters.at(biggest_cluster_idx).size(); i++)
			{
				sum += clusters.at(biggest_cluster_idx).at(i);
			}

			powerline_2d_angle = 0.75*powerline_2d_angle + 0.25*(sum / count); // simple low pass filter
		}
	}

	// update direction axis that is used for 3D parallel line fit
	dir_axis(0) = cos(powerline_2d_angle);
	dir_axis(1) = sin(powerline_2d_angle);
	dir_axis(2) = 0;

	_global_powerline_2d_angle = powerline_2d_angle;

	if (_launch_with_debug > 0)
	{	
		std::string txt_angle = std::to_string((int)roundf(powerline_2d_angle*DEG_PER_RAD))+"*";
	
		cv::putText(img, //target image
				txt_angle, //text
				cv::Point((img.cols / 2)-15, img.rows-25), //bottom-center position
				cv::FONT_HERSHEY_DUPLEX,
				1.0,
				cv::Scalar(255,255,255), //font color
				2);

		sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img).toImageMsg();

		hough_line_pub->publish(*msg.get());
	}
}

void RadarPCLFilter::direction_extraction_3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
												Eigen::Vector3f &dir_axis) {

	if (cloud_in->size() < 1)
	{
		return;
	}
	

	int img_size = _cluster_crop_radius*10*2;
	cv::Mat img(img_size, img_size, CV_8UC1, cv::Scalar(0));

	this->get_parameter("hough_pixel_diameter", _hough_pixel_size);

	// project point cloud onto ground and create mat from points
	for (size_t i = 0; i < cloud_in->size(); i++)
	{
		pcl::PointXYZ point = (*cloud_in)[i];

		float y_tmp;
		float x_tmp;

		// x and y swapped to align image with world x y (up = +x, left = +y)
		_drone_xyz_mutex.lock(); {
			y_tmp = roundf( img_size/2 - 10.0*(point.x - _t_xyz(0)) );
			x_tmp = roundf( img_size/2 - 10.0*(point.y - _t_xyz(1)) );
		} _drone_xyz_mutex.unlock();

		if (_hough_pixel_size > 1)
		{
			cv::circle(img, cv::Point(x_tmp,y_tmp),round(_hough_pixel_size/2), cv::Scalar(255,255,255), -1, 8,0);
		}
		else
		{
			img.at<uchar>(y_tmp, x_tmp) = 255;
		}
	}

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud (cloud_in);

	pcl::RandomSample<pcl::PointXYZ> sampler;
	sampler.setInputCloud (cloud_in);
	sampler.setSeed (std::rand ());
	sampler.setSample((unsigned int)(10000));
	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	sampler.filter(*sample_cloud);

	std::vector<float> tmp_angles;
	float tmp_pl_world_yaw = -0.0;

	RCLCPP_INFO(this->get_logger(),  "Atan2:");
	for (size_t i = 0; i < sample_cloud->size(); i++)
	{
		pcl::PointXYZ searchPoint = sample_cloud->at(i);

		// RCLCPP_INFO(this->get_logger(), "\n\nSearch point:\nX: %f\nY: %f\nZ: %f\n", searchPoint.x , searchPoint.y, searchPoint.z);
		int K = 2;

		std::vector<int> pointIdxKNNSearch(K);
		std::vector<float> pointKNNSquaredDistance(K);

		if ( kdtree.nearestKSearch (searchPoint, K, pointIdxKNNSearch, pointKNNSquaredDistance) > 0 )
		{
			float orig_x = sample_cloud->at(i).x;
			float orig_y = sample_cloud->at(i).y;

			float nn_x = (*cloud_in)[ pointIdxKNNSearch[1] ].x;
			float nn_y = (*cloud_in)[ pointIdxKNNSearch[1] ].y;

			// calculate angle (+-90 deg around world X-axis)
			float diff_x = orig_x - nn_x; //(float)l[0] - (float)l[2];
			float diff_y = orig_y - nn_y; //(float)l[3] - (float)l[1];

			float tmp_atan2 = atan2f(diff_y, diff_x);
			RCLCPP_INFO(this->get_logger(),  "%f", tmp_atan2);

			float tmp_atan2_swapped = atan2f(diff_x, diff_y); // args swapped to rot 90deg


			// flip sign to align with right hand rule direction
			tmp_pl_world_yaw = -1 * tmp_atan2_swapped;
			
			// flip angle if outside +-90 deg range
			if (tmp_pl_world_yaw > PI/2)
			{
				tmp_pl_world_yaw = tmp_pl_world_yaw - PI;
			}
			else if (tmp_pl_world_yaw < -PI/2)
			{
				tmp_pl_world_yaw = tmp_pl_world_yaw + PI;
			}


			// check if angle value is broken
			if (isnan(tmp_pl_world_yaw) || tmp_pl_world_yaw == 0.0 || tmp_pl_world_yaw != tmp_pl_world_yaw)
			{
				continue;
			}

			tmp_angles.push_back(tmp_pl_world_yaw);
		}
	}

	// RCLCPP_INFO(this->get_logger(),  "Angles: %d\n", tmp_angles.size());

	static float powerline_2d_angle = 0.0; 

	// find most popular hough line angle
	if (tmp_angles.size() > 1)
	{
		this->get_parameter("2d_direction_bin_threshold", _2d_direction_bin_threshold);

		sort(tmp_angles.begin(), tmp_angles.end());

		std::vector<std::vector<float>> clusters;
		static float eps = (float)pcl::deg2rad((double)_2d_direction_bin_threshold); // threshold between bins
		float curr_angle = tmp_angles.at(0);

		int over_85_count = 0;
		int under_minus_85_count = 0;

		if (curr_angle > 1.48352986)
		{
			over_85_count++;
		}
		else if (curr_angle < -1.48352986)
		{
			under_minus_85_count++;
		}		

		std::vector<float> curr_cluster;
		curr_cluster.push_back(curr_angle);

		// divide found hough angles into clusters
		for (size_t i = 1; i < tmp_angles.size(); i++)
		{		
			if (tmp_angles.at(i) > 1.48352986) // 85 deg
			{
				over_85_count++;
			}
			else if (tmp_angles.at(i) < -1.48352986) // -85 deg
			{
				under_minus_85_count++;
			}
				
			if( abs(tmp_angles.at(i) - curr_angle) <= eps ) 
			{
				curr_cluster.push_back(tmp_angles.at(i));
			} 
			else 
			{
				clusters.push_back(curr_cluster);
				curr_cluster.clear();
				curr_cluster.push_back(tmp_angles.at(i));
			}
			curr_angle = tmp_angles.at(i);
		}
		clusters.push_back(curr_cluster);

		// if both >85 and <-85 measurements exist, combine into one bin to fix jump
		if (under_minus_85_count > 0 && over_85_count > 0)
		{
			if (under_minus_85_count > over_85_count)
			{
				if(clusters.at((clusters.size()-1)).size() > (size_t)over_85_count)
				{
					for (size_t i = 0; i < (size_t)over_85_count; i++)
					{
						clusters.at(((int)clusters.size()-1)).erase((clusters.at(((int)clusters.size()-1)).end()-1));
						clusters.at(0).push_back(-1.56905099754); // -89.99 deg
					}
				} 
				else
				{
					clusters.erase(clusters.end());

					for (int i = 0; i < over_85_count; i++)
					{
						clusters.at(0).push_back(-1.56905099754); // -89.99 deg
					}
				}
			}

			if (under_minus_85_count <= over_85_count)
			{
				if(clusters.at(0).size() > (size_t)under_minus_85_count)
				{
					for (size_t i = 0; i < (size_t)under_minus_85_count; i++)
					{
						clusters.at(0).erase((clusters.at(0).begin()));
						clusters.at((clusters.size()-1)).push_back(1.56905099754); // 89.99 deg
					}
				} 
				else
				{
					clusters.erase(clusters.begin());

					for (int i = 0; i < under_minus_85_count; i++)
					{
						clusters.at((clusters.size()-1)).push_back(1.56905099754); // 89.99 deg
					}
				}
			}
		}
				
		// print clusters
		// RCLCPP_INFO(this->get_logger(),  "Angle values:");
		// for (size_t i = 0; i < clusters.size(); i++)
		// {
		// 	RCLCPP_INFO(this->get_logger(),  "Cluster %d:", i);
		// 	for (size_t j = 0; j < clusters.at(i).size(); j++)
		// 	{
		// 		RCLCPP_INFO(this->get_logger(),  "%f \t", clusters.at(i).at(j));
		// 	}
		// }
		// RCLCPP_INFO(this->get_logger(),  "over 85: %d", over_85_count);
		// RCLCPP_INFO(this->get_logger(),  "under -85: %d", under_minus_85_count);


		// find biggest cluster
		int biggest_cluster = -1;
		int biggest_cluster_idx = -1;
		int second_biggest_cluster = -2;
		for (size_t i = 0; i < clusters.size(); i++)
		{

			if ((int)clusters.at(i).size() > biggest_cluster)
			{
				biggest_cluster = (int)clusters.at(i).size();
				biggest_cluster_idx = (int)i;
			}
			else 
			{
				if ((int)clusters.at(i).size() > second_biggest_cluster)
				{
					second_biggest_cluster = (int)clusters.at(i).size();
				}
			}
		}

		if (biggest_cluster == second_biggest_cluster || biggest_cluster_idx < 0)
		{
			// empty or tie, no clear hough direction = keep previous powerline angle
			powerline_2d_angle = powerline_2d_angle;
		}
		else
		{
			// average cluster with most votes
			float count = (float)clusters.at(biggest_cluster_idx).size();
			float sum = 0.0;

			for (size_t i = 0; i < clusters.at(biggest_cluster_idx).size(); i++)
			{
				sum += clusters.at(biggest_cluster_idx).at(i);
			}

			powerline_2d_angle = 0.8*powerline_2d_angle + 0.2*(sum / count); // simple low pass filter
		}
	}

	// update direction axis that is used for 3D parallel line fit
	dir_axis(0) = cos(powerline_2d_angle);
	dir_axis(1) = sin(powerline_2d_angle);
	dir_axis(2) = 0;

	if (_launch_with_debug > 0)
	{	
		std::string txt_angle = std::to_string((int)roundf(powerline_2d_angle*DEG_PER_RAD))+"*";
	
		cv::putText(img, //target image
				txt_angle, //text
				cv::Point((img.cols / 2)-15, img.rows-25), //bottom-center position
				cv::FONT_HERSHEY_DUPLEX,
				1.0,
				cv::Scalar(255,255,255), //font color
				2);

		sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img).toImageMsg();

		hough_line_pub->publish(*msg.get());
	}
}	

void RadarPCLFilter::crop_distant_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
									pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped) {

	this->get_parameter("cluster_crop_radius", _cluster_crop_radius);

	float minX;
	float minY;
	float maxX;
	float maxY;

	_drone_xyz_mutex.lock(); {
	
		minX = _t_xyz(0) - _cluster_crop_radius;
		minY = _t_xyz(1) - _cluster_crop_radius;

		maxX = _t_xyz(0) + _cluster_crop_radius;
		maxY = _t_xyz(1) + _cluster_crop_radius;

	} _drone_xyz_mutex.unlock();

	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(minX, minY, -1000, 1.0));
	boxFilter.setMax(Eigen::Vector4f(maxX, maxY, 1000, 1.0));
	boxFilter.setInputCloud(cloud_in);
	boxFilter.filter(*cloud_cropped);

}												

void RadarPCLFilter::concatenate_poincloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
														pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points) {
	// Adds new points 

	*concat_points += *new_points;

}

void RadarPCLFilter::fixed_size_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
														pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points) {
// Removes oldest points if _concat_size is reached
	this->get_parameter("concat_size", _concat_size);

	_concat_history.push_back(new_points->size());

	if ((int)_concat_history.size() > _concat_size)
	{
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		for (size_t i = 0; i < (size_t)_concat_history.front(); i++)
		{
			inliers->indices.push_back(i);
		}

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(concat_points);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*concat_points);

		_concat_history.pop_front();
	}
}

void RadarPCLFilter::downsample_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr all_points,
														pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_points) {
// downsamples cloud with voxel grid
	this->get_parameter("leaf_size", _leaf_size);

	static pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud (all_points);
	voxel_grid.setLeafSize ((float)_leaf_size, (float)_leaf_size, (float)_leaf_size);
	voxel_grid.filter (*downsampled_points);
}

void RadarPCLFilter::filter_pointcloud(float ground_threshold, float drone_threshold, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	
	this->get_parameter("sensor_upwards_or_downwards", _sensor_upwards_or_downwards);

	int pcl_size = cloud->size();

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (size_t i = 0; i < (size_t)pcl_size; i++)
	{

		if (_sensor_upwards_or_downwards == "downwards")
		{

			if ( ( cloud->at(i).z > ground_threshold ) )
			{
				inliers->indices.push_back(i);
			}

		}
		else if (_sensor_upwards_or_downwards == "upwards")
		{
			if (  ( cloud->at(i).z > (_height_above_ground+drone_threshold) ) )
			{
				inliers->indices.push_back(i);
			}
		}
		else
		{
			RCLCPP_INFO(this->get_logger(),  "Unknown sensor direction: %s \n Allowed values: 'upwards' or 'downwards'", _sensor_upwards_or_downwards.c_str());
		}
		
	}

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(cloud);
	extract.setIndices(inliers);
	extract.filter(*cloud);
}

void RadarPCLFilter::read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

	// read PointCloud2 msg data
	int pcl_size = msg->width;
	uint8_t *ptr = msg->data.data();
	const uint32_t POINT_STEP = 12;

	static int count = 0;
	static float azimuth_tan_constant;
	static float elevation_tan_constant;

	if (count++ == 25) // only do at ~1Hz
	{
		this->get_parameter("radar_azimuth_fov", _radar_azimuth_fov);
		this->get_parameter("radar_elevation_fov", _radar_elevation_fov);
		this->get_parameter("drone_threshold", _drone_threshold);

		azimuth_tan_constant = tan( (_radar_azimuth_fov*RAD_PER_DEG) / 2 );
		elevation_tan_constant = tan( (_radar_elevation_fov*RAD_PER_DEG) / 2);

		count = 0;
	}

	

	for (size_t i = 0; i < (size_t)pcl_size; i++) 
	{
		pcl::PointXYZ point(
				(float)(*(reinterpret_cast<float*>(ptr + 0))),
				(float)(*(reinterpret_cast<float*>(ptr + 4))),
				(float)(*(reinterpret_cast<float*>(ptr + 8)))
			);

		// if ( point.y > 0.0 && abs(point.x)*azimuth_tan_constant < point.y && abs(point.z)*elevation_tan_constant < point.y )
		// if ( (float)point.y > 0.0 && (float)point.y*(float)azimuth_tan_constant < (float)abs(point.x) && (float)point.y*(float)elevation_tan_constant < (float)abs(point.z) )
		if ( (float)point.y > (float)_drone_threshold && 
				(float)abs(point.x)/(float)point.y < (float)azimuth_tan_constant &&
				(float)abs(point.z)/(float)point.y < (float)elevation_tan_constant )
		{
			cloud->push_back(point);
		}

		ptr += POINT_STEP;
	}
}   

void RadarPCLFilter::powerline_detection() {

	this->get_parameter("launch_with_debug", _launch_with_debug);
	this->get_parameter("voxel_or_time_concat", _voxel_or_time_concat);
	this->get_parameter("add_crop_downsample_rate", _add_crop_downsample_rate);
	this->get_parameter("reset_global_point_cloud", _reset_global_point_cloud);

	if (_reset_global_point_cloud == true)
	{
		_concat_cloud->clear();
		_pl_search_cloud->clear();
		this->set_parameter(rclcpp::Parameter("reset_global_point_cloud", false));
	}
	
	
	static int since_add_crop_downsample = _add_crop_downsample_rate-1;

	if (++since_add_crop_downsample == _add_crop_downsample_rate)
	{
		_concat_cloud_mutex.lock(); {	

			RadarPCLFilter::concatenate_poincloud(_concat_cloud, _pl_search_cloud);

			_concat_cloud->clear();		

    	} _concat_cloud_mutex.unlock();

		if (_pl_search_cloud->size() > 1)
		{
			RadarPCLFilter::crop_distant_points(_pl_search_cloud, _pl_search_cloud);

			if (_voxel_or_time_concat == "voxel")
			{
				RadarPCLFilter::downsample_pointcloud(_pl_search_cloud, _pl_search_cloud);
			} else {
				RadarPCLFilter::fixed_size_pointcloud(_pl_search_cloud, _pl_search_cloud);
			}
		}

		since_add_crop_downsample = 0;

		static Eigen::Vector3f dir_axis;
		// RadarPCLFilter::direction_extraction_2D(_pl_search_cloud, dir_axis);
		// RadarPCLFilter::direction_extraction_3D(_pl_search_cloud, dir_axis);
		RadarPCLFilter::direction_extraction_25D(_pl_search_cloud, dir_axis);

		// pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		if (_pl_search_cloud->size() > 1)
		{
			this->get_parameter("line_or_point_follow", _line_or_point_follow);

			if (_line_or_point_follow == "line")
			{
				_line_models = RadarPCLFilter::parallel_line_extraction(_pl_search_cloud, dir_axis);
			}
			else if (_line_or_point_follow == "point")
			{
				_line_models = RadarPCLFilter::follow_point_extraction(_pl_search_cloud, _global_powerline_2d_angle);
			}
			else 
			{
				RCLCPP_INFO(this->get_logger(), "Invalid follow method: %s", _line_or_point_follow);
			}
			
			if (_launch_with_debug > 0)
			{	
				auto pcl_msg = sensor_msgs::msg::PointCloud2();
				RadarPCLFilter::create_pointcloud_msg(_pl_search_cloud, &pcl_msg);
				output_pointcloud_pub->publish(pcl_msg);  
			}
				
		}
	}
}

	
			
int main(int argc, char *argv[])
{
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RadarPCLFilter>());

	rclcpp::shutdown();
	return 0;
}
