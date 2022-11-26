// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.h"
#include "geometry.h"

 // MISC includes
#include <algorithm>
#include <cstdlib>
#include <stdlib.h> 
#include <iostream>
#include <chrono>
#include <ctime>    
#include <math.h> 
#include <cmath> 
#include <limits>
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
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
// #include "opencv2/core/hal/interface.hpp"



#define DEG_PER_RAD 57.296
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

			this->declare_parameter<float>("model_thresh", 1.0);
			this->get_parameter("model_thresh", _model_thresh);

			this->declare_parameter<float>("ground_threshold", 1.5);
			this->get_parameter("ground_threshold", _ground_threshold);

			this->declare_parameter<float>("drone_threshold", 0.5);
			this->get_parameter("drone_threshold", _drone_threshold);

			this->declare_parameter<float>("cluster_crop_radius", 20);
			this->get_parameter("cluster_crop_radius", _cluster_crop_radius);

			this->declare_parameter<float>("line_model_parallel_angle_threshold", 10.0);
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

			this->declare_parameter<int>("point_follow_outlier_filter", 0);
			this->get_parameter("point_follow_outlier_filter", _point_follow_outlier_filter);

			this->declare_parameter<std::string>("sensor_upwards_or_downwards", "downwards");
			this->get_parameter("sensor_upwards_or_downwards", _sensor_upwards_or_downwards);

			this->declare_parameter<int>("add_crop_downsample_rate", 5);
			this->get_parameter("add_crop_downsample_rate", _add_crop_downsample_rate);


			raw_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/iwr6843_pcl",	10,
			std::bind(&RadarPCLFilter::add_new_radar_pointcloud, this, std::placeholders::_1));

			output_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world_pcl", 10);

			pl_direction_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/powerline_direction", 10);

			direction_array_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/powerline_array", 10);

			hough_line_pub = this->create_publisher<sensor_msgs::msg::Image>("/hough_line_img", 10);
			
			debug_array_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/debug_array", 10);

			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

			geometry_msgs::msg::TransformStamped drone_tf;

			_timer_pl = this->create_wall_timer(33ms, 
				std::bind(&RadarPCLFilter::update_powerline_poses, this)); //, std::placeholders::_1

			_timer_tf = this->create_wall_timer(33ms, 
				std::bind(&RadarPCLFilter::update_tf, this)); //, std::placeholders::_1

			_timer_pcl = this->create_wall_timer(100ms, 
				std::bind(&RadarPCLFilter::powerline_detection, this)); //, std::placeholders::_1


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
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_pub;
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr direction_array_pub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr hough_line_pub;
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr debug_array_pub;

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

		// void transform_pointcloud_to_world(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

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

		std::vector<line_model_t> line_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);		


		std::vector<line_model_t> parallel_line_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
																Eigen::Vector3f axis);

		std::vector<line_model_t> follow_point_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
																Eigen::Vector3f axis);																						

		void create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, auto * pcl_msg);

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

		_drone_xyz_mutex.lock(); {
	
			new_drone_xyz(0) = _t_xyz(0);
			new_drone_xyz(1) = _t_xyz(1);
			new_drone_xyz(2) = -_t_xyz(2);

			proj_plane = create_plane(_line_models.at(0).quaternion, _t_xyz);

		} _drone_xyz_mutex.unlock();

		for (size_t i = 0; i < _line_models.size(); i++)
		{	

			// point_t pl_point(
			// 	_line_models.at(i).position(0),
			// 	_line_models.at(i).position(1),
			// 	_line_models.at(i).position(2)
			// );


			point_t proj_pl_point = projectPointOnPlane(_line_models.at(i).position, proj_plane);

			// RCLCPP_INFO(this->get_logger(),  "Proj point: \n X %f \n Y %f \n Z %f", proj_pl_point(0), proj_pl_point(1), proj_pl_point(2));
			
			new_point_vec.push_back(proj_pl_point);

			new_quat_vec.push_back(_line_models.at(i).quaternion);

			auto pose_msg = geometry_msgs::msg::Pose();

			pose_msg.orientation.x = _line_models.at(i).quaternion(0);
			pose_msg.orientation.y = _line_models.at(i).quaternion(1);
			pose_msg.orientation.z = _line_models.at(i).quaternion(2);
			pose_msg.orientation.w = _line_models.at(i).quaternion(3);
			pose_msg.position.x = proj_pl_point(0);
			pose_msg.position.y = proj_pl_point(1);
			pose_msg.position.z = proj_pl_point(2);

			pose_array_msg.poses.push_back(pose_msg);
		}


		point_t drone_xyz_delta = new_drone_xyz - prev_drone_xyz;
		

		// look for matches to merge new and existing point
		for (size_t i = 0; i < new_point_vec.size(); i++)
		{
			// TODO: GET POINTS TO ORIGO BEFORE ROTATING TO MINIMIZE WOBBLE - works?
			// - Get ID poses down from 2x Z
			// new_point_vec.at(i) = new_point_vec.at(i) - new_drone_xyz; // (new_point_vec.at(i) + drone_xyz_delta) - new_drone_xyz;
			// new_point_vec.at(i) = rotateVector(rot, new_point_vec.at(i));
			bool merged = false;
			
			for (size_t j = 0; j < prev_point_vec.size(); j++)
			{				
				float dist_x = abs( (prev_point_vec.at(j).point(0) + drone_xyz_delta(0)) - new_point_vec.at(i)(0) );
				float dist_y = abs( (prev_point_vec.at(j).point(1) + drone_xyz_delta(1)) - new_point_vec.at(i)(1) );
				float dist_z = abs( (prev_point_vec.at(j).point(2) + drone_xyz_delta(2)) - new_point_vec.at(i)(2) );
				float euclid_dist = sqrt( pow(dist_x, 2) + pow(dist_y, 2) + pow(dist_z, 2) );
				
				if (euclid_dist < 1.5) // param this
				{
					prev_point_vec.at(j).point(0) = ( new_point_vec.at(i)(0) + prev_point_vec.at(j).point(0) ) / 2.0f; //new_point_vec.at(i)(0);
					prev_point_vec.at(j).point(1) = ( new_point_vec.at(i)(1) + prev_point_vec.at(j).point(1) ) / 2.0f; //new_point_vec.at(i)(1);
					prev_point_vec.at(j).point(2) = ( new_point_vec.at(i)(2) + prev_point_vec.at(j).point(2) ) / 2.0f; //new_point_vec.at(i)(2);
					
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
		

		if (prev_point_vec.size() > 0)
		{		
			auto track_pose_array_msg = geometry_msgs::msg::PoseArray();
			track_pose_array_msg.header = std_msgs::msg::Header();
			track_pose_array_msg.header.stamp = this->now();
			track_pose_array_msg.header.frame_id = "world";

			for (size_t i = 0; i < prev_point_vec.size(); i++)
			{

				if (prev_point_vec.at(i).alive_count < 250)
				{
					continue;
				}
				

				RCLCPP_INFO(this->get_logger(),  "\nPoint %d: \n X %f\t Y %f\t Z %f\t ID %d\t ALIVE %d\n", 
					i, prev_point_vec.at(i).point(0), prev_point_vec.at(i).point(1), prev_point_vec.at(i).point(2), 
					prev_point_vec.at(i).id, prev_point_vec.at(i).alive_count);
			
			
				auto track_pose_msg = geometry_msgs::msg::Pose();
				track_pose_msg.orientation.x = prev_quat_vec.at(i)(0);
				track_pose_msg.orientation.y = prev_quat_vec.at(i)(1);
				track_pose_msg.orientation.z = prev_quat_vec.at(i)(2);
				track_pose_msg.orientation.w = prev_quat_vec.at(i)(3);
				track_pose_msg.position.x = prev_point_vec.at(i).point(0);
				track_pose_msg.position.y = prev_point_vec.at(i).point(1);
				track_pose_msg.position.z = prev_point_vec.at(i).point(2);
				track_pose_array_msg.poses.push_back(track_pose_msg);
			}

			debug_array_pub->publish(track_pose_array_msg);
		}
		
		
		new_point_vec.clear();
		new_quat_vec.clear();

		direction_array_pub->publish(pose_array_msg);

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
	this->get_parameter("ground_threshold", _drone_threshold);
	RadarPCLFilter::filter_pointcloud(_ground_threshold, _drone_threshold, world_points);

	if (world_points->size() < 1)
	{
		return;
	}

	_concat_cloud_mutex.lock(); {

        RadarPCLFilter::concatenate_poincloud(world_points, _concat_cloud);

    } _concat_cloud_mutex.unlock();

}


void RadarPCLFilter::create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, auto * pcl_msg) {

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
			inliers->indices.size() > (int)_line_model_inlier_thresh)
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

		line_model = {

            .position = pl_position,
            .quaternion = pl_quat

        };

		line_models.push_back(line_model);

		extract.setInputCloud(reduced_cloud);

		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*reduced_cloud); 
		

		if (reduced_cloud->size() < (int)_line_model_inlier_thresh)
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

	if (inliers->indices.size() < (int)_line_model_inlier_thresh)
	{
		return _line_models;
	}
	
	line_model_t line_model;
	int count = 0;

	// Continue line extraction if first line model has pitch below threshold and inliers above threshold
	while (abs(coefficients->values[5]) < _line_model_pitch_thresh && 
			inliers->indices.size() > (int)_line_model_inlier_thresh)
	{

		// RCLCPP_INFO(this->get_logger(),  "Axis: \n X %f\n Y %f\n Z %f\n", coefficients->values[3], coefficients->values[4], coefficients->values[5]);

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

		line_model = {

            .position = pl_position,
            .quaternion = pl_quat

        };

		line_models.push_back(line_model);

		extract.setInputCloud(reduced_cloud);

		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*reduced_cloud); 
		

		if (reduced_cloud->size() < (int)_line_model_inlier_thresh)
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
																	Eigen::Vector3f axis) {
	
	if (cloud_in->size() > 50)
	{
		this->get_parameter("point_follow_outlier_filter", _point_follow_outlier_filter);

		if (_point_follow_outlier_filter > 0)
		{
		
			// filter cloud for outliers
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud (cloud_in);
			sor.setMeanK (5); // param this
			sor.setStddevMulThresh (6.0); // and this?
			sor.filter (*cloud_in);

		}

		// find highest point
		pcl::PointXYZ max_z_point;
		for (size_t i = 0; i < cloud_in->size (); ++i)
		{
			// Check if the point is invalid
			if (!std::isfinite ((*cloud_in)[i].x) || 
				!std::isfinite ((*cloud_in)[i].y) || 
				!std::isfinite ((*cloud_in)[i].z))
				continue;

			if ((*cloud_in)[i].z > max_z_point.z) {
				max_z_point = (*cloud_in)[i];
			}
		}

	// create line model from hough angle and highest point XYZ
	orientation_t hough_angle (
		0,
		0,
		acos(axis(0))
	);

	std::vector<line_model_t> line_model_vec;
	line_model_t line_model;

	line_model.position(0) = max_z_point.x;
	line_model.position(1) = max_z_point.y;
	line_model.position(2) = max_z_point.z;

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

		auto pose_msg = geometry_msgs::msg::PoseStamped();

		pose_msg.header = std_msgs::msg::Header();
		pose_msg.header.stamp = this->now();
		pose_msg.header.frame_id = "world";
		pose_msg.pose.orientation.x = yaw_quat(0);
		pose_msg.pose.orientation.y = yaw_quat(1);
		pose_msg.pose.orientation.z = yaw_quat(2);
		pose_msg.pose.orientation.w = yaw_quat(3);

		_drone_xyz_mutex.lock(); {

			pose_msg.pose.position.x = _t_xyz(0);
			pose_msg.pose.position.y = _t_xyz(1);
			pose_msg.pose.position.z = _t_xyz(2);
			
		} _drone_xyz_mutex.unlock();

		pl_direction_pub->publish(pose_msg);

	}	
}

void RadarPCLFilter::direction_extraction_2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
												Eigen::Vector3f &dir_axis) {

	int img_size = _cluster_crop_radius*10*2;
	cv::Mat img(img_size, img_size, CV_8UC1, cv::Scalar(0));

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
		// img.at<uchar>(x_tmp, y_tmp) = 255;
		cv::circle(img, cv::Point(x_tmp,y_tmp),1, cv::Scalar(255,255,255), -1, 8,0);

	}

	// Probabilistic Line Transform
    std::vector<cv::Vec4i> linesP; // will hold the results of the detection
	std::vector<float> tmp_angles;
    cv::HoughLinesP(img, linesP, 1, PI/180, 35, 35, 30 ); // rho res pixels, theta res rads, min intersections, min line length, max line gap
    // Draw the lines
	float tmp_pl_world_yaw = -0.0;
    for( size_t i = 0; i < linesP.size(); i++ )
    {
        cv::Vec4i l = linesP[i];
        cv::line( img, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(127,127,127), 3, cv::LINE_AA);
		// break;
		// RCLCPP_INFO(this->get_logger(),  "Points \n XY: %f %f \n XY: %f %f \n", (float)l[0], (float)l[1], (float)l[2], (float)l[3]);
		
		float diff_x = (float)l[0] - (float)l[2];
		float diff_y = (float)l[1] - (float)l[3];
		float ratio = diff_y / diff_x;

		tmp_pl_world_yaw = (float)(abs(ratio) / ratio) * acos(abs(ratio));

		if (isnan(tmp_pl_world_yaw) || tmp_pl_world_yaw != tmp_pl_world_yaw || tmp_pl_world_yaw == 0.0)
		{
			continue;
		}

		tmp_angles.push_back(tmp_pl_world_yaw);

		// RCLCPP_INFO(this->get_logger(),  "Angle: %f\n", (tmp_pl_world_yaw*DEG_PER_RAD));
	}



	static float powerline_2d_angle = 0.0; 

	// find most popular hough line angle
	if (tmp_angles.size() > 1)
	{
	
		sort(tmp_angles.begin(), tmp_angles.end());

		std::vector<std::vector<float>> clusters;
		float eps = (float)pcl::deg2rad((double)15.0);
		float curr_angle = tmp_angles.at(0);
		std::vector<float> curr_cluster;
		curr_cluster.push_back(curr_angle);

		// divide found hough angles into clusters
		for (size_t i = 1; i < tmp_angles.size(); i++)
		{			
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
		
		// print clusters
		// RCLCPP_INFO(this->get_logger(),  "Angle values:");
		for (size_t i = 0; i < clusters.size(); i++)
		{
			// RCLCPP_INFO(this->get_logger(),  "Cluster %d:", i);
			for (size_t j = 0; j < clusters.at(i).size(); j++)
			{
				// RCLCPP_INFO(this->get_logger(),  "%f \t", clusters.at(i).at(j));
			}
			
		}

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

			powerline_2d_angle = sum / count;


			// RCLCPP_INFO(this->get_logger(),  "Sum %f:", sum);
			// RCLCPP_INFO(this->get_logger(),  "Count %f:", count);
		}
		
	}
	// RCLCPP_INFO(this->get_logger(),  "Angle %f:", powerline_2d_angle);

	std::string txt_angle = std::to_string((int)roundf(powerline_2d_angle*DEG_PER_RAD));
	
	cv::putText(img, //target image
            txt_angle, //text
            cv::Point((img.cols / 2)-15, img.rows-25), //bottom-center position
            cv::FONT_HERSHEY_DUPLEX,
            1.0,
            cv::Scalar(255,255,255), //font color
            2);

	// static int name_counter = 0;
	// std::string filename = std::to_string(name_counter++);
	// std::string extension = ".jpg";
	// filename = filename + extension;
	// std::string path = "/home/nm/uzh_ws/ros2_ws/test_folder/";
	// cv::imwrite( (path+filename), img );

	// update direction axis that is used for 3D parallel line fit
	dir_axis(0) = cos(powerline_2d_angle);
	dir_axis(1) = sin(powerline_2d_angle);
	dir_axis(2) = 0;

	sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", img).toImageMsg();

	hough_line_pub->publish(*msg.get());

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
	this->get_parameter("leaf_size", _leaf_size);

	*concat_points += *new_points;

}

void RadarPCLFilter::fixed_size_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
														pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points) {
// Removes oldest points if _concat_size is reached
	this->get_parameter("concat_size", _concat_size);

	_concat_history.push_back(new_points->size());

	if (_concat_history.size() > _concat_size)
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

			if ( ( cloud->at(i).z > ground_threshold )  && ( cloud->at(i).z < (_height_above_ground-drone_threshold) ) )
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

	for (size_t i = 0; i < (size_t)pcl_size; i++) 
	{
		pcl::PointXYZ point(
				(float)(*(reinterpret_cast<float*>(ptr + 0))),
				(float)(*(reinterpret_cast<float*>(ptr + 4))),
				(float)(*(reinterpret_cast<float*>(ptr + 8)))
			);

		cloud->push_back(point);

		ptr += POINT_STEP;
	}
}   

void RadarPCLFilter::powerline_detection() {

	this->get_parameter("voxel_or_time_concat", _voxel_or_time_concat);
	this->get_parameter("add_crop_downsample_rate", _add_crop_downsample_rate);
	
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
	}
	


	static Eigen::Vector3f dir_axis;
	RadarPCLFilter::direction_extraction_2D(_pl_search_cloud, dir_axis); // add lines sorting stuff at some point

	pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (_pl_search_cloud->size() > 1)
	{
		this->get_parameter("line_or_point_follow", _line_or_point_follow);

		if (_line_or_point_follow == "line")
		{
			_line_models = RadarPCLFilter::parallel_line_extraction(_pl_search_cloud, dir_axis);
		}
		else if (_line_or_point_follow == "point")
		{
			_line_models = RadarPCLFilter::follow_point_extraction(_pl_search_cloud, dir_axis);
		}
		else 
		{
			RCLCPP_INFO(this->get_logger(), "Invalid follow method: %s", _line_or_point_follow);
		}
		
		auto pcl_msg = sensor_msgs::msg::PointCloud2();
		RadarPCLFilter::create_pointcloud_msg(_pl_search_cloud, &pcl_msg);
		output_pointcloud_pub->publish(pcl_msg);  
			
	}
}

	
			
int main(int argc, char *argv[])
{
	std::cout << "Starting radar_pcl_filter_node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<RadarPCLFilter>());

	rclcpp::shutdown();
	return 0;
}

