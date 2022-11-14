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


			raw_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/iwr6843_pcl",	10,
			std::bind(&RadarPCLFilter::transform_pointcloud_to_world, this, std::placeholders::_1));

			output_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world_pcl", 10);

			pl_direction_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/powerline_direction", 10);

			direction_array_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/powerline_array", 10);

			hough_line_pub = this->create_publisher<sensor_msgs::msg::Image>("/hough_line_img", 10);

			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

			geometry_msgs::msg::TransformStamped drone_tf;

			timer_ = this->create_wall_timer(33ms, 
				std::bind(&RadarPCLFilter::update_powerline_poses, this)); //, std::placeholders::_1

			
			_timer_tf = this->create_wall_timer(33ms, 
				std::bind(&RadarPCLFilter::update_tf, this)); //, std::placeholders::_1


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

				pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
				_concat_points = tmp_cloud;

			}
		}

		~RadarPCLFilter() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down radar_pointcloud_filter node..");
		}

	private:
		std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
  		std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::TimerBase::SharedPtr _timer_tf;

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pointcloud_pub;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_pub;
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr direction_array_pub;
		rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr hough_line_pub;

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pcl_subscription_;

		float _ground_threshold;
		float _drone_threshold;
		float _height_above_ground = 0;
		int _concat_size; 
		std::string _voxel_or_time_concat;
		float _leaf_size;
		float _model_thresh;
		float _cluster_crop_radius;
		float _line_model_parallel_angle_threshold;
		float _line_model_distance_thresh;
		float _line_model_inlier_thresh;
		float _line_model_pitch_thresh;

		int _t_tries = 0;

		bool _first_message = false;

		float _powerline_world_yaw; // +90 to -90 deg relative to x-axis

		vector_t _t_xyz;
		quat_t _t_rot;

		std::deque<int> _concat_history; 

		pcl::PointCloud<pcl::PointXYZ>::Ptr _concat_points;

		std::vector<line_model_t> _line_models;

		void transform_pointcloud_to_world(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

		void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void filter_pointcloud(float ground_threshold, float drone_threshold, 
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void concatenate_poincloud_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
												pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points);

		void concatenate_poincloud_fixed_size(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
												pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points);

		void crop_distant_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
									pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped);

		void direction_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
									pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

		float direction_extraction_2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
										Eigen::Vector3f &dir_axis);

		std::vector<line_model_t> line_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);		

		std::vector<line_model_t> parallel_line_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
																	Eigen::Vector3f axis);							

		void create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, auto * pcl_msg);

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


	_t_xyz(0) = t.transform.translation.x;
	_t_xyz(1) = t.transform.translation.y;
	_t_xyz(2) = t.transform.translation.z;

	_t_rot(0) = t.transform.rotation.x;
	_t_rot(1) = t.transform.rotation.y;
	_t_rot(2) = t.transform.rotation.z;
	_t_rot(3) = t.transform.rotation.w;

}


void RadarPCLFilter::update_powerline_poses() {

	if (_line_models.size() > 0)
	{

		auto pose_array_msg = geometry_msgs::msg::PoseArray();
		pose_array_msg.header = std_msgs::msg::Header();
		pose_array_msg.header.stamp = this->now();
		pose_array_msg.header.frame_id = "world";
		
		plane_t proj_plane = create_plane(_line_models.at(0).quaternion, _t_xyz);

		for (size_t i = 0; i < _line_models.size(); i++)
		{	

			point_t pl_point(
				_line_models.at(i).position(0),
				_line_models.at(i).position(1),
				_line_models.at(i).position(2)
			);

			point_t proj_pl_point = projectPointOnPlane(pl_point, proj_plane);

			// RCLCPP_INFO(this->get_logger(),  "Proj point: \n X %f \n Y %f \n Z %f", proj_pl_point(0), proj_pl_point(1), proj_pl_point(2));
			
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

		direction_array_pub->publish(pose_array_msg);

	}


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

		RCLCPP_INFO(this->get_logger(),  "Powerline yaw: %f", (_powerline_world_yaw*DEG_PER_RAD));
	
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
		pose_msg.pose.position.x = _t_xyz(0);
		pose_msg.pose.position.y = _t_xyz(1);
		pose_msg.pose.position.z = _t_xyz(2);


		pl_direction_pub->publish(pose_msg);

	}	
}


float RadarPCLFilter::direction_extraction_2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, 
												Eigen::Vector3f &dir_axis) {

	int img_size = _cluster_crop_radius*10*2;
	cv::Mat img(img_size, img_size, CV_8UC1, cv::Scalar(0));

	// project point cloud onto ground and create mat from points
	for (size_t i = 0; i < cloud_in->size(); i++)
	{
		pcl::PointXYZ point = (*cloud_in)[i];

		// x and y swapped to align image with world x y (up = +x, left = +y)
		float y_tmp = roundf( img_size/2 - 10.0*(point.x - _t_xyz(0)) );
		float x_tmp = roundf( img_size/2 - 10.0*(point.y - _t_xyz(1)) );

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
			float sum;

			for (size_t i = 0; i < clusters.at(biggest_cluster_idx).size(); i++)
			{
				sum += clusters.at(biggest_cluster_idx).at(i);
			}

			powerline_2d_angle = sum / count;
		}
		
	}
	
	RCLCPP_INFO(this->get_logger(),  "Angle %f:", powerline_2d_angle);

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

	float minX = _t_xyz(0) - _cluster_crop_radius;
	float minY = _t_xyz(1) - _cluster_crop_radius;

	float maxX = _t_xyz(0) + _cluster_crop_radius;
	float maxY = _t_xyz(1) + _cluster_crop_radius;

	pcl::CropBox<pcl::PointXYZ> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(minX, minY, -1000, 1.0));
	boxFilter.setMax(Eigen::Vector4f(maxX, maxY, 1000, 1.0));
	boxFilter.setInputCloud(cloud_in);
	boxFilter.filter(*cloud_cropped);

}


void RadarPCLFilter::concatenate_poincloud_fixed_size(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
														pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points) {
// concatenates pointclouds until _concat_size is reached then removes oldest points
	this->get_parameter("concat_size", _concat_size);

	*concat_points += *new_points;

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


void RadarPCLFilter::concatenate_poincloud_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
														pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points) {
// Continuously adds new points and downsamples cloud with voxel grid
	this->get_parameter("leaf_size", _leaf_size);

	*concat_points += *new_points;

	static pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud (concat_points);
	voxel_grid.setLeafSize ((float)_leaf_size, (float)_leaf_size, (float)_leaf_size);
	voxel_grid.filter (*concat_points);
}


void RadarPCLFilter::filter_pointcloud(float ground_threshold, float drone_threshold, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	
	int pcl_size = cloud->size();

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (size_t i = 0; i < (size_t)pcl_size; i++)
	{
		if ( ( cloud->at(i).z > ground_threshold )  && ( cloud->at(i).z < (_height_above_ground-drone_threshold) ) )
		{
			inliers->indices.push_back(i);
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



void RadarPCLFilter::transform_pointcloud_to_world(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

	if (msg->width < 1)
	{
		return;
	}


	// make transform drone->world

	_height_above_ground = _t_xyz(2); ///t.transform.translation.z;


	homog_transform_t world_to_drone = getTransformMatrix(_t_xyz, _t_rot);

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

	// publish transformed pointcloud

	// RCLCPP_INFO(this->get_logger(), "World cloud size: %d", world_points->size());

	this->get_parameter("voxel_or_time_concat", _voxel_or_time_concat);

	if (_voxel_or_time_concat == "voxel")
	{
		RadarPCLFilter::concatenate_poincloud_downsample(world_points, _concat_points);
	} else {
		RadarPCLFilter::concatenate_poincloud_fixed_size(world_points, _concat_points);
	}
	

	

	RadarPCLFilter::crop_distant_points(_concat_points, _concat_points);

	static Eigen::Vector3f dir_axis;
	RadarPCLFilter::direction_extraction_2D(_concat_points, dir_axis); // add lines sorting stuff at some point

	// RCLCPP_INFO(this->get_logger(),  "Axis: \n X %f\n Y %f\n Z %f\n", dir_axis(0), dir_axis(1), dir_axis(2));

	pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (_concat_points->size() > 1)
	{
		// RadarPCLFilter::direction_extraction(_concat_points, extracted_cloud);
		// _line_models = RadarPCLFilter::line_extraction(_concat_points);
		_line_models = RadarPCLFilter::parallel_line_extraction(_concat_points, dir_axis);
	}
	

	// auto pcl_msg = sensor_msgs::msg::PointCloud2();
	// RadarPCLFilter::create_pointcloud_msg(extracted_cloud, &pcl_msg);
	// output_pointcloud_pub->publish(pcl_msg);  

	auto pcl_msg = sensor_msgs::msg::PointCloud2();
	RadarPCLFilter::create_pointcloud_msg(_concat_points, &pcl_msg);
	output_pointcloud_pub->publish(pcl_msg);  

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

