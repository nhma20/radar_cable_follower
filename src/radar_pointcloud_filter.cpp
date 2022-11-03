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

// PCL includes
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
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


#define DEG_PER_RAD 57.296

using namespace std::chrono_literals;

//creates a RadarPCLFilter class that subclasses the generic rclcpp::Node base class.
class RadarPCLFilter : public rclcpp::Node
{
	public:
		RadarPCLFilter() : Node("radar_pcl_filter_node") {
			
			// Params
			this->declare_parameter<int>("concat_size", 200);
			this->get_parameter("concat_size", _concat_size);

			this->declare_parameter<float>("leaf_size", 0.75);
			this->get_parameter("leaf_size", _leaf_size);

			this->declare_parameter<float>("model_thresh", 1.0);
			this->get_parameter("model_thresh", _model_thresh);

			this->declare_parameter<float>("ground_threshold", 5);
			this->get_parameter("ground_threshold", _ground_threshold);

			this->declare_parameter<float>("cluster_parallelism_threshold", 1.);
			this->get_parameter("cluster_parallelism_threshold", _cluster_parallelism_threshold);

			this->declare_parameter<int>("cluster_min_size", 25);
			this->get_parameter("cluster_min_size", _cluster_min_size);

			this->declare_parameter<float>("cluster_crop_radius", 20);
			this->get_parameter("cluster_crop_radius", _cluster_crop_radius);

			this->declare_parameter<float>("line_model_distance_threshold", 1.5);
			this->get_parameter("model_thresh", _line_model_distance_thresh);

			this->declare_parameter<float>("line_model_inlier_threshold", 10.0);
			this->get_parameter("model_thresh", _line_model_inlier_thresh);

			this->declare_parameter<float>("line_model_pitch_threshold", 0.25);
			this->get_parameter("model_thresh", _line_model_pitch_thresh);


			raw_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/iwr6843_pcl",	10,
			std::bind(&RadarPCLFilter::transform_pointcloud_to_world, this, std::placeholders::_1));

			output_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world_pcl", 10);

			pl_direction_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/powerline_direction", 10);

			direction_array_pub = this->create_publisher<geometry_msgs::msg::PoseArray>("/powerline_direction_array", 10);

			tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
			transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

			geometry_msgs::msg::TransformStamped drone_tf;

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

		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr output_pointcloud_pub;
		rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pl_direction_pub;
		rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr direction_array_pub;

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pcl_subscription_;

		float _ground_threshold;

		float _height_above_ground = 0;

		int _t_tries = 0;

		bool _first_message = false;

		int _concat_size; 

		float _leaf_size;

		float _model_thresh;

		float _powerline_world_yaw; // +90 to -90 deg relative to x-axis

		float _cluster_parallelism_threshold;

		int _cluster_min_size;

		float _cluster_crop_radius;

		float _line_model_distance_thresh;
		float _line_model_inlier_thresh;
		float _line_model_pitch_thresh;

		vector_t _t_xyz;

		std::deque<int> _concat_history; 

		pcl::PointCloud<pcl::PointXYZ>::Ptr _concat_points;

		void transform_pointcloud_to_world(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

		void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void filter_pointcloud(float ground_threshold, float drone_threshold, 
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void concatenate_poincloud_downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points,
												pcl::PointCloud<pcl::PointXYZ>::Ptr concat_points);

		void concatenate_poincloud_fixed_size(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points);

		void crop_distant_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
									pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped);

		void direction_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
									pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

		std::vector<line_model_t> line_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in);									

		void create_pointcloud_msg(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, auto * pcl_msg);
};



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
		RCLCPP_INFO(this->get_logger(),  "Powerline yaw: %f", (_powerline_world_yaw*DEG_PER_RAD));
		
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
		extract.filter(*reduced_cloud); // "The indices size exceeds the size of the input."
		

		if (reduced_cloud->size() < 1)
		{
			break;
		}
		
		seg.setInputCloud (reduced_cloud);
		seg.segment (*inliers, *coefficients);
	}		

	auto pose_array_msg = geometry_msgs::msg::PoseArray();
	pose_array_msg.header = std_msgs::msg::Header();
	pose_array_msg.header.stamp = this->now();
	pose_array_msg.header.frame_id = "world";

	// orientation_t pl_eul(
	// 		0,
	// 		0,
	// 		_powerline_world_yaw
	// 	);

	
	if (line_models.size() > 0)
	{
		plane_t proj_plane = create_plane(line_models.at(0).quaternion, _t_xyz);

		for (size_t i = 0; i < line_models.size(); i++)
		{	

			point_t pl_point(
				line_models.at(i).position(0),
				line_models.at(i).position(1),
				line_models.at(i).position(2)
			);

			point_t proj_pl_point = projectPointOnPlane(pl_point, proj_plane);

			RCLCPP_INFO(this->get_logger(),  "Proj point: \n X %f \n Y %f \n Z %f", proj_pl_point(0), proj_pl_point(1), proj_pl_point(2));
			
			auto pose_msg = geometry_msgs::msg::Pose();

			pose_msg.orientation.x = line_models.at(i).quaternion(0);
			pose_msg.orientation.y = line_models.at(i).quaternion(1);
			pose_msg.orientation.z = line_models.at(i).quaternion(2);
			pose_msg.orientation.w = line_models.at(i).quaternion(3);
			pose_msg.position.x = proj_pl_point(0);
			pose_msg.position.y = proj_pl_point(1);
			pose_msg.position.z = proj_pl_point(2);

			pose_array_msg.poses.push_back(pose_msg);
		}
	}

	direction_array_pub->publish(pose_array_msg);

	return line_models;

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
	
	// RCLCPP_INFO(this->get_logger(),  "%f", coefficients->values[3]); // X coordinate of a line's direction
	// RCLCPP_INFO(this->get_logger(),  "%f", coefficients->values[4]); // Y coordinate of a line's direction
	// RCLCPP_INFO(this->get_logger(),  "%f", coefficients->values[5]); // Z coordinate of a line's direction

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

		// RCLCPP_INFO(this->get_logger(),  "Q1: %f", yaw_quat(3));
		// RCLCPP_INFO(this->get_logger(),  "Q2: %f", yaw_quat(0));
		// RCLCPP_INFO(this->get_logger(),  "Q3: %f", yaw_quat(1));
		// RCLCPP_INFO(this->get_logger(),  "Q4: %f", yaw_quat(2));

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


void RadarPCLFilter::concatenate_poincloud_fixed_size(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points) {
// concatenates pointclouds until _concat_size is reached then removes oldest points
	this->get_parameter("concat_size", _concat_size);

	*_concat_points += *new_points;

	_concat_history.push_back(new_points->size());

	if (_concat_history.size() > _concat_size)
	{
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		for (size_t i = 0; i < (size_t)_concat_history.front(); i++)
		{
			inliers->indices.push_back(i);
		}

		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(_concat_points);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*_concat_points);

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


	// make transform drone->world

	_height_above_ground = t.transform.translation.z;

	_t_xyz(0) = t.transform.translation.x;
	_t_xyz(1) = t.transform.translation.y;
	_t_xyz(2) = t.transform.translation.z;

	quat_t t_rot;
	t_rot(0) = t.transform.rotation.x;
	t_rot(1) = t.transform.rotation.y;
	t_rot(2) = t.transform.rotation.z;
	t_rot(3) = t.transform.rotation.w;

	homog_transform_t world_to_drone = getTransformMatrix(_t_xyz, t_rot);

	// transform points in pointcloud

	pcl::PointCloud<pcl::PointXYZ>::Ptr local_points (new pcl::PointCloud<pcl::PointXYZ>);

	RadarPCLFilter::read_pointcloud(msg, local_points);

	pcl::PointCloud<pcl::PointXYZ>::Ptr world_points (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::transformPointCloud (*local_points, *world_points, world_to_drone);

	// filter ground and drone points 
	this->get_parameter("ground_threshold", _ground_threshold);
	RadarPCLFilter::filter_pointcloud(_ground_threshold, 0.2, world_points);

	if (world_points->size() < 1)
	{
		return;
	}

	// publish transformed pointcloud

	RCLCPP_INFO(this->get_logger(), "World cloud size: %d", world_points->size());

	RadarPCLFilter::concatenate_poincloud_downsample(world_points, _concat_points);

	RadarPCLFilter::crop_distant_points(_concat_points, _concat_points);

	pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (_concat_points->size() > 1)
	{
		// RadarPCLFilter::direction_extraction(_concat_points, extracted_cloud);
		RadarPCLFilter::line_extraction(_concat_points);
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

