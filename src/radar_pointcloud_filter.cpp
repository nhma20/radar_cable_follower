// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
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
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

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

			this->declare_parameter<float>("model_thresh", 0.5);
			this->get_parameter("model_thresh", _model_thresh);

			this->declare_parameter<float>("ground_threshold", 5);
			this->get_parameter("ground_threshold", _ground_threshold);

			raw_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/iwr6843_pcl",	10,
			std::bind(&RadarPCLFilter::transform_pointcloud_to_world, this, std::placeholders::_1));

			output_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world_pcl", 10);

			pl_direction_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/powerline_direction", 10);

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

		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pcl_subscription_;

		float _ground_threshold;

		float _height_above_ground = 0;

		int _t_tries = 0;

		bool _first_message = false;

		int _concat_size; 

		float _leaf_size;

		float _model_thresh;

		float _powerline_world_yaw; // +90 to -90 deg relative to x-axis

		vector_t _t_xyz;

		std::deque<int> _concat_history; 

		pcl::PointCloud<pcl::PointXYZ>::Ptr _concat_points;

		void transform_pointcloud_to_world(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

		void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void filter_pointcloud(float ground_threshold, float drone_threshold, 
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void concatenate_poincloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points);

		void direction_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
									pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered);

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


// void RadarPCLFilter::direction_extraction(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,
// 											pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered) {

// 	pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr
// 		line_model(new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud_in));

// 	std::vector<int> inliers;
// 	pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (line_model);
//     ransac.setDistanceThreshold ((float)_model_thresh);
//     ransac.computeModel();
//     ransac.getInliers(inliers);

// 	pcl::copyPointCloud (*cloud_in, inliers, *cloud_filtered);
// }
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


// void RadarPCLFilter::concatenate_poincloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points) {
// // concatenates pointclouds until _concat_size is reached then keeps cloud at that size
// 	this->get_parameter("concat_size", _concat_size);

// 	*_concat_points += *new_points;

// 	_concat_history.push_back(new_points->size());

// 	if (_concat_history.size() > _concat_size)
// 	{
// 		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

// 		for (size_t i = 0; i < (size_t)_concat_history.front(); i++)
// 		{
// 			inliers->indices.push_back(i);
// 		}

// 		pcl::ExtractIndices<pcl::PointXYZ> extract;
// 		extract.setInputCloud(_concat_points);
// 		extract.setIndices(inliers);
// 		extract.setNegative(true);
// 		extract.filter(*_concat_points);

// 		_concat_history.pop_front();
// 	}
// }
void RadarPCLFilter::concatenate_poincloud(pcl::PointCloud<pcl::PointXYZ>::Ptr new_points) {
// Continuously adds new points and downsamples cloud with voxed grid
	this->get_parameter("leaf_size", _leaf_size);

	*_concat_points += *new_points;

	static pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setInputCloud (_concat_points);
	voxel_grid.setLeafSize ((float)_leaf_size, (float)_leaf_size, (float)_leaf_size);
	voxel_grid.filter (*_concat_points);
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

	RadarPCLFilter::concatenate_poincloud(world_points);

	pcl::PointCloud<pcl::PointXYZ>::Ptr extracted_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	if (world_points->size() > 1)
	{
		RadarPCLFilter::direction_extraction(_concat_points, extracted_cloud);
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

