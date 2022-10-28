// ROS2 includes
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
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
#include <limits>
#include <vector>
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

// #include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
// #include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
// #include <pcl/visualization/pcl_visualizer.h>

using namespace std::chrono_literals;

//creates a RadarPCLFilter class that subclasses the generic rclcpp::Node base class.
class RadarPCLFilter : public rclcpp::Node
{
	public:
		RadarPCLFilter() : Node("radar_pcl_filter_node") {
			
			raw_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/iwr6843_pcl",	10,
			std::bind(&RadarPCLFilter::transform_pointcloud_to_world, this, std::placeholders::_1));

			output_pointcloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/world_pcl", 10);

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
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pcl_subscription_;

		float _height_above_ground = 0;

		int _t_tries = 0;

		bool _first_message = false;

		int _concat_size = 200;

		std::vector<int> _concat_history; 

		void transform_pointcloud_to_world(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

		void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void filter_pointcloud(float ground_threshold, float drone_threshold, 
								pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

		void concatenate_poincloud(Eigen::MatrixXf * new_points, Eigen::MatrixXf * concat_points);

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


void RadarPCLFilter::filter_pointcloud(float ground_threshold, float drone_threshold, 
										pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
	
	int pcl_size = cloud->size();

	// std::vector<int> remove_indeces;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	for (size_t i = 0; i < (size_t)pcl_size; i++)
	{
		if ( ( cloud->at(i).z > ground_threshold )  && ( cloud->at(i).z < (_height_above_ground-drone_threshold) ) )
		{
			inliers->indices.push_back(i);
		}
		// else {
		// 	inliers->indices.push_back(i);
		// }
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

	vector_t t_xyz;
	t_xyz(0) = t.transform.translation.x;
	t_xyz(1) = t.transform.translation.y;
	t_xyz(2) = t.transform.translation.z;

	quat_t t_rot;
	t_rot(0) = t.transform.rotation.x;
	t_rot(1) = t.transform.rotation.y;
	t_rot(2) = t.transform.rotation.z;
	t_rot(3) = t.transform.rotation.w;

	homog_transform_t world_to_drone = getTransformMatrix(t_xyz, t_rot);

	// transform points in pointcloud

	pcl::PointCloud<pcl::PointXYZ>::Ptr local_points (new pcl::PointCloud<pcl::PointXYZ>);

	RadarPCLFilter::read_pointcloud(msg, local_points);

	pcl::PointCloud<pcl::PointXYZ>::Ptr world_points (new pcl::PointCloud<pcl::PointXYZ>);

	pcl::transformPointCloud (*local_points, *world_points, world_to_drone);

	// filter ground and drone points 

	RadarPCLFilter::filter_pointcloud(1, 0.2, world_points);

	// publish transformed pointcloud

	auto pcl_msg = sensor_msgs::msg::PointCloud2();

	RCLCPP_INFO(this->get_logger(), "World cloud size: %d", world_points->size());

	RadarPCLFilter::create_pointcloud_msg(world_points, &pcl_msg);

	if (_first_message == false)
	{
		RCLCPP_INFO(this->get_logger(), "Published transformed pointcloud");
		_first_message = true;
	}

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

