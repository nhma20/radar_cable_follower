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

		void read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, Eigen::MatrixXf * data_holder);

		void filter_pointcloud(float ground_threshold, float drone_threshold, Eigen::MatrixXf * data_in, Eigen::MatrixXf * data_out);

		void concatenate_poincloud(Eigen::MatrixXf * new_points, Eigen::MatrixXf * concat_points);

		void create_pointcloud_msg(Eigen::MatrixXf * data, auto * pcl_msg);
};




void RadarPCLFilter::create_pointcloud_msg(Eigen::MatrixXf * data, auto * pcl_msg) {

  // create PointCloud2 msg
	//https://github.com/ros-drivers/velodyne/blob/master/velodyne_laserscan/tests/system.cpp
	int pcl_size = data->cols();
	auto pcl2_msg = sensor_msgs::msg::PointCloud2();
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
	const uint32_t POINT_STEP = 12;
	if(pcl_size > 0){
		pcl_msg->data.resize(std::max((size_t)1, (size_t)pcl_size) * POINT_STEP, 0x00);
	}
	pcl_msg->point_step = POINT_STEP; // size (bytes) of 1 point (float32 * dimensions (3 when xyz))
	pcl_msg->row_step = pcl_msg->data.size();//pcl_msg->point_step * pcl_msg->width; // only 1 row because unordered
	pcl_msg->height = 1;  // because unordered cloud
	pcl_msg->width = pcl_msg->row_step / POINT_STEP; // number of points in cloud
	pcl_msg->is_dense = false; // there may be invalid points

	// fill PointCloud2 msg data
	if(pcl_size > 0){
		uint8_t *ptr = pcl_msg->data.data();
		for (size_t i = 0; i < (size_t)pcl_size; i++)
		{
			*(reinterpret_cast<float*>(ptr + 0)) = data->coeffRef(0,i);
			*(reinterpret_cast<float*>(ptr + 4)) = data->coeffRef(1,i);
			*(reinterpret_cast<float*>(ptr + 8)) = data->coeffRef(2,i);
			ptr += POINT_STEP;
		}
	}
}


void RadarPCLFilter::filter_pointcloud(float ground_threshold, float drone_threshold, Eigen::MatrixXf * data_in, Eigen::MatrixXf * data_out) {
	
	int pcl_size = data_in->cols();

	int filtered_pcl_size = 0;


	for (size_t i = 0; i < (size_t)pcl_size; i++)
	{
		if ( ( data_in->coeffRef(2,i) > ground_threshold )  && ( data_in->coeffRef(2,i) < (_height_above_ground-drone_threshold) ) )
		{
			data_out->col(filtered_pcl_size) = data_in->col(i);
			filtered_pcl_size++;
			// RCLCPP_INFO(this->get_logger(), "valid point");
		}
		// else {
		// 	RCLCPP_INFO(this->get_logger(), "invalid point");
		// 	RCLCPP_INFO(this->get_logger(), "Height: %f", _height_above_ground);
		// 	RCLCPP_INFO(this->get_logger(), "Point: %f", data_in->coeffRef(2,i));

		// }
	}

	data_out->conservativeResize(4,filtered_pcl_size);

}


void RadarPCLFilter::read_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg, Eigen::MatrixXf * data_holder) {

  // read PointCloud2 msg data
  int pcl_size = msg->width;
  uint8_t *ptr = msg->data.data();
  const uint32_t POINT_STEP = 12;

  for (size_t i = 0; i < (size_t)pcl_size; i++) {

	data_holder->coeffRef(0,i) = *(reinterpret_cast<float*>(ptr + 0)); // x
	data_holder->coeffRef(1,i) = *(reinterpret_cast<float*>(ptr + 4)); // y
	data_holder->coeffRef(2,i) = *(reinterpret_cast<float*>(ptr + 8)); // z
	data_holder->coeffRef(3,i) = 1.0;

	ptr += POINT_STEP;

  }
}


void RadarPCLFilter::transform_pointcloud_to_world(const sensor_msgs::msg::PointCloud2::SharedPtr msg){

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

	int pcl_size = msg->width;

	Eigen::MatrixXf local_points(4,pcl_size);

	RadarPCLFilter::read_pointcloud(msg, &local_points);

	Eigen::MatrixXf world_points(4,pcl_size);

	world_points = world_to_drone * local_points;

	// filter pointcloud based on distance

	Eigen::MatrixXf filtered_world_points(4,pcl_size);

	RadarPCLFilter::filter_pointcloud(1, 0.2, &world_points, &filtered_world_points);

	// publish transformed pointcloud

	auto pcl_msg = sensor_msgs::msg::PointCloud2();

	RadarPCLFilter::create_pointcloud_msg(&filtered_world_points, &pcl_msg);

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

