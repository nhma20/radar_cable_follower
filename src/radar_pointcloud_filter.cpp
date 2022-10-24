#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>


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


using namespace std::chrono_literals;

//creates a RadarPCLFilter class that subclasses the generic rclcpp::Node base class.
class RadarPCLFilter : public rclcpp::Node
{

//Creates a function for when messages are to be sent. 
	public:
		RadarPCLFilter() : Node("radar_pcl_filter_node") {
			
			raw_pcl_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
			"/iwr6843_pcl",	10,
			std::bind(&RadarPCLFilter::OnDepthMsg, this, std::placeholders::_1));

		}

		~RadarPCLFilter() {
			RCLCPP_INFO(this->get_logger(),  "Shutting down radar_pcl_filter_node..");
		}

	private:
		rclcpp::TimerBase::SharedPtr timer_;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pcl_publisher_;
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pcl_subscription_;

		void OnDepthMsg(const sensor_msgs::msg::PointCloud2::SharedPtr _msg);
};



// mmwave message callback function
void RadarPCLFilter::OnDepthMsg(const sensor_msgs::msg::PointCloud2::SharedPtr _msg){
	// read PointCloud2 msg data
	int pcl_size = _msg->width;
	uint8_t *ptr = _msg->data.data();
	const uint32_t  POINT_STEP = 12;
	std::vector<float> pcl_x;
	std::vector<float> pcl_y;
	std::vector<float> pcl_z;
	for (size_t i = 0; i < pcl_size; i++)
	{
		float temp_z = *(reinterpret_cast<float*>(ptr + 8));


		pcl_x.push_back(*(reinterpret_cast<float*>(ptr + 0)));
		pcl_y.push_back(*(reinterpret_cast<float*>(ptr + 4)));
		pcl_z.push_back(*(reinterpret_cast<float*>(ptr + 8)));
		ptr += POINT_STEP;
	}
	float closest_dist = std::numeric_limits<float>::max(); 
	float current_dist = 0;
	int closest_dist_idx = 0;
	// find closest point in pointcloud msg, store all angles and dists
	this->objects_dists.clear();
	this->objects_xz_angle.clear();
	this->objects_yz_angle.clear();
	this->closest_idx = 0;
	if(pcl_size > 0){
		for (int i = 0; i < pcl_size; i++)
		{
			current_dist = sqrt( pow(pcl_x.at(i), 2) + pow(pcl_y.at(i), 2) + pow(pcl_z.at(i), 2) );

			this->objects_dists.push_back(current_dist);
			this->objects_xz_angle.push_back( asin(pcl_x.at(i) / sqrt(pow(pcl_x.at(i),2) + pow(pcl_z.at(i),2))) );
			this->objects_yz_angle.push_back( asin(pcl_y.at(i) / sqrt(pow(pcl_y.at(i),2) + pow(pcl_z.at(i),2))) );

			if( current_dist < closest_dist ){
				closest_dist = current_dist;
				this->closest_idx = i;
			}
		}
		/*std::cout << "closest dist: " << this->objects_dists.at(this->closest_idx) << std::endl;
		std::cout << "closest xz: " << this->objects_xz_angle.at(this->closest_idx) << std::endl;
		std::cout << "closest yz: " << this->objects_yz_angle.at(this->closest_idx) << std::endl;*/
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
