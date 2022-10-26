/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <px4_msgs/msg/vehicle_odometry.hpp>

#include <memory>
#include <string>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>


/*****************************************************************************/
// Defines
/*****************************************************************************/
typedef Eigen::Vector3f orientation_t;

typedef Eigen::Vector4f quat_t;

typedef Eigen::Matrix3f rotation_matrix_t;

typedef Eigen::Vector3f point_t;
/*****************************************************************************/
// Class
/*****************************************************************************/

class DroneFrameBroadcasterNode : public rclcpp::Node {
public:
explicit
    DroneFrameBroadcasterNode(const std::string & node_name="drone_frame_broadcaster", const std::string & node_namespace="/drone_frame_broadcaster")
            : Node(node_name, node_namespace) {

        // Params
        this->declare_parameter<std::string>("drone_frame_id", "drone");
        this->declare_parameter<std::string>("world_frame_id", "world");

        this->get_parameter("drone_frame_id", drone_frame_id_);
        this->get_parameter("world_frame_id", world_frame_id_);


        // Initialize the transform broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        std::ostringstream stream;

        subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/vehicle_odometry/out", 10,
            std::bind(&DroneFrameBroadcasterNode::odometryCallback, this, std::placeholders::_1));

        R_NED_to_body_frame = DroneFrameBroadcasterNode::eulToR(orientation_t(M_PI, 0, 0));
    }

private:
    void odometryCallback(const std::shared_ptr<px4_msgs::msg::VehicleOdometry> msg) {

        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = now;
        t.header.frame_id = world_frame_id_;
        t.child_frame_id = drone_frame_id_;

        point_t position(
            msg->position[0],
            msg->position[1], 
            msg->position[2]
        );

        position = R_NED_to_body_frame * position;

        quat_t quat(
            msg->q[0],
            msg->q[1],
            msg->q[2],
            msg->q[3]
        );

        orientation_t eul = DroneFrameBroadcasterNode::quatToEul(quat);
        eul(1) = -eul(1);                       // Dirty hack
        eul(2) = -eul(2);
        quat = DroneFrameBroadcasterNode::eulToQuat(eul);

        t.transform.translation.x = position(0);
        t.transform.translation.y = position(1);
        t.transform.translation.z = position(2);

        t.transform.rotation.w = quat(0);
        t.transform.rotation.x = quat(1);
        t.transform.rotation.y = quat(2);
        t.transform.rotation.z = quat(3);

        // Send the transformation
        tf_broadcaster_->sendTransform(t);

    }

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rotation_matrix_t R_NED_to_body_frame;

    std::string drone_frame_id_;
    std::string world_frame_id_;

    rotation_matrix_t eulToR(orientation_t eul);

    orientation_t quatToEul(quat_t quat);

    quat_t eulToQuat(orientation_t eul);

};


rotation_matrix_t DroneFrameBroadcasterNode::eulToR(orientation_t eul) {

    float cos_yaw = cos(eul[2]);
    float cos_pitch = cos(eul[1]);
    float cos_roll = cos(eul[0]);
    float sin_yaw = sin(eul[2]);
    float sin_pitch = sin(eul[1]);
    float sin_roll = sin(eul[0]);

    rotation_matrix_t mat;

    mat(0,0) = cos_pitch*cos_yaw;
    mat(0,1) = sin_roll*sin_pitch*cos_yaw-cos_roll*sin_yaw;
    mat(0,2) = cos_roll*sin_pitch*cos_yaw+sin_roll*sin_yaw;
    mat(1,0) = cos_pitch*sin_yaw;
    mat(1,1) = sin_roll*sin_pitch*sin_yaw+cos_roll*cos_yaw;
    mat(1,2) = cos_roll*sin_pitch*sin_yaw-sin_roll*cos_pitch; // wrong? cos_roll*sin_pitch*sin_yaw-sin_roll*cos_yaw
    mat(2,0) = -sin_pitch;
    mat(2,1) = sin_roll*cos_pitch;
    mat(2,2) = cos_roll*cos_pitch;

    return mat;

}


orientation_t DroneFrameBroadcasterNode::quatToEul(quat_t quat) {

    orientation_t eul(
        atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]), 1-2*(quat[1]*quat[1] + quat[2]*quat[2])),
        asin(2*(quat[0]*quat[2] - quat[3]*quat[1])),
        atan2(2*(quat[0]*quat[3] + quat[1]*quat[2]), 1-2*(quat[2]*quat[2]+quat[3]*quat[3]))
    );

    return eul;

}


quat_t DroneFrameBroadcasterNode::eulToQuat(orientation_t eul) {

    // Abbreviations for the various angular functions
    float cy = cos(eul(2) * 0.5);
    float sy = sin(eul(2) * 0.5);
    float cp = cos(eul(1) * 0.5);
    float sp = sin(eul(1) * 0.5);
    float cr = cos(eul(0) * 0.5);
    float sr = sin(eul(0) * 0.5);

    quat_t q;
    q(0) = cr * cp * cy + sr * sp * sy;
    q(1) = sr * cp * cy - cr * sp * sy;
    q(2) = cr * sp * cy + sr * cp * sy;
    q(3) = cr * cp * sy - sr * sp * cy;

    return q;

}



/*****************************************************************************/
// Main
/*****************************************************************************/

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DroneFrameBroadcasterNode>());
    rclcpp::shutdown();
    return 0;
}