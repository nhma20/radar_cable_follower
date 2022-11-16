/*****************************************************************************/
// Includes
/*****************************************************************************/

#include "geometry.h"
#include <iostream>

/*****************************************************************************/
// Function implementations
/*****************************************************************************/

rotation_matrix_t eulToR(orientation_t eul) {

    Eigen::Quaternionf q; 

    q = Eigen::AngleAxisf(eul[0], Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(eul[1], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(eul[2], Eigen::Vector3f::UnitZ());

    Eigen::Matrix3f rotationMatrix = q.normalized().toRotationMatrix();

    return rotationMatrix;

}

vector_t rotateVector(rotation_matrix_t R, vector_t v) {

    vector_t ret_vec = R*v;

    return ret_vec;
}

point_t projectPointOnPlane(point_t point, plane_t plane) {
//     line_t l = {
//         .p = point,
//         .v = plane.normal
//     };

//     float t = - plane.normal.dot(l.p) / plane.normal.dot(plane.normal);

//     point_t proj_point = l.p + (point_t)(t*l.v);

//     return proj_point;

    vector_t diff = point - plane.p;

    float dist = diff.dot(plane.normal);

    point_t proj_point = point - (point_t)(dist*plane.normal);

    return proj_point;

}

orientation_t quatToEul(quat_t quat) {

    // Eigen::Quaternionf q;
    // q.x() = quat[0];
    // q.y() = quat[1];
    // q.z() = quat[2];
    // q.w() = quat[3];

    // orientation_t eul  = q.toRotationMatrix().eulerAngles(0, 1, 2);

    // thanks wikipedia
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Quaternion_to_Euler_angles_conversion
    orientation_t eul;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quat[3] * quat[0] + quat[1] * quat[2]);
    double cosr_cosp = 1 - 2 * (quat[0] * quat[0] + quat[1] * quat[1]);
    eul(0) = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (quat[3] * quat[1] - quat[2] * quat[0]);
    if (std::abs(sinp) >= 1)
        eul(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        eul(1) = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quat[3] * quat[2] + quat[0] * quat[1]);
    double cosy_cosp = 1 - 2 * (quat[1] * quat[1] + quat[2] * quat[2]);
    eul(2) = std::atan2(siny_cosp, cosy_cosp);


    return eul;

}

quat_t quatInv(quat_t quat) {

    quat_t ret_quat(quat[0], -quat[1], -quat[2], -quat[3]);

    return ret_quat;

}

quat_t quatMultiply(quat_t quat1, quat_t quat2) {

    quat_t ret_quat(
        // quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
        // quat1[0]*quat2[1] + quat1[1]*quat2[0] + quat1[2]*quat2[3] - quat1[3]*quat2[2],
        // quat1[0]*quat2[2] - quat1[1]*quat2[3] + quat1[2]*quat2[0] + quat1[3]*quat2[1],
        // quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1] + quat1[3]*quat2[0]
         quat1[0] * quat2[3] + quat1[1] * quat2[2] - quat1[2] * quat2[1] + quat1[3] * quat2[0],
        -quat1[0] * quat2[2] + quat1[1] * quat2[3] + quat1[2] * quat2[0] + quat1[3] * quat2[1],
         quat1[0] * quat2[1] - quat1[1] * quat2[0] + quat1[2] * quat2[3] + quat1[3] * quat2[2],
        -quat1[0] * quat2[0] - quat1[1] * quat2[1] - quat1[2] * quat2[2] + quat1[3] * quat2[3]
    );

    return ret_quat;

}

rotation_matrix_t quatToMat(quat_t quat) {

    Eigen::Quaternionf q;
    q.x() = quat[0];
    q.y() = quat[1];
    q.z() = quat[2];
    q.w() = quat[3];
    Eigen::Matrix3f test_ER = q.normalized().toRotationMatrix(); //.cast<float>()

    return test_ER;

}

quat_t matToQuat(rotation_matrix_t R) {

    Eigen::Quaternionf q(R);

    quat_t quat(q.x(), q.y(), q.z(), q.w());

    return quat;

}

quat_t eulToQuat(orientation_t eul) {

    Eigen::Quaternionf q; 

    q = Eigen::AngleAxisf(eul[0], Eigen::Vector3f::UnitX())
        * Eigen::AngleAxisf(eul[1], Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(eul[2], Eigen::Vector3f::UnitZ());


    quat_t quat;
    quat(0) = q.x();
    quat(1) = q.y();
    quat(2) = q.z();
    quat(3) = q.w();


    // Abbreviations for the various angular functions
    // double cr = cos(eul(0) * 0.5);
    // double sr = sin(eul(0) * 0.5);
    // double cp = cos(eul(1) * 0.5);
    // double sp = sin(eul(1) * 0.5);
    // double cy = cos(eul(2) * 0.5);
    // double sy = sin(eul(2) * 0.5);

    // quat_t quat;
    // quat(0) = sr * cp * cy - cr * sp * sy;
    // quat(1) = cr * sp * cy + sr * cp * sy;
    // quat(2) = cr * cp * sy - sr * sp * cy;
    // quat(3) = cr * cp * cy + sr * sp * sy;

    return quat;
}

transform_t getTransformMatrix(vector_t vec, quat_t quat) {

    Eigen::Matrix4f Trans; // Transformation Matrix

    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    
    Trans.block<3,3>(0,0) = quatToMat(quat); // Copy rotation matrix into Trans
    
    Trans.block<3,1>(0,3) = vec; // Copy translation matrix into Trans

    return Trans;
}

plane_t create_plane(quat_t powerline_direction, point_t drone_xyz) {

    vector_t unit_x(1, 0, 0);

    orientation_t eul = quatToEul(powerline_direction);

    //orientation_t rotation(0, -eul[1], direction_tmp);

    vector_t plane_normal = rotateVector(eulToR(eul), unit_x);

	plane_t projection_plane;

    projection_plane.p = drone_xyz;
    projection_plane.normal = plane_normal;

	return projection_plane;
}


pose_eul_t pose_NWU_to_NED(pose_eul_t NWU_pose) {

    static rotation_matrix_t R_NWU_to_NED = eulToR(orientation_t(-M_PI, 0, 0));

    pose_eul_t NED_pose;

    NED_pose.position = R_NWU_to_NED * NWU_pose.position;

    NED_pose.orientation(0) = NWU_pose.orientation(0); 
    NED_pose.orientation(1) = -NWU_pose.orientation(1);         // Dirty hack
    NED_pose.orientation(2) = -NWU_pose.orientation(2);

    return NED_pose;
}
