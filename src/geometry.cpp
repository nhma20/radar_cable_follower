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

    line_t l = {
        .p = point,
        .v = plane.normal
    };

    float t = - plane.normal.dot(l.p) / plane.normal.dot(plane.normal);

    point_t proj_point = l.p + (point_t)(t*l.v);

    return proj_point;

}

orientation_t quatToEul(quat_t quat) {

    Eigen::Quaternionf q;
    q.x() = quat[0];
    q.y() = quat[1];
    q.z() = quat[2];
    q.w() = quat[3];

    orientation_t eul  = q.toRotationMatrix().eulerAngles(0, 1, 2);

    return eul;

}

quat_t quatInv(quat_t quat) {

    quat_t ret_quat(quat[0], -quat[1], -quat[2], -quat[3]);

    return ret_quat;

}

quat_t quatMultiply(quat_t quat1, quat_t quat2) {

    quat_t ret_quat(
        quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
        quat1[0]*quat2[1] + quat1[1]*quat2[0] + quat1[2]*quat2[3] - quat1[3]*quat2[2],
        quat1[0]*quat2[2] - quat1[1]*quat2[3] + quat1[2]*quat2[0] + quat1[3]*quat2[1],
        quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1] + quat1[3]*quat2[0]
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

    return quat;

}

transform_t getTransformMatrix(vector_t vec, quat_t quat) {

    Eigen::Matrix4f Trans; // Transformation Matrix

    Trans.setIdentity();   // Set to Identity to make bottom row of Matrix 0,0,0,1
    
    Trans.block<3,3>(0,0) = quatToMat(quat); // Copy rotation matrix into Trans
    
    Trans.block<3,1>(0,3) = vec; // Copy translation matrix into Trans

    return Trans;
}







// /*****************************************************************************/
// // Includes
// /*****************************************************************************/

// #include "geometry.h"
// #include <iostream>

// /*****************************************************************************/
// // Function implementations
// /*****************************************************************************/

// rotation_matrix_t eulToR(orientation_t eul) {

//     float cos_yaw = cos(eul[2]);
//     float cos_pitch = cos(eul[1]);
//     float cos_roll = cos(eul[0]);
//     float sin_yaw = sin(eul[2]);
//     float sin_pitch = sin(eul[1]);
//     float sin_roll = sin(eul[0]);

//     rotation_matrix_t mat;

//     mat(0,0) = cos_pitch*cos_yaw;
//     mat(0,1) = sin_roll*sin_pitch*cos_yaw-cos_roll*sin_yaw;
//     mat(0,2) = cos_roll*sin_pitch*cos_yaw+sin_roll*sin_yaw;
//     mat(1,0) = cos_pitch*sin_yaw;
//     mat(1,1) = sin_roll*sin_pitch*sin_yaw+cos_roll*cos_yaw;
//     mat(1,2) = cos_roll*sin_pitch*sin_yaw-sin_roll*cos_pitch; // wrong? cos_roll*sin_pitch*sin_yaw-sin_roll*cos_yaw
//     mat(2,0) = -sin_pitch;
//     mat(2,1) = sin_roll*cos_pitch;
//     mat(2,2) = cos_roll*cos_pitch;

//     return mat;

// }

// vector_t rotateVector(rotation_matrix_t R, vector_t v) {

//     vector_t ret_vec = R*v;

//     return ret_vec;
// }

// point_t projectPointOnPlane(point_t point, plane_t plane) {

//     line_t l = {
//         .p = point,
//         .v = plane.normal
//     };

//     float t = - plane.normal.dot(l.p) / plane.normal.dot(plane.normal);

//     point_t proj_point = l.p + (point_t)(t*l.v);

//     return proj_point;

// }

// orientation_t quatToEul(quat_t quat) {

//     orientation_t eul(
//         atan2(2*(quat[0]*quat[1] + quat[2]*quat[3]), 1-2*(quat[1]*quat[1] + quat[2]*quat[2])),
//         asin(2*(quat[0]*quat[2] - quat[3]*quat[1])),
//         atan2(2*(quat[0]*quat[3] + quat[1]*quat[2]), 1-2*(quat[2]*quat[2]+quat[3]*quat[3]))
//     );

//     return eul;

// }

// quat_t quatInv(quat_t quat) {

//     quat_t ret_quat(quat[0], -quat[1], -quat[2], -quat[3]);

//     return ret_quat;

// }

// quat_t quatMultiply(quat_t quat1, quat_t quat2) {

//     quat_t ret_quat(
//         quat1[0]*quat2[0] - quat1[1]*quat2[1] - quat1[2]*quat2[2] - quat1[3]*quat2[3],
//         quat1[0]*quat2[1] + quat1[1]*quat2[0] + quat1[2]*quat2[3] - quat1[3]*quat2[2],
//         quat1[0]*quat2[2] - quat1[1]*quat2[3] + quat1[2]*quat2[0] + quat1[3]*quat2[1],
//         quat1[0]*quat2[3] + quat1[1]*quat2[2] - quat1[2]*quat2[1] + quat1[3]*quat2[0]
//     );

//     return ret_quat;

// }

// rotation_matrix_t quatToMat(quat_t quat) {

//     rotation_matrix_t mat;
//     //mat(0,0) = 1-2*quat[2]*quat[2]-2*quat[3]*quat[3];
//     //mat(0,1) = 2*quat[1]*quat[2]-2*quat[0]*quat[3];
//     //mat(0,2) = 2*quat[1]*quat[3]+2*quat[0]*quat[2];
//     //mat(1,0) = 2*quat[1]*quat[2]+2*quat[0]*quat[3];
//     //mat(1,1) = 1-2*quat[1]*quat[1]-2*quat[3]*quat[3];
//     //mat(1,2) = 2*quat[2]*quat[3]-2*quat[0]*quat[1];
//     //mat(2,0) = 2*quat[1]*quat[3]-2*quat[0]*quat[3];
//     //mat(2,1) = 2*quat[2]*quat[3]+2*quat[0]*quat[1];
//     //mat(2,2) = 1-2*quat[1]*quat[1]-2*quat[2]*quat[2];

//     float q0 = quat[0];
//     float q1 = quat[1];
//     float q2 = quat[2];
//     float q3 = quat[3];
     
//     // First row of the rotation matrix
//     float r00 = 2 * (q0 * q0 + q1 * q1) - 1;
//     float r01 = 2 * (q1 * q2 - q0 * q3) ;
//     float r02 = 2 * (q1 * q3 + q0 * q2) ;
     
//     // Second row of the rotation matrix
//     float r10 = 2 * (q1 * q2 + q0 * q3);
//     float r11 = 2 * (q0 * q0 + q2 * q2) - 1;
//     float r12 = 2 * (q2 * q3 - q0 * q1);
     
//     // Third row of the rotation matrix
//     float r20 = 2 * (q1 * q3 - q0 * q2);
//     float r21 = 2 * (q2 * q3 + q0 * q1);
//     float r22 = 2 * (q0 * q0 + q3 * q3) - 1;
     
//     // 3x3 rotation matrix
//     mat(0,0) = r00;
//     mat(0,1) = r01;
//     mat(0,2) = r02;
//     mat(1,0) = r10;
//     mat(1,1) = r11;
//     mat(1,2) = r12;
//     mat(2,0) = r20;
//     mat(2,1) = r21;
//     mat(2,2) = r22;

//     return mat;

// }

// quat_t matToQuat(rotation_matrix_t R) {

//     float tr = R(0,0) + R(1,1) + R(2,2);

//     float qw, qx, qy, qz;

//     if (tr > 0) { 

//         float S = sqrt(tr+1.0) * 2; // S=4*qw 
//         qw = 0.25 * S;
//         qx = (R(2,1) - R(1,2)) / S;
//         qy = (R(0,2) - R(2,0)) / S; 
//         qz = (R(1,0) - R(0,1)) / S; 

//     } else if ((R(0,0) > R(1,1)) && (R(0,0) > R(2,2))) { 

//         float S = sqrt(1.0 + R(0,0) - R(1,1) - R(2,2)) * 2; // S=4*qx 
//         qw = (R(2,1) - R(1,2)) / S;
//         qx = 0.25 * S;
//         qy = (R(0,1) + R(1,0)) / S; 
//         qz = (R(0,2) + R(2,0)) / S; 

//     } else if (R(1,1) > R(2,2)) { 

//         float S = sqrt(1.0 + R(1,1) - R(0,0) - R(2,2)) * 2; // S=4*qy
//         qw = (R(0,2) - R(2,0)) / S;
//         qx = (R(0,1) + R(1,0)) / S; 
//         qy = 0.25 * S;
//         qz = (R(1,2) + R(2,1)) / S; 

//     } else { 

//         float S = sqrt(1.0 + R(2,2) - R(0,0) - R(1,1)) * 2; // S=4*qz
//         qw = (R(1,0) - R(0,1)) / S;
//         qx = (R(0,2) + R(2,0)) / S;
//         qy = (R(1,2) + R(2,1)) / S;
//         qz = 0.25 * S;

//     }

//     quat_t quat(qw, qx, qy, qz);

//     return quat;

// }

// quat_t eulToQuat(orientation_t eul) {

//     // Abbreviations for the various angular functions
//     float cy = cos(eul(2) * 0.5);
//     float sy = sin(eul(2) * 0.5);
//     float cp = cos(eul(1) * 0.5);
//     float sp = sin(eul(1) * 0.5);
//     float cr = cos(eul(0) * 0.5);
//     float sr = sin(eul(0) * 0.5);

//     quat_t q;
//     q(0) = cr * cp * cy + sr * sp * sy;
//     q(1) = sr * cp * cy - cr * sp * sy;
//     q(2) = cr * sp * cy + sr * cp * sy;
//     q(3) = cr * cp * sy - sr * sp * cy;

//     return q;

// }

// transform_t getTransformMatrix(vector_t vec, quat_t quat) {

//     transform_t T;

//     rotation_matrix_t R = quatToMat(quat);

//     for ( int i = 0; i < 3; i++) {

//         for (int j = 0; j < 3; j++) {

//             T(i,j) = R(i,j);
//         }

//         T(i,3) = vec(i);

//     }

//     T(3,0) = 0;
//     T(3,1) = 0;
//     T(3,2) = 0;
//     T(3,3) = 1;

//     return T;

// }