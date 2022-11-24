#pragma once

/*****************************************************************************/
// Includes
/*****************************************************************************/

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

/*****************************************************************************/
// Defines
/*****************************************************************************/

typedef Eigen::Vector3f point_t;

typedef Eigen::Vector3f orientation_t;

typedef Eigen::Vector4f quat_t;

typedef Eigen::Vector4f homog_point_t;

typedef Eigen::Vector3f vector_t;

typedef struct {

    point_t p;
    vector_t v;

} line_t;

typedef struct {

    point_t p;
    vector_t normal;

} plane_t;

typedef Eigen::Matrix3f rotation_matrix_t;

typedef Eigen::Matrix4f transform_t;

typedef Eigen::Matrix4f homog_transform_t;

typedef struct {

	point_t position;
	quat_t quaternion;
    
} line_model_t;

typedef struct {

	point_t position;
	quat_t quaternion;
    
} pose_t;

typedef struct {

	point_t position;
	orientation_t orientation;
    
} pose_eul_t;

typedef struct {

    point_t point;
    int id;
    int alive_count;

} id_point_t;




/*****************************************************************************/
// Function declarations
/*****************************************************************************/

rotation_matrix_t eulToR(orientation_t eul);

vector_t rotateVector(rotation_matrix_t R, vector_t v);

point_t projectPointOnPlane(point_t point, plane_t plane);

orientation_t quatToEul(quat_t quat);

quat_t quatInv(quat_t quat);

quat_t quatMultiply(quat_t quat1, quat_t quat2);

rotation_matrix_t quatToMat(quat_t quat);

quat_t matToQuat(rotation_matrix_t R);

quat_t eulToQuat(orientation_t eul);

transform_t getTransformMatrix(vector_t vec, quat_t quat);

plane_t create_plane(quat_t powerline_direction, point_t drone_xyz);

pose_eul_t pose_NWU_to_NED(pose_eul_t NWU_pose);