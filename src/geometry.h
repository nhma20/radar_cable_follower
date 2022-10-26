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