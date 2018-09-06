

#ifndef SIMPLE_DVO_UTIL_H
#define SIMPLE_DVO_UTIL_H
#include <sophus/se3.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

/**
 * @brief bilinear interporation to compute the pixel intensity at (x, y).
 */
float interpolate(const float* img_ptr, float x, float y, int w, int h);

/**
 * @brief Conversion from se(3) to rotation/translation matrix.
 */
void SE3ToRt(const Eigen::VectorXf &xi, Eigen::Matrix3f &rot, Eigen::Vector3f &t);

/**
 * @brief Conversion from rotation/translation matrix to se(3).
 */
void RtToSE3(const Eigen::Matrix3f &rot, const Eigen::Vector3f &t, Eigen::VectorXf &xi);

#endif //SIMPLE_DVO_UTIL_H
