#ifndef SIMPLE_DVO_IMAGE_ALIGNMENT_H
#define SIMPLE_DVO_IMAGE_ALIGNMENT_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <opencv2/core/core.hpp>
#include "ros/ros.h"


/**
 * @class DirectImageAlignment
 * @brief Class for direct image alignment between 2 images.
 */
class DirectImageAlignment {
private:
    //Parameters
    static const int num_pyramid = 5;
    static const int num_GNiterations = 20;

    //Image and camera matrix.
    cv::Mat img_cur;
    cv::Mat img_prev;
    cv::Mat depth_cur;
    cv::Mat depth_prev;
    Eigen::Matrix3f K;

    //Image and camera matrix pyramids.
    cv::Mat img_prev_Pyramid[num_pyramid];
    cv::Mat depth_prev_Pyramid[num_pyramid];
    cv::Mat img_cur_Pyramid[num_pyramid];
    cv::Mat depth_cur_Pyramid[num_pyramid];
    Eigen::Matrix3f k_Pyramid[num_pyramid];

    //Robust weight estimation
    static constexpr float INITIAL_SIGMA = 5.0f;
    static constexpr float DEFAULT_DOF = 5.0f;

public:

    /**
     * @brief Gray Image downsampling to make pyramid.
     */
    cv::Mat downsampleImg(const cv::Mat &gray);

    /**
     * @brief Depth Image downsampling to make pyramid.
     */
    cv::Mat downsampleDepth(const cv::Mat &depth);

    /**
     * @brief Make pyramids of gray/depth image and camera matrix.
     */
    void makePyramid();

    /**
     * @brief Calculate image intensity gradients.
     * @param[in] direction: xdirection(0) or ydirection(1).
     */
    void calcGradient(const cv::Mat &img, cv::Mat &gradient, int direction);

    /**
     * @brief Calculate residual (photometric error between previous image and current image)
     */
    Eigen::VectorXf calcRes(const Eigen::VectorXf &xi, const int level);

    /**
     * @brief Calculate Jacobian for minimizing least square error.
     */
    Eigen::MatrixXf calcJacobian(const Eigen::VectorXf &xi, const int level);

    /**
     * @brief Compute robust weights from residuals.
     */
    void weighting(Eigen::VectorXf &residuals, Eigen::VectorXf &weights);

    /**
     * @brief Execute GaussNewton optimzation and find the optimam rotation and translation.
     * @param[out] rot, t: camera pose transformation between 2 images.
     */
    void doGaussNewton(Eigen::Matrix3f& rot, Eigen::Vector3f& t);

    /**
     * @brief Execute direct image alignment between last and current image.
     * @param[out] transform: camera pose transformation between 2 images.
     */
    void doAlignment( Eigen::Matrix4f& transform, const cv::Mat& img_prev, const cv::Mat& depth_prev,
                      const cv::Mat& img_cur, const cv::Mat& depth_cur, const Eigen::Matrix3f& K);
};



#endif //SIMPLE_DVO_IMAGE_ALIGNMENT_H
