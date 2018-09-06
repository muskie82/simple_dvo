#include "util.h"


float interpolate(const float* img_ptr, float x, float y, int w, int h) {

    float val_itpltd = nan("1");
    int x0 = floor(x), y0 = floor(y);
    int x1 = x0 + 1, y1 = y0 + 1;

    float x1_weight = x - x0, y1_weight = y - y0;
    float x0_weight = 1 - x1_weight, y0_weight = 1 - y1_weight;

    //Check if warped points are in the image.
    if (x0 < 0 or x0 >= w)
        x0_weight = 0;
    if (x1 < 0 or x1 >= w)
        x1_weight = 0;
    if (y0 < 0 or y0 >= h)
        y0_weight = 0;
    if (y1 < 0 or y1 >= h)
        y1_weight = 0;
    float w00 = x0_weight * y0_weight;
    float w10 = x1_weight * y0_weight;
    float w01 = x0_weight * y1_weight;
    float w11 = x1_weight * y1_weight;

    //Compute interpolated pixel intensity.
    float sumWeights = w00 + w10 + w01 + w11;
    float total = 0;
    if (w00 > 0)
        total += img_ptr[y0*w + x0] * w00;
    if (w01 > 0)
        total += img_ptr[y1*w + x0] * w01;
    if (w10 > 0)
        total += img_ptr[y0*w + x1] * w10;
    if (w11 > 0)
        total += img_ptr[y1*w + x1] * w11;

    if (sumWeights > 0)
        val_itpltd = total / sumWeights;

    return val_itpltd;
}

void SE3ToRt(const Eigen::VectorXf &xi, Eigen::Matrix3f &rot, Eigen::Vector3f &t) {
    Sophus::SE3f se3 = Sophus::SE3f::exp(xi);
    Eigen::Matrix4f mat = se3.matrix();
    rot = mat.topLeftCorner(3, 3);
    t = mat.topRightCorner(3, 1);
}

void RtToSE3(const Eigen::Matrix3f &rot, const Eigen::Vector3f &t, Eigen::VectorXf &xi){
    Sophus::SE3f se3(rot, t);
    xi = Sophus::SE3f::log(se3);
}

