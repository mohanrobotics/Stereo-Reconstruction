#ifndef DEPTHESTIMATION_H
#define DEPTHESTIMATION_H

#include<Reconstruction/cameracalibration.h>

struct RectifiedImages {
    Mat left_rect_lower, left_rect_upper;
};

struct Disparities {
    Mat disparity, disparity_norm;
};

struct DepthImage {
    Mat depth_image;
};


/**
 * Class name: DepthEstimation
 * Description: Contains the functionalities for obtaining disparity and depth images
*/


class DepthEstimation
{
public:
    DepthEstimation(){}
    virtual ~DepthEstimation() {}


    static RectifiedImages stereoRectification(const Mat &left_lower,
                                               const Mat &left_upper,
                                               const intrinsicCalibration &iC,
                                               const CalibrationMatrix &sC); // Rectifying the stereo images
    static Disparities disparityFiltering(
            const RectifiedImages &rectified_images); // Obtaining the disparitiess and filtering

    static DepthImage dispToDepth(const Disparities &disp,
                                  const CalibrationMatrix &sC); // Disparities into depth map

};

#endif // DEPTH_ESTIMATION_H
