#include "Reconstruction/depthestimation.h"
using namespace cv::ximgproc;


RectifiedImages DepthEstimation::stereoRectification(const Mat &left_lower, const Mat &left_upper,
                                                       const intrinsicCalibration &iC, const CalibrationMatrix &sC)
{

    Mat rmap[2][2];
   //Precompute maps for cv::remap()
    initUndistortRectifyMap(iC.M1, iC.D1, sC.R1, sC.P1, left_lower.size(), CV_16SC2, rmap[0][0], rmap[0][1]);
    initUndistortRectifyMap(iC.M2, iC.D2, sC.R2, sC.P2, left_upper.size(), CV_16SC2, rmap[1][0], rmap[1][1]);

    RectifiedImages rectified;
    remap(left_lower, rectified.left_rect_lower, rmap[0][0], rmap[0][1], INTER_LINEAR);
    remap(left_upper, rectified.left_rect_upper, rmap[1][0], rmap[1][1], INTER_LINEAR);

    return rectified;
}


Disparities DepthEstimation::disparityFiltering(const RectifiedImages &rectified_images)
{
       Mat left_rect_lower, left_rect_upper;
       left_rect_lower = rectified_images.left_rect_lower;
       left_rect_upper = rectified_images.left_rect_upper;

         Mat left_for_matcher, right_for_matcher;
         Mat left_disp,right_disp;
         Mat filtered_disp,solved_disp,solved_filtered_disp;
         int max_disp =16;
         int wsize = 7;
         double lambda = 8000.0;
         double sigma = 1.5;

    //    resize(left ,left_for_matcher ,Size(),0.5,0.5, INTER_LINEAR);
    //    resize(right,right_for_matcher,Size(),0.5,0.5, INTER_LINEAR);
         left_for_matcher  = left_rect_lower.clone();
         right_for_matcher = left_rect_upper.clone();


         Ptr<DisparityWLSFilter> wls_filter;
         double matching_time, filtering_time;

         Ptr<StereoBM> left_matcher = StereoBM::create(max_disp,wsize);
         wls_filter = createDisparityWLSFilter(left_matcher);
         Ptr<StereoMatcher> right_matcher = createRightMatcher(left_matcher);
         cvtColor(left_for_matcher,  left_for_matcher,  COLOR_BGR2GRAY);
         cvtColor(right_for_matcher, right_for_matcher, COLOR_BGR2GRAY);

         matching_time = (double)getTickCount();
         left_matcher-> compute(left_for_matcher, right_for_matcher,left_disp);
         right_matcher->compute(right_for_matcher,left_for_matcher, right_disp);
         matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

         wls_filter->setLambda(lambda);
         wls_filter->setSigmaColor(sigma);
         filtering_time = (double)getTickCount();
         wls_filter->filter(left_disp,left_rect_lower,filtered_disp,right_disp);
         filtering_time = ((double)getTickCount() - filtering_time)/getTickFrequency();
         Disparities disp;

         disp.disparity = filtered_disp;
         normalize(filtered_disp, disp.disparity_norm, 0, 255, CV_MINMAX, CV_8U);
         return disp;

}

DepthImage DepthEstimation::dispToDepth(const Disparities &disp, const CalibrationMatrix &sC)
{
    DepthImage depth;
    reprojectImageTo3D(disp.disparity_norm, depth.depth_image, sC.Q, false, CV_32F);
    return depth;
}

