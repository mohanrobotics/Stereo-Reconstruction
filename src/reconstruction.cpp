#include "Reconstruction/cameracalibration.h"
#include "Reconstruction/depthestimation.h"
#include "Reconstruction/pclvisualizer.h"
#include <iostream>


int main(int /* argc */, char ** /*argv*/)
{
    std::string packagePath = PROJECT_SOURCE_DIR;

    // -- Setting the image load ans save paths
    // -----------------------------------------

    string calib_images_path = packagePath + "/data/inputs/calibration_images/*png";
    string stereo_images_path = packagePath + "/data/inputs/stereo_images/*png";
    string parameters_path = packagePath + "/data/outputs/camera_parameters/";
    string disp_images_path = packagePath + "/data/outputs/disparity_images/Phalaenopsis_Left";
    string disp_norm_images_path = packagePath + "/data/outputs/disparity_normalized/Phalaenopsis_Left";
    string depth_images_path = packagePath + "/data/outputs/depth_images/Phalaenopsis_Left";
    string point_cloud_path = packagePath + "/data/outputs/point_clouds/Phalaenopsis_Left";
    // -----------------------------------------


    // -- Camera Calibration
    // -----------------------------------------

    const imageSet calib_image_set = CameraCalibration::readCheckerboardImages(calib_images_path);
    const intrinsicCalibration iC = CameraCalibration::findCheckerboardCorners(calib_image_set);
    CameraCalibration::CalibrationQualityCheck(iC, calib_image_set);
    const CalibrationMatrix sC = CameraCalibration::stereoCalibration(iC,calib_image_set);
    CameraCalibration::write_data(iC, sC, parameters_path);
    // -----------------------------------------


    // -- Obtaining stereo images
    // -----------------------------------------

    Mat left_lower, left_upper;
    const imageSet stereo_image_set = CameraCalibration::readCheckerboardImages(stereo_images_path);
    vector<Mat> l_stereo_images = stereo_image_set.lower_images;
    vector<Mat> u_stereo_images = stereo_image_set.upper_images;
    // -----------------------------------------

    for (int s = 0; s <  stereo_image_set.numImage; s++)
    {
        left_lower = l_stereo_images[s];
        left_upper = u_stereo_images[s];
        string image_number = to_string(s);


        // -- Rectifying the  images
        // -----------------------------------------

        const RectifiedImages rectified_images =  DepthEstimation::stereoRectification(left_lower, left_upper, iC, sC);
        // -----------------------------------------


        // -- Finding the disparity in the images and saving
        // -----------------------------------------

        const Disparities disp = DepthEstimation::disparityFiltering(rectified_images);
        imwrite( disp_images_path + image_number + ".png", disp.disparity);
        imwrite( disp_norm_images_path + image_number + ".png", disp.disparity_norm);
        // -----------------------------------------


        // -- Obtaining the the depth image
        // -----------------------------------------
        const DepthImage depth_image =  DepthEstimation::dispToDepth(disp,sC);
        imwrite( depth_images_path + image_number + ".png", depth_image.depth_image);
        // -----------------------------------------


        // -- Creating the point cloud and saving it
        // -----------------------------------------
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr =  PclVisualizer::CreatingPointCloud(depth_image.depth_image, rectified_images.left_rect_lower, iC);
        pcl::io::savePLYFileBinary(point_cloud_path + image_number + ".ply", *cloudPtr);
        // -----------------------------------------

        // -- Point Cloud Visualization
        // -----------------------------------------
        PclVisualizer::VisualizingPointCloud(cloudPtr);
        // -----------------------------------------

     }

    return 0;
}
