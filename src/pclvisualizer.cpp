#include "Reconstruction/pclvisualizer.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PclVisualizer::CreatingPointCloud(const Mat &depth_img, const Mat &left_rect_1, const intrinsicCalibration &iC)

{
    const double camera_factor = 200000;
    const double camera_cx = iC.M1 .at<double>(0,2);
    const double camera_cy = iC.M1 .at<double>(1,2);
    const double camera_fx = iC.M1 .at<double>(0,0);
    const double camera_fy = iC.M1 .at<double>(1,1);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int m = 0; m < depth_img.rows; m++)
    {
        for (int n = 0; n < depth_img.cols; n++)
        {
            ushort d = depth_img.ptr<ushort>(m)[n];
            if (d == 0)
            {
                continue;
            }
            pcl::PointXYZRGB p;
            p.z = double(d) / camera_factor;
            p.x = (n - camera_cx) * p.z / camera_fx;
            p.y = (m - camera_cy) * p.z / camera_fy;

            p.r = left_rect_1.at<cv::Vec3b>(m,n)[2];
            p.g = left_rect_1.at<cv::Vec3b>(m,n)[1];
            p.b = left_rect_1.at<cv::Vec3b>(m,n)[0];
            cloudPtr->points.push_back(p);
        }
    }

    return cloudPtr;


}

void PclVisualizer::VisualizingPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPtr)
{

    pcl::visualization::CloudViewer viewer("PCL_Visualizer");
    viewer.showCloud(cloudPtr);
    while (!viewer.wasStopped())
    {
    }

}




