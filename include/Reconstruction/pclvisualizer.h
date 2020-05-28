#ifndef PCLVISUALIZER_H
#define PCLVISUALIZER_H

#include "cameracalibration.h"
#include "depthestimation.h"

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>

/**
 * Class name: PclVisualizer
 * Description: Contains the functionalities for point cloud generation and viewing
*/


class PclVisualizer
{
public:
    PclVisualizer(){}
    virtual ~PclVisualizer() {}

    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr CreatingPointCloud(
            const Mat &img, const Mat &left_rect_1,
            const intrinsicCalibration &iC); // Creation of point cloud

    static void VisualizingPointCloud(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloudPtr); //Viewing the point cloud

};

#endif // PCLVISUALIZER_H
