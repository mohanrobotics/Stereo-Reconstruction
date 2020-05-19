#ifndef CAMERACALIBRATION_H
#define CAMERACALIBRATION_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/ximgproc.hpp>
#include <vector>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>


using namespace cv;
using namespace std;


struct imageSet{
    vector<cv::String> images_list;
    vector<Mat> lower_images, upper_images;
    Size imageSize;
    int numImage;
 };

struct intrinsicCalibration {

    Mat R, T, E, F, M1, M2, D1, D2;
    double rms_error, avg_epipolar_err;
    vector< vector< Point3f > > object_points;
    vector< vector< Point2f > > imagePoints1, imagePoints2;
   };

struct CalibrationMatrix {
    Mat R1, R2, P1, P2, Q;
    Rect validRoi[2];
};


/**
 * Class name: CameraCalibration
 * Description: Contains the functionalities for camera calibration
*/
class CameraCalibration
{

public:

    static const Size boardSize;
    static const float squareSize;

    CameraCalibration() {}
    virtual ~CameraCalibration() {}

    static imageSet readCheckerboardImages(const string &path); // Image loading function

    static intrinsicCalibration findCheckerboardCorners(const imageSet  &image_set); //Obtaining Image Points and object points

    static void CalibrationQualityCheck(const intrinsicCalibration &iC,
                                        const imageSet  &image_set); //Camera calibration quality check

    static CalibrationMatrix stereoCalibration(const intrinsicCalibration &iC,
                                               const imageSet &image_set); //Stereo calibration

    static void write_data(const intrinsicCalibration &iC,
                           const CalibrationMatrix &sC,
                           const string &param_path); //Saving the camera parameters

};

#endif // CAMERACALIBRATION_H
