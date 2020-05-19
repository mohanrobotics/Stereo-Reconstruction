#include "Reconstruction/cameracalibration.h"
#include <iostream>
#include <fstream>


using namespace cv::ximgproc;
const Size  CameraCalibration::boardSize = Size(9,12);
const float  CameraCalibration::squareSize = 10.0;

imageSet CameraCalibration::readCheckerboardImages(const string &path )
{
     imageSet image_set;

     glob(path, image_set.images_list, false);

    for (size_t i=0; i<image_set.images_list.size(); i++)
    {
        if (image_set.images_list[i].find("Lower") != string::npos)
        {
             image_set.lower_images.push_back(imread(image_set.images_list[i],CV_LOAD_IMAGE_COLOR));
        }
        else if (image_set.images_list[i].find("Upper") != string::npos)
        {
            image_set.upper_images.push_back(imread(image_set.images_list[i],CV_LOAD_IMAGE_COLOR));
        }
    }

    image_set.imageSize = image_set.upper_images[0].size();
    image_set.numImage =  image_set.upper_images.size();
    return image_set;

}


intrinsicCalibration CameraCalibration::findCheckerboardCorners(const imageSet &image_set)

{

    vector<Mat> l_images = image_set.lower_images;
    vector<Mat> u_images = image_set.upper_images;

    // Declaring variables for Calibration
    vector< vector< Point3f > > object_points;
    vector< vector< Point2f > > imagePoints1, imagePoints2;
    vector< Point2f > corners1, corners2;
    vector< vector< Point2f > > left_img_points, right_img_points;
    Mat img1, img2, gray1, gray2;

    int board_width = 9;   // number of horizontal corners
    int board_height = 12;   // number of vertical corners
    float square_size = 10.0;

    Size board_size = Size(board_width, board_height);

    // Obtaining the image points and object points
    for (int i = 0; i <  image_set.numImage; i++)
    {
        img1 = l_images[i];
        img2 = u_images[i];
        cvtColor(img1, gray1, CV_BGR2GRAY);
        cvtColor(img2, gray2, CV_BGR2GRAY);
        bool found1 = false, found2 = false;
        found1 = cv::findChessboardCorners(img1, board_size, corners1,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        found2 = cv::findChessboardCorners(img2, board_size, corners2,
        CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

        if(!found1 || !found2){
          cout << "Chessboard find error!" << endl;
          continue;
        }

        if (found1)
        {
          cv::cornerSubPix(gray1, corners1, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
          cv::drawChessboardCorners(gray1, board_size, corners1, found1);
        }

        if (found2)
        {
          cv::cornerSubPix(gray2, corners2, cv::Size(5, 5), cv::Size(-1, -1),
      cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
          cv::drawChessboardCorners(gray2, board_size, corners2, found2);
        }

        vector< Point3f > obj;
         for (int i = 0; i < board_height; i++)
           for (int j = 0; j < board_width; j++)
             obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

         if (found1 && found2) {
//           cout << i << ". Found corners!" << endl;
           imagePoints1.push_back(corners1);
           imagePoints2.push_back(corners2);
           object_points.push_back(obj);
         }
    }


    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(object_points,imagePoints1,img1.size(),0);
    cameraMatrix[1] = initCameraMatrix2D(object_points,imagePoints2,img1.size(),0);

    Mat M1, M2, D1, D2;
    M1 = initCameraMatrix2D(object_points,imagePoints1,img1.size(),0);
    M2 = initCameraMatrix2D(object_points,imagePoints2,img1.size(),0);

    Mat R, T, E, F ;
    double rms_error = stereoCalibrate(object_points, imagePoints1, imagePoints2,
                                   M1, D1, M2, D2, img1.size(),
                                   R, T, E, F,
                                   CALIB_FIX_ASPECT_RATIO + CALIB_ZERO_TANGENT_DIST +
                                   CALIB_USE_INTRINSIC_GUESS + CALIB_SAME_FOCAL_LENGTH +
                                   CALIB_RATIONAL_MODEL + CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
                                   TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 100, 1e-5) );
    cout << "RMS error=" << rms_error << endl;
    intrinsicCalibration iC;

    iC.imagePoints1 = imagePoints1;
    iC.imagePoints2 = imagePoints2;
    iC.object_points = object_points;
    iC.rms_error = rms_error;
    iC.M1 = M1;
    iC.M2 = M2;
    iC.D1 = D1;
    iC.D2 = D2;
    iC.R = R;
    iC.T = T;
    iC.E = E;
    iC.F = F;

    return iC;

}


void CameraCalibration::CalibrationQualityCheck(const intrinsicCalibration &iC, const imageSet &image_set)
{

            double err = 0;
            int npoints = 0;
            vector<Vec3f> lines[2];

            Mat cameraMatrix[2], distCoeffs[2];
            cameraMatrix[0] = iC.M1;
            cameraMatrix[1] = iC.M2;
            distCoeffs[0] = iC.D1;
            distCoeffs[1] = iC.D2;


            for( int i = 0; i < image_set.numImage; i++ )
            {

                int npt = (int)iC.imagePoints1[i].size();
                Mat imgpt[2];
                double errij;
                for( int k = 0; k < 2; k++ )
                {
                    if (k == 0)
                    {
                        imgpt[k] = Mat(iC.imagePoints1[i]);
                    }
                    else
                    {
                        imgpt[k] = Mat(iC.imagePoints2[i]);
                    }

                    cv::undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
                    cv::computeCorrespondEpilines(imgpt[k], k+1, iC.F, lines[k]);
                }

                for( int j = 0; j < npt; j++ )
                {

                    if (j == 0)
                    {
                        errij = fabs(iC.imagePoints1[i][j].x*lines[1][j][0] + iC.imagePoints1[i][j].y*lines[1][j][1] + lines[1][j][2]) +
                                fabs(iC.imagePoints1[i][j].x*lines[0][j][0] + iC.imagePoints1[i][j].y*lines[0][j][1] + lines[0][j][2]);
                        err += errij;
                    }
                    else
                    {
                        errij = fabs(iC.imagePoints2[i][j].x*lines[1][j][0] + iC.imagePoints2[i][j].y*lines[1][j][1] + lines[1][j][2]) +
                                fabs(iC.imagePoints2[i][j].x*lines[0][j][0] + iC.imagePoints2[i][j].y*lines[0][j][1] + lines[0][j][2]);
                        err += errij;
                    }


                }
                npoints += npt;
            }

        cout << "Average epipolar error = " <<  err/npoints << endl;

}

CalibrationMatrix CameraCalibration::stereoCalibration(const intrinsicCalibration &iC, const imageSet &image_set)
{
    CalibrationMatrix sC;
    stereoRectify(iC.M1, iC.D1, iC.M2, iC.D2,image_set.imageSize,iC.R,iC.T,
                     sC.R1, sC.R2, sC.P1, sC.P2, sC.Q,CALIB_ZERO_DISPARITY, 1,
                     image_set.imageSize, &sC.validRoi[0], &sC.validRoi[1]);
    return sC;


}

void CameraCalibration::write_data(const intrinsicCalibration &iC, const CalibrationMatrix &sC, const string &param_path)
{
        string intrinsics_path = param_path + "intrinsics.yml";
        string extrinsics_path = param_path + "extrinsics.yml";


        FileStorage fs(intrinsics_path, FileStorage::WRITE);
        if( fs.isOpened() )
        {
            fs << "M1" << iC.M1 << "D1" << iC.D1 << "M2" << iC.M2 << "D2" << iC.D2;
            fs.release();
        }
        else
            cout << "Error: can not save the intrinsic parameters\n";

            fs.open(extrinsics_path, FileStorage::WRITE);
            if( fs.isOpened() )
            {
                fs << "R" << iC.R << "T" << iC.T << "R1" << sC.R1 << "R2" << sC.R2 << "P1" << sC.P1 << "P2" << sC.P2 << "Q" << sC.Q;
                fs.release();
            }
            else
                cout << "Error: can not save the extrinsic parameters\n";

}









