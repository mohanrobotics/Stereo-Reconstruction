# Stereo-Reconstruction

Stereo 3D Reconstruction using C++ and QT creator.

Most of core algorithm code was based on [Camera Calibration and 3D Reconstruction](https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html?highlight=findcirclesgrid) and [Stereo Calibration](https://jayrambhia.com/blog/stereo-calibration)

![Flowchart](/extras/3d_Reconstruction.gif)

Please open the Reconstruction.pro files and edit the path to the OpenCV library and PCL library, which should be already installed on your computer.

## Requirements

* PCL
* Open-cv 

## Project Tree
```
├── include
│   └── Reconstruction
│       ├── cameracalibration.h
│       ├── depthestimation.h
│       ├── pclvisualizer.h
│       └── reconstruction.h
└── src
│   ├──  cameracalibration.cpp
│   ├──  depthestimation.cpp
│   ├──  pclvisualizer.cpp
│   ├── reconstruction.cpp
├── Reconstruction.pro
├── Reconstruction.pro.user
├── data
│   ├── inputs
│   │   ├── calibration_images
│   │   │   ├── <chessboard_images>.png
│   │   │   └── README.md
│   │   └── stereo_images
│   │       ├── <stereo_images>.png
│   │       └── README.md
│   └── outputs
│       ├── camera_parameters
│       │   ├── extrinsics.yml
│       │   ├── intrinsics.yml
│       │   └── README.md
│       ├── depth_images
│       │   ├── <Depth image outputs>.png
│       │   └── README.md
│       ├── disparity_images
│       │   ├── <disparity image outputs>.png
│       │   └── README.md
│       ├── disparity_normalized
│       │   ├── <normalized disparity image outputs>.png  
│       │   └── README.md
│       └── point_clouds
│           ├── <point clouds>.ply
│           └── README.md
├── extras
│   └── 3d_Reconstruction.gif
├── README.md
```

