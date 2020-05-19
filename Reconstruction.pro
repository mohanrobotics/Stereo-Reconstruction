QT -= gui

TEMPLATE = app
DEFINES += RECONSTRUCTION_LIBRARY

CONFIG += c++11

DEFINES += QT_DEPRECATED_WARNINGS

DEFINES += PROJECT_SOURCE_DIR=\\\"$$absolute_path(".")\\\"

INCLUDEPATH += include
INCLUDEPATH += /usr/local/include/opencv2 /usr/include/pcl-1.8/ /usr/include/eigen3/ /usr/include/vtk-6.3/
LIBS += -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lpthread -lopencv_imgproc -lopencv_ximgproc -lopencv_calib3d -lpcl_visualization -lboost_system -lpcl_io -lpcl_common


SOURCES += \
    src/cameracalibration.cpp \
    src/depthestimation.cpp \
    src/pclvisualizer.cpp \
    src/reconstruction.cpp

HEADERS += \
    include/Reconstruction/cameracalibration.h \
    include/Reconstruction/depthestimation.h \
    include/Reconstruction/pclvisualizer.h

# Default rules for deployment.
unix {
    target.path = /usr/lib
}
!isEmpty(target.path): INSTALLS += target
