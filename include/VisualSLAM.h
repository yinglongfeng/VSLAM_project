#include "BundleAdjuster.h"
#include "Map.h"
#include "VisualOdometry.h"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>



class VisualSLAM{
private:
    BundleAdjuster BA;
    Map map;
    VisualOdometry VO;
    Eigen::Matrix3d K;


public:
    VisualSLAM();
    int getTestValueFromMap();
    //TO DO get camera intrinsics
    Eigen::Matrix3d getCameraIntrinsics(std::string camera_intrinsics_path);


    //TO DO estimate3D2DFrontEndWithOpicalFlow()
};