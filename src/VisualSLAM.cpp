#include "VisualSLAM.h"

VisualSLAM::VisualSLAM() {}

int VisualSLAM::getTestValueFromMap() {
    return map.getValue();
}

//TO DO get camera intrinsics
Eigen::Matrix3d VisualSLAM::getCameraIntrinsics(std::string camera_intrinsics_path) {
    std::cout<<"camera_intrinsics_path is: "<<camera_intrinsics_path<<std::endl;
    
}


//TO DO poseEstimate3D2DFrontEndWithOpicalFlow()  return pose
    //TO DO harrisDection
    //TO DO featureTracking
    //TO DO poseEstimate2D3DPnp
    //TO DO reInitialize