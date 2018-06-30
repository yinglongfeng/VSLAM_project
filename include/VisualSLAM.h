#include <iostream>
#include <fstream>
// project file
#include "BundleAdjuster.h"
#include "Map.h"
#include "VisualOdometry.h"
// eigen file
#include <Eigen/Core>
#include <Eigen/Geometry>
// opencv file
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "opencv2/imgproc/imgproc.hpp"
// Sophus
#include <sophus/se3.h>


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
    Sophus::SE3 estimate3D2DFrontEndWithOpicalFlow(cv::Mat leftImage_, cv::Mat rightImage, std::vector<cv::Point2f>
            &previousFrame2DPoints, std::vector<cv::Point2f>&currFrame2DPoints,cv::Mat& previousImage);

};