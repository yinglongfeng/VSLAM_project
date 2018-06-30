#include "VisualSLAM.h"

VisualSLAM::VisualSLAM() {}

int VisualSLAM::getTestValueFromMap() {
    return map.getValue();
}

//TO DO get camera intrinsics
Eigen::Matrix3d VisualSLAM::getCameraIntrinsics(std::string camera_intrinsics_path) {
//    std::cout<<"camera_intrinsics_path is: "<<camera_intrinsics_path<<std::endl;
    std::ifstream file(camera_intrinsics_path);
    std::string number;
    double fx,K01,cx,K10,K11,fy,cy,K21,K22;

    file >>number>>fx>>K01>>cx>>K10>>K11>>fy>>cy>>K21>>K22;
    K << fx , 0 , cx ,
         0  , fy ,cy ,
         0 ,  0 , 1 ;
//    std::cout<<"camera intrinsics: "<< K << std::endl;
    return K;
    
}


//TO DO poseEstimate3D2DFrontEndWithOpicalFlow()  return pose
    //TO DO harrisDection
    //TO DO featureTracking
    //TO DO poseEstimate2D3DPnp
    //TO DO reInitialize
Sophus::SE3 VisualSLAM::estimate3D2DFrontEndWithOpicalFlow(cv::Mat leftImage, cv::Mat rightImage,
                                                           std::vector<cv::Point2f> &previousFrame2DPoints,
                                                           std::vector<cv::Point2f> &currFrame2DPoints,
                                                           cv::Mat &peviousImage) {
    Sophus::SE3 pose;
    int maxCorners=500;
    cv::Size subPixel(10,10);
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);

//    if(previousFrame2DPoints.empty()){
        //TO DO harrisDection

        cv::goodFeaturesToTrack(leftImage,currFrame2DPoints,maxCorners,0.01,10,cv::Mat(),3,3,false,0.01);

        // test corners dectection results
    /*std::cout<<"** Number of corners detected: "<<currFrame2DPoints.size()<<std::endl;
    for (size_t i=0; i<currFrame2DPoints.size(); i++)
        cv::circle(leftImage, currFrame2DPoints[i], 4, cv::Scalar(200,0,0));
    cv::namedWindow("leftImage",CV_WINDOW_AUTOSIZE);
    cv::imshow("leftImage",leftImage);
    cv::waitKey(); */


        cv::cornerSubPix(leftImage,currFrame2DPoints,subPixel,cv::Size(-1,-1),termcrit);
//        previousFrame2DPoints.clear();
//        previousFrame2DPoints=currFrame2DPoints;

        return pose;
//    }
    //TO DO featureTracking
    //TO DO poseEstimate2D3DPnp


}