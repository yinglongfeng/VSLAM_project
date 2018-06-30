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
Sophus::SE3 VisualSLAM::estimate3D2DFrontEndWithOpicalFlow(cv::Mat leftImage_, cv::Mat rightImage,
                                                           std::vector<cv::Point2f> &previousFrame2DPoints,
                                                           std::vector<cv::Point2f> &currFrame2DPoints,
                                                           cv::Mat& previousImage) {
    cv::Mat leftImage = leftImage_;
    Sophus::SE3 pose;
    int maxCorners=500;
    cv::Size subPixel(10,10);
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);

    if( previousFrame2DPoints.empty()){
        std::cout<<" previousFram2DPoints size "<<previousFrame2DPoints.size() << std::endl;
        //TO DO Harris Detection

        cv::goodFeaturesToTrack(leftImage,currFrame2DPoints,maxCorners,0.01,10,cv::Mat(),3,3,false,0.01);
        cv::cornerSubPix(leftImage,currFrame2DPoints,subPixel,cv::Size(-1,-1),termcrit);

        // test corners detection results
    /*std::cout<<"** Number of corners detected: "<<currFrame2DPoints.size()<<std::endl;
    for (size_t i=0; i<currFrame2DPoints.size(); i++)
    cv::circle(leftImage, currFrame2DPoints[i], 4, cv::Scalar(200,0,0));
    cv::namedWindow("leftImage",CV_WINDOW_AUTOSIZE);
    cv::imshow("leftImage",leftImage);
    cv::waitKey(); */

        //TO DO getDisparityMapFromCurrImage
        
        //TO DO getDepth3DPointsFromCurrImage


        leftImage.copyTo(previousImage);
        previousFrame2DPoints.clear();
        previousFrame2DPoints=currFrame2DPoints;
        currFrame2DPoints.clear();
        std::cout<<" Just for the first image "<<std::endl;
        std::cout<<" previousFram2DPoints size "<<previousFrame2DPoints.size() << std::endl;
        return pose;
    }

    //TO DO featureTracking
    std::vector<uchar> status;
    status = VO.corr2DPointsFromPreFrame2DPoints(previousImage,leftImage,previousFrame2DPoints,currFrame2DPoints);
    std::vector<cv::Point2f> trackedCurrFrame2DPoints;
    for (int i = 0; i <status.size() ; ++i) {
        if ( status[i] == 1) {
            //TO DO delete 3D points in the previousFrame erase

            trackedCurrFrame2DPoints.push_back(currFrame2DPoints[i]);
        }
    }
    std::cout<<" currFrame2DPoints size "<<currFrame2DPoints.size() << std::endl;
    std::cout<<" trackedCurrFrame2DPoints size "<<trackedCurrFrame2DPoints.size() << std::endl;

    //TO DO getDisparityMapFromPreviousImage
    //TO DO getDepth3DPointsFromPreviousImage

    //TO DO poseEstimate3D2DPnp



    previousFrame2DPoints.clear();
    previousFrame2DPoints=trackedCurrFrame2DPoints;
    currFrame2DPoints.clear();

    Eigen::Matrix3d tmpMatrix;
    tmpMatrix << 1 , 0.2 , 0.3 , 4 , 5 , 6 , 7 , 8 , 9 ;
    pose.setRotationMatrix(tmpMatrix);
    return pose;


}