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
        std::cout<<"The first image "<<std::endl;
        //TO DO Harris Detection

        cv::goodFeaturesToTrack(leftImage,currFrame2DPoints,maxCorners,0.01,10,cv::Mat(),3,3,false,0.04);
        cv::cornerSubPix(leftImage,currFrame2DPoints,subPixel,cv::Size(-1,-1),termcrit);
        std::cout << "step1 check feature size " << currFrame2DPoints.size() << std::endl;


        /*
        // test corners detection results
        std::cout<<"** Number of corners detected: "<<currFrame2DPoints.size()<<std::endl;
        for (size_t i=0; i<currFrame2DPoints.size(); i++)
        cv::circle(leftImage, currFrame2DPoints[i], 4, cv::Scalar(200,0,0));
        cv::namedWindow("leftImage",CV_WINDOW_AUTOSIZE);
        cv::imshow("leftImage",leftImage);
        cv::waitKey();
         */
        //TO DO getDisparityMapFromCurrImage
        // Define the first image as the world coordinate
        std::vector<cv::Point3f> p3d;
        VO.generateDisparityMap(leftImage,rightImage);
        p3d = VO.getDepth3DPointsFromCurrImage(currFrame2DPoints,K);
        int maxDistance = 150 ;
        for (int i = 0; i <p3d.size() ; ++i) {
            if (p3d[i].z > maxDistance ){
//                std::cout<<"depth "<<p3d[i]<<std::endl;
                p3d.erase(p3d.begin()+i);
                currFrame2DPoints.erase(currFrame2DPoints.begin()+i);
            }
//            else if (std::isnan(p3d[i].z)){
//                p3d.erase(p3d.begin()+i);
//                currFrame2DPoints.erase(currFrame2DPoints.begin()+i);
//            }
        }
        std::cout<<"p3d size "<<p3d.size()<<"  currFrame2DPoints size"<<currFrame2DPoints.size()<<std::endl;
        std::cout << "step 2 check feature size after erase " << currFrame2DPoints.size() << std::endl;

        leftImage.copyTo(previousImage);
        previousFrame2DPoints.clear();
        previousFrame2DPoints=currFrame2DPoints;
        currFrame2DPoints.clear();
        return pose;
    }

    //TO DO featureTracking
    std::vector<uchar> status;
    status = VO.corr2DPointsFromPreFrame2DPoints(previousImage,leftImage,previousFrame2DPoints,currFrame2DPoints);
    std::vector<cv::Point2f> trackedCurrFrame2DPoints , trackedPreviousFrame2DPoints;
    for (int i = 0; i <status.size() ; ++i) {
        if ( status[i] == 1) {
            //TO DO delete 3D points in the previousFrame
            trackedPreviousFrame2DPoints.push_back(previousFrame2DPoints[i]);
            trackedCurrFrame2DPoints.push_back(currFrame2DPoints[i]);

        }
    }

    //TO DO getDisparityMapFromPreviousImage
    //TO DO getDepth3DPointsFromPreviousImage
    std::vector<cv::Point3f> p3DCurrFrame;
    VO.generateDisparityMap(leftImage,rightImage);
    p3DCurrFrame = VO.getDepth3DPointsFromCurrImage(trackedCurrFrame2DPoints,K);

    std::cout<<"previousFrame2DPoints size "<<previousFrame2DPoints.size() << std::endl;
    std::cout<<"currFrame2DPoints size "<<currFrame2DPoints.size() << std::endl;
    std::cout<<"trackedCurrFrame2DPoints size "<<trackedCurrFrame2DPoints.size() << std::endl;
    std::cout<<"p2d size "<<trackedPreviousFrame2DPoints.size() << std::endl;
    std::cout<<"p3d size  "<<p3DCurrFrame.size()<<std::endl;

    //TO DO poseEstimate3D2DPnp
    pose=VO.poseEstimate2D3DPNP(p3DCurrFrame,trackedPreviousFrame2DPoints,K);

    previousFrame2DPoints.clear();
    previousFrame2DPoints=trackedCurrFrame2DPoints;
    currFrame2DPoints.clear();

    return pose;
}