
#include "VisualOdometry.h"




//TO DO harrisDection
//TO DO featureTracking
std::vector<uchar> VisualOdometry::corr2DPointsFromPreFrame2DPoints(cv::Mat previousImage, cv::Mat currImage,
                                                                            std::vector<cv::Point2f> &previousFrame2DPoints_,
                                                                            std::vector<cv::Point2f> &currFrame2DPoints) {
    // Parameters for lucas kanade optical flow
//    std::vector<cv::Point2f> currFrame2DPoints= previousFrame2DPoints_;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::Size winSize = cv::Size(21, 21);
    int maxLevel = 3;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);

    cv::calcOpticalFlowPyrLK(previousImage, currImage, previousFrame2DPoints_,currFrame2DPoints,status,err,winSize,maxLevel,termcrit);
//    std::cout<< "currFrame2DPoints size : "<< currFrame2DPoints.size()<<std::endl;
    // trackedCurrFrame2DPoints

//    std::cout<<" trackedCurrFrame2DPoints size "<<trackedCurrFrame2DPoints.size() << std::endl;

    return status;
}
//TO DO poseEstimate2D3DPnp