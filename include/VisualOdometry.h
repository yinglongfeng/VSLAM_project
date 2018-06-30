#include <iostream>
#include <vector>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>





class VisualOdometry{
private:
//    std::vector<unsigned char> status;
public:
    VisualOdometry() {};

    //TO DO harrisDection
    //TO DO featureTracking
    std::vector<uchar> corr2DPointsFromPreFrame2DPoints(cv::Mat previousImage, cv::Mat currImage,
                                                                std::vector<cv::Point2f>& previousFrame2DPoints,
                                                                std::vector<cv::Point2f>& currFrame2DPoints);
    //TO DO poseEstimate2D3DPnp
};