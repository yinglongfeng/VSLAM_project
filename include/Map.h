#include <sophus/se3.h>
#include <vector>
#include <map>
#include <iostream>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>



class Map{
private:
    int test_a;
    std::vector<Sophus::SE3> cumPose;
    int poseIndex;
    int ReInitial3DPointsIndexOffset;
    std::vector<cv::Point3f> Points3D;
    std::map <int,std::vector<std::pair<int , cv::Point2f >>> p3dIndexAndP2dPoints;

public:
    Map();
    int getValue();

    void updateCumPose(Sophus::SE3 newPose);
    void updatePoseIndex();

    std::vector<Sophus::SE3> getCumPose();

    //TO DO add3DPoints
    void add3DPoints(std::vector<cv::Point3f>& p3d, std::vector<int>& p2dToP3dIndices);

    //TO DO add2DPoints
    void add2DPoints(std::vector<cv::Point2f>& p2d, std::vector<int > &p2dToP3dIndices , bool reIntial);
};