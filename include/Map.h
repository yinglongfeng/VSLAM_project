
#pragma once
#include <sophus/se3.h>
#include <vector>
#include <map>
#include <set>
#include <iostream>
// opencv
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

class Map {
 private:
  int test_a;
  std::vector<Sophus::SE3> cumPose;
  int poseIndex;
  int ReInitial3DPointsIndexOffset;
  std::vector<cv::Point3f> Points3D;
  std::map<int, std::vector<std::pair<int, cv::Point2f >>> p3dIndexAndP2dPoints;
  int offsetIndex;

 public:
  Map();
  int getValue();

  void updateCumPose(Sophus::SE3 newPose);
  void updatePoseIndex();

  std::vector<Sophus::SE3> getCumPose() const;

  //TODO add3DPoints.
  void add3DPoints(std::vector<cv::Point3f> &p3d, std::vector<int> &p2dToP3dIndices);

  //TODO add2DPoints.
  void add2DPoints(std::vector<cv::Point2f> &p2d, std::vector<int> &p2dToP3dIndices, bool reIntial);

  std::map<int, std::vector<std::pair<int, cv::Point2f >>> get2DPoints() const;
  std::vector<cv::Point3f> get3DPoints() const;
  int getCurrFrameIndex() const;

//    void update3DPoints(int pointsNum, double position[][3]);
  void update3DPoints(std::set<int> uniquePointIndices, double position[][3]);
  void setCumPose(const int poseIndex, const Sophus::SE3 newPose);

  void printCumPose();

};