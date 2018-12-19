
#include "Map.h"

Map::Map() {
  test_a = 666;
  poseIndex = 0;
  ReInitial3DPointsIndexOffset = 0;
}

int Map::getValue() {
  return test_a;
}

void Map::updateCumPose(Sophus::SE3 newPose) {

  if (cumPose.empty()) {
    cumPose.push_back(newPose);
    return;
  }

  assert(poseIndex==cumPose.size());

  cumPose.push_back(newPose.inverse()*cumPose[poseIndex - 1]);

}

void Map::updatePoseIndex() {

  poseIndex = poseIndex + 1;
}

std::vector<Sophus::SE3> Map::getCumPose() const {
  return cumPose;
}

void Map::add3DPoints(std::vector<cv::Point3f> &p3d, std::vector<int> &p2dToP3dIndices) {

  if (p3d.empty()) {
    throw std::runtime_error("add3DPoints p3d is empty ");
  }
  Sophus::SE3 currPose = cumPose[poseIndex];
  assert(p3d.size()==p2dToP3dIndices.size());
  for (int i = 0; i < p3d.size(); ++i) {
    Eigen::Vector3d Pworld_pose(p3d[i].x, p3d[i].y, p3d[i].z);
    Eigen::Vector3d Pworld = currPose.inverse()*Pworld_pose;
    Points3D.push_back(cv::Point3f(Pworld[0], Pworld[1], Pworld[2]));
  }
//    std::cout<<"Points3D size "<<Points3D.size()<<std::endl;

}

void Map::add2DPoints(std::vector<cv::Point2f> &p2d,
                      std::vector<int> &p2dToP3dIndices,
                      bool reIntial) {

  assert(p2d.size()==p2dToP3dIndices.size());
  if (reIntial) {
    ReInitial3DPointsIndexOffset = Points3D.size();
  }

  std::vector<std::pair<int, cv::Point2f>> observations;
  for (int i = 0; i < p2dToP3dIndices.size(); ++i) {
    observations
        .push_back(std::make_pair(ReInitial3DPointsIndexOffset + p2dToP3dIndices[i], p2d[i]));
  }
  p3dIndexAndP2dPoints[poseIndex] = observations;
//    std::cout<<"p3dIndexAndP2dPoints.size(): " <<p3dIndexAndP2dPoints[poseIndex].size() <<std::endl;
}

std::map<int, std::vector<std::pair<int, cv::Point2f >>> Map::get2DPoints() const {
  return p3dIndexAndP2dPoints;
}

std::vector<cv::Point3f> Map::get3DPoints() const {
  return Points3D;
}
int Map::getCurrFrameIndex() const {
  return poseIndex;
}

void Map::setCumPose(const int poseIndex, const Sophus::SE3 newPose) {

  if (poseIndex < 0 || poseIndex > cumPose.size()) {
    throw std::runtime_error("setCumPose(): poseIndex out of the bound");
  }

  cumPose[poseIndex] = newPose;

//    std::cout<<"poseIndex:"<< poseIndex <<std::endl;
}

//
//void Map::update3DPoints(int pointsNum, double position[][3]) {
//
////    if (position_num < 0 || position_num > Points3D.size()){
////        throw std::runtime_error("position_num: out of bound");
////    }
//    for (int i = 0; i < pointsNum ; ++i) {
//        Eigen::Vector3d newP3d;
//        newP3d[0] = position[i][0];
//        newP3d[1] = position[i][1];
//        newP3d[2] = position[i][2];
//        cv::Point3f point (newP3d[0], newP3d[1], newP3d[2]);
//        Points3D[i] = point;
//    }
//}
void Map::update3DPoints(std::set<int> uniquePointIndices, double position[][3]) {

//    if (position_num < 0 || position_num > Points3D.size()){
//        throw std::runtime_error("position_num: out of bound");
//    }
  for (int i = 0; i < uniquePointIndices.size(); ++i) {
    Eigen::Vector3d newP3d;
    newP3d[0] = position[i][0];
    newP3d[1] = position[i][1];
    newP3d[2] = position[i][2];
    cv::Point3f point(newP3d[0], newP3d[1], newP3d[2]);
    Points3D[i] = point;
  }
}

void Map::printCumPose() {

  for (int i = 0; i < cumPose.size(); i++) {
    std::cout << "cumPose:  " << i << std::endl << cumPose[i].matrix() << std::endl;
  }

}