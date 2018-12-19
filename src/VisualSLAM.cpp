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
  double fx, K01, cx, K10, K11, fy, cy, K21, K22;

  file >> number >> fx >> K01 >> cx >> K10 >> K11 >> fy >> cy >> K21 >> K22;
  K << fx, 0, cx,
      0, fy, cy,
      0, 0, 1;
//    std::cout<<"camera intrinsics: "<< K << std::endl;
  return K;

}

void VisualSLAM::readGroundTruthData(std::string fileName,
                                     int numberFrames,
                                     std::vector<Sophus::SE3> &groundTruthData) {
  std::ifstream inFile;
  inFile.open(fileName, std::ifstream::in);

  if (!inFile) {
    throw std::runtime_error("readGroundTruthData() : Cannot read the file with ground truth data");
  }
  if (numberFrames <= 0) {
    throw std::runtime_error("readGroundTruthData() : Number of frames is non-positive!");
  }

  groundTruthData.clear();

  int i = 0;
  while (i < numberFrames && !inFile.eof()) {
    double rotationElements[9], translationElements[3];
    int k = 0;
    for (int j = 1; j <= 12; j++) {
      if (j%4==0) {
        inFile >> translationElements[j/4 - 1];
      } else {
        inFile >> rotationElements[k++];
      }
    }
    cv::Mat R_CV = cv::Mat(3, 3, CV_64F, rotationElements);
    Eigen::Matrix3d R_Eigen;
    cv::cv2eigen(R_CV, R_Eigen);
    Sophus::SE3
        newPose = Sophus::SE3(Eigen::Quaterniond(R_Eigen), Eigen::Vector3d(translationElements));
    groundTruthData.push_back(newPose);
    i++;
  }
}

//TODO poseEstimate3D2DFrontEndWithOpicalFlow()  return pose
//TODO harrisDection
//TODO featureTracking
//TODO poseEstimate2D3DPnp
//TODO reInitialize
Sophus::SE3 VisualSLAM::estimate3D2DFrontEndWithOpicalFlow(cv::Mat leftImage_,
                                                           cv::Mat rightImage,
                                                           std::vector<cv::Point2f> &previousFrame2DPoints,
                                                           std::vector<cv::Point2f> &currFrame2DPoints,
                                                           cv::Mat &previousImage,
                                                           Sophus::SE3 prePose) {
  cv::Mat leftImage = leftImage_;
  Sophus::SE3 pose;
  int maxCorners = 500;
  cv::Size subPixel(10, 10);
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);

  if (previousFrame2DPoints.empty()) {
    std::cout << "The first image " << std::endl;
    //TO DO Harris Detection

    cv::goodFeaturesToTrack(leftImage,
                            currFrame2DPoints,
                            maxCorners,
                            0.01,
                            10,
                            cv::Mat(),
                            3,
                            3,
                            false,
                            0.04);
    cv::cornerSubPix(leftImage, currFrame2DPoints, subPixel, cv::Size(-1, -1), termcrit);
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
    VO.generateDisparityMap(leftImage, rightImage);
    p3d = VO.getDepth3DPointsFromCurrImage(currFrame2DPoints, K);
    int maxDistance = 150;
    for (int i = 0; i < p3d.size(); ++i) {
      if (p3d[i].z > maxDistance) {
//                std::cout<<"depth "<<p3d[i]<<std::endl;
        p3d.erase(p3d.begin() + i);
        currFrame2DPoints.erase(currFrame2DPoints.begin() + i);
      } else if (std::isnan(p3d[i].z)) {
        p3d.erase(p3d.begin() + i);
        currFrame2DPoints.erase(currFrame2DPoints.begin() + i);
      }
    }
//        std::cout<<"p3d size "<<p3d.size()<<"  currFrame2DPoints size "<<currFrame2DPoints.size()<<std::endl;

    //TO DO add3DPoints (p3d, p2dToP3dIndices)
    //TO DO add2DPoints (p2d, p2dToP3dIndices)
    std::vector<int> p2dToP3dIndices(p3d.size());
    std::iota(p2dToP3dIndices.begin(), p2dToP3dIndices.end(), 0);
    /*
    for (int j = 0; j <p2dToP3dIndices.size() ; ++j) {
        std::cout<<"p2dToP3dIndices[] "<<p2dToP3dIndices[j]<<std::endl;
    }
     */

    map.add2DPoints(currFrame2DPoints, p2dToP3dIndices, true);
    map.updateCumPose(pose);
    map.add3DPoints(p3d, p2dToP3dIndices);
    map.updatePoseIndex();

    leftImage.copyTo(previousImage);
    previousFrame2DPoints.clear();
    previousFrame2DPoints = currFrame2DPoints;
//        currFrame2DPoints.clear();
    return pose;
  }

  //TO DO featureTracking
  std::vector<uchar> status;
  std::vector<int> p2dToP3dIndices;
  status = VO.corr2DPointsFromPreFrame2DPoints(previousImage,
                                               leftImage,
                                               previousFrame2DPoints,
                                               currFrame2DPoints);
  std::vector<cv::Point2f> trackedCurrFrame2DPoints, trackedPreviousFrame2DPoints;
  for (int i = 0; i < status.size(); ++i) {
    if (status[i]==1) {
      //TO DO delete 3D points in the previousFrame
      trackedPreviousFrame2DPoints.push_back(previousFrame2DPoints[i]);
      trackedCurrFrame2DPoints.push_back(currFrame2DPoints[i]);
      p2dToP3dIndices.push_back(i);
    }
  }

  map.add2DPoints(trackedCurrFrame2DPoints, p2dToP3dIndices, false);
  //TO DO getDisparityMapFromPreviousImage
  //TO DO getDepth3DPointsFromPreviousImage
  std::vector<cv::Point3f> p3DCurrFrame;
  VO.generateDisparityMap(leftImage, rightImage);
  p3DCurrFrame = VO.getDepth3DPointsFromCurrImage(trackedCurrFrame2DPoints, K);

//    std::cout<<"previousFrame2DPoints size "<<previousFrame2DPoints.size() << std::endl;
//    std::cout<<"currFrame2DPoints size "<<currFrame2DPoints.size() << std::endl;
//    std::cout<<"trackedCurrFrame2DPoints size "<<trackedCurrFrame2DPoints.size() << std::endl;
//    std::cout<<"p2d size "<<trackedPreviousFrame2DPoints.size() << std::endl;
//    std::cout<<"p3d size  "<<p3DCurrFrame.size()<<std::endl;

  //TO DO poseEstimate3D2DPnp
  pose = VO.poseEstimate2D3DPNP(p3DCurrFrame, trackedPreviousFrame2DPoints, K, prePose);

  //TO DO reInitial
  if (VO.getReInitial()) {
    //TO DO Harris Detection
    std::cout << "re-initial " << std::endl;
    currFrame2DPoints.clear();
    cv::goodFeaturesToTrack(leftImage,
                            currFrame2DPoints,
                            maxCorners,
                            0.01,
                            10,
                            cv::Mat(),
                            3,
                            3,
                            false,
                            0.04);
    cv::cornerSubPix(leftImage, currFrame2DPoints, subPixel, cv::Size(-1, -1), termcrit);
//        std::cout << "step1 check feature size " << currFrame2DPoints.size() << std::endl;

    //TO DO getDisparityMapFromCurrImage
    std::vector<cv::Point3f> p3d;
    VO.generateDisparityMap(leftImage, rightImage);
    p3d = VO.getDepth3DPointsFromCurrImage(currFrame2DPoints, K);
    int maxDistance = 150;
    for (int i = 0; i < p3d.size(); ++i) {
      if (p3d[i].z > maxDistance) {
//                std::cout<<"depth "<<p3d[i]<<std::endl;
        p3d.erase(p3d.begin() + i);
        currFrame2DPoints.erase(currFrame2DPoints.begin() + i);
      } else if (std::isnan(p3d[i].z)) {
        p3d.erase(p3d.begin() + i);
        currFrame2DPoints.erase(currFrame2DPoints.begin() + i);
      }
    }
    trackedPreviousFrame2DPoints = currFrame2DPoints;
//        std::cout<<" re-initial p3d size "<<p3d.size()<<"  re-initial currFrame2DPoints size"<<currFrame2DPoints.size()<<std::endl;

    std::vector<int> p2dToP3dIndices(p3d.size());
    std::iota(p2dToP3dIndices.begin(), p2dToP3dIndices.end(), 0);
    map.add2DPoints(currFrame2DPoints, p2dToP3dIndices, true);
//        map.updatePoseIndex();
//        map.updateCumPose(pose);
    map.add3DPoints(p3d, p2dToP3dIndices);

    VO.setReInitial();
//        return pose;
  }

//    // optimization 0-->400
//    int keyFrameStep = 1 ;//3;
//    int InitialnumKeyFrames =30 ;//10
////    if (map.getCurrFrameIndex() % (InitialnumKeyFrames*keyFrameStep) == 0 && map.getCurrFrameIndex() > 0 && map.getCurrFrameIndex() <InitialnumKeyFrames*keyFrameStep +1)
//        if (map.getCurrFrameIndex() % (InitialnumKeyFrames*keyFrameStep) == 0 && map.getCurrFrameIndex() > 0 ){
//        std::cout<<"before ba  " <<std::endl;
//        map.printCumPose();
//        std::vector<Sophus::SE3> newPoseVector;
//        newPoseVector = BA.optimizeLocalPoseBA_ceres(map, keyFrameStep, InitialnumKeyFrames);
//
////        int startFrame =  map.getCurrFrameIndex() - keyFrameStep*InitialnumKeyFrames;
////        int newPoseVectorIndex = startFrame;
////        for (int i = startFrame; i < map.getCurrFrameIndex(); i+=keyFrameStep) {
////            map.setCumPose(i,newPoseVector[newPoseVectorIndex]);
////            newPoseVectorIndex ++ ;
////        }
//
//        std::cout<<"after ba  " <<std::endl;
//        map.printCumPose();
//    }

  map.updateCumPose(pose);
  map.updatePoseIndex();
  previousFrame2DPoints.clear();
  previousFrame2DPoints = currFrame2DPoints;
  leftImage.copyTo(previousImage);

  return pose;
}

Sophus::SE3 VisualSLAM::getPose(int k) {
  if (k < 0 || k > map.getCumPose().size()) {
    throw std::runtime_error("VisualSLAM::getPose(int k): Index out of the bounds");
  }
  return map.getCumPose().at(k);
}

// visualization
void VisualSLAM::plotTrajectoryNextStep(cv::Mat &window,
                                        int index,
                                        Eigen::Vector3d &translGTAccumulated,
                                        Eigen::Vector3d &translEstimAccumulated,
                                        Sophus::SE3 groundTruthPose,
                                        Sophus::SE3 groundTruthPrevPose,
                                        Eigen::Matrix3d &cumR,
                                        Sophus::SE3 estimPose,
                                        Sophus::SE3 estimPrevPose) {
  int offsetX = 200;
  int offsetY = 500;
  Sophus::SE3 pose = estimPose.inverse();
  Sophus::SE3 prevPose = estimPrevPose.inverse();

  if (index==0) {
    translGTAccumulated = groundTruthPose.translation();
    translEstimAccumulated = pose.translation();
  } else {
    translGTAccumulated = translGTAccumulated
        + (groundTruthPose.so3().inverse()*groundTruthPrevPose.so3())
            *(groundTruthPose.translation() - groundTruthPrevPose.translation());
    translEstimAccumulated = translEstimAccumulated
        + (pose.so3().inverse()*prevPose.so3())*(pose.translation() - prevPose.translation());
  }
  cv::circle(window,
             cv::Point2d(offsetX + translGTAccumulated[0], offsetY - translGTAccumulated[2]),
             3,
             cv::Scalar(0, 0, 255),
             -1);
  cv::circle(window,
             cv::Point2f(offsetX + translEstimAccumulated[0], offsetY - translEstimAccumulated[2]),
             3,
             cv::Scalar(0, 255, 0),
             -1);
  cv::imshow("Trajectory", window);
  cv::waitKey(3);
  cumR = cumR*pose.so3().matrix();

}