#include "BundleAdjuster.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;

BundleAdjuster::BundleAdjuster() {}

//std::vector<Sophus::SE3> BundleAdjuster::optimizeLocalPoseBA_ceres(Map map,
//                                                                   int keyFrameStep,
//                                                                   int frame_num) {
//    int startFrame = 0;//= map.getCurrFrameIndex() - keyFrameStep*frame_num; // 0
//    int startFrameForOptimization = map.getCurrFrameIndex() - keyFrameStep*frame_num;
//
//    std::vector<Sophus::SE3> T_esti;
//    T_esti = map.getCumPose();
//    std::vector<cv::Point3f> p3d = map.get3DPoints();
//    std::map<int, std::vector<std::pair<int, cv::Point2f >>> p2d = map.get2DPoints();
//
//    std::set<int> points3DIndices;
//    for (int k = startFrame; k < map.getCurrFrameIndex() ; k +=keyFrameStep){
////        for (int k = 0; k < map.getCurrFrameIndex() ; k +=keyFrameStep){
//        for (int i = 0; i < p2d[k].size() ; ++i) {
//            points3DIndices.insert(p2d[k][i].first);
//        }
////        std::cout<< "p2d[k].size(): "<<p2d[k].size() <<std::endl;
//    }
//
//
//    int position_num=points3DIndices.size();//p3d.size(); //points3DIndices.size();
//    frame_num = map.getCurrFrameIndex();
//    double rotation[frame_num][4];
//    double translation[frame_num][3];
//    double position[position_num][3];
////    std::cout<< "points3DIndices.size() : "<<points3DIndices.size() <<std::endl;
//
////    std::cout<<"frame_num: "<<frame_num << " T_esti.size()" <<T_esti.size()<<std::endl;
//    std::cout<<"startFrame: "<< map.getCurrFrameIndex() - keyFrameStep*frame_num  <<std::endl;
//    std::cout<<"startFrame: "<< startFrame <<std::endl;
////    std::cout<<"position_num: "<<position_num << "p3d.size()" <<p3d.size()<<std::endl;
//    //local BA
//    ceres::Problem problem;
//    ceres::LossFunction* loss_function = new ceres::HuberLoss(1.0);
//    ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();
//    std::cout << " begin local BA " << std::endl;
////     for (int i = 0; i < frame_num; i++)
//    int numberPose = 0;
//    for (int i = startFrame; i < map.getCurrFrameIndex(); i+=keyFrameStep)
////    for (int i = 0; i < map.getCurrFrameIndex(); i+=keyFrameStep)
//    {
//        //double array for ceres
//        Eigen::Matrix3d R = T_esti[i].so3().matrix();
//        Eigen::Vector3d t = T_esti[i].translation();
//
//        Eigen::Quaterniond q_rotation(R) ;
//
////        std::cout<<"t: "<<std::endl<<t<<std::endl;
//        int cameraIndex =(i - startFrame) / keyFrameStep;// (i - 0) / keyFrameStep;//
////        std::cout<<"cameraIndex: "<<std::endl<<cameraIndex<<std::endl;
////        std::cout<<"T_esti.size(): "<<std::endl<<T_esti.size()<<std::endl;
////        std::cout<<" map.getCurrFrameIndex(): "<<std::endl<< map.getCurrFrameIndex()<<std::endl;
//
//        translation[cameraIndex][0] = t[0];
//        translation[cameraIndex][1] = t[1];
//        translation[cameraIndex][2] = t[2];
//        rotation[cameraIndex][0] = q_rotation.w();
//        rotation[cameraIndex][1] = q_rotation.x();
//        rotation[cameraIndex][2] = q_rotation.y();
//        rotation[cameraIndex][3] = q_rotation.z();
//        problem.AddParameterBlock(rotation[cameraIndex], 4, local_parameterization);
//        problem.AddParameterBlock(translation[cameraIndex], 3);
//    }
//
/////////////////* add 3 D points      */////////////////////////////////////////////
//
//    int indicesK =0;
//    for (auto indices = points3DIndices.begin();indices!= points3DIndices.end();indices ++){
//        position[indicesK][0] = p3d[*indices].x;
//        position[indicesK][1] = p3d[*indices].y;
//        position[indicesK][2] = p3d[*indices].z;
//        indicesK ++ ;
//    }
////    for (int k = 0; k <p3d.size() ; ++k) {
////        position[k][0] = p3d[k].x;
////        position[k][1] = p3d[k].y;
////        position[k][2] = p3d[k].z;
////    }
//
////    std::cout<<"indicesK :"<< p3d.size() <<std::endl;
//    std::cout<<"startFrameForOptimization: "   <<startFrameForOptimization << std::endl;
//
//    for (int i = startFrame; i < map.getCurrFrameIndex() ; i +=keyFrameStep){
////        for (int i = 0; i < map.getCurrFrameIndex() ; i +=keyFrameStep){
//        for (int j = 0; j < p2d[i].size(); j++)
//        {
//            int cameraIndex = (i - startFrame) / keyFrameStep;//i
//
//            if (cameraIndex < startFrameForOptimization ){
//                continue;
//            }
//            else {
//                ///////////////* add 2 D points      *///////////////////////////////////////
//                ceres::CostFunction* cost_function = ReprojectionError3D::Create(
//                        p2d[i][j].second.x,
//                        p2d[i][j].second.y);
//                ///////////////* add 3 D points      *///////////////////////////////////////
////            int p3dId = p2d[i][j].first;
//                int p3dId = std::distance(points3DIndices.begin(), points3DIndices.find(p2d[i][j].first));
//
////            std::cout<<"poseId: "<<i <<" p2d_p3d_Indices: "<< p3dId <<" p2d: "<<p2d[i][j].second.x <<" p3d: "<<position[p3dId][0]<<std::endl;
////                std::cout<<"translation[i]: "<<i << std::endl;
////                std::cout<<"cameraIndex: "   <<cameraIndex << std::endl;
//
//                problem.AddResidualBlock(cost_function, loss_function, rotation[cameraIndex], translation[cameraIndex],
//                                         position[p3dId]);
//            }
//
//        }
//
//    }
//
//
//    ceres::Solver::Options options;
//    options.preconditioner_type = ceres::SCHUR_JACOBI;
//    options.linear_solver_type = ceres::DENSE_SCHUR;
//    options.minimizer_progress_to_stdout = false;
//    options.max_num_iterations = 100;
//    options.function_tolerance = 7e-5;
//    // options.max_solver_time_in_seconds = 0.2;
//
//    ceres::Solver::Summary summary;
//    ceres::Solve(options, &problem, &summary);
//    std::cout << summary.BriefReport() << "\n";
//
//    std::vector<Sophus::SE3> newPoseVector;
////    map.update3DPoints(points3DIndices, position );
//    for (int l = startFrame; l < map.getCurrFrameIndex() ; l +=keyFrameStep) {
////    for (int l = 0; l < map.getCurrFrameIndex() ; l +=keyFrameStep) {
//        Eigen::Quaterniond quaterniond_(rotation[l][0],rotation[l][1],rotation[l][2],rotation[l][3]);
//        Eigen::Vector3d translation_(translation[l][0],translation[l][1],translation[l][2]);
//        Sophus::SE3 SE3_quaternion(quaterniond_,translation_);
////        map.setCumPose(l,SE3_quaternion);
//
////        std::cout<<"SE3_quaternion.matrix(): "<< l << std::endl<<SE3_quaternion.matrix()<<std::endl;
//        newPoseVector.push_back(SE3_quaternion);
//    }
//
//
//    // TO DO update3DPoints in map
//
//    // TO DO setCumPose in map
//
//    return newPoseVector;
//}
