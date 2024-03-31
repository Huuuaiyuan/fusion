#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <vector>
#include <map>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <stdio.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <stdarg.h>
#include <iomanip>

ros::Publisher pub_global_odometry;
double last_t = 0;
std::map<double, std::vector<double>> RelocalizePoseMap;
std::map<double, Eigen::Matrix4d> PerturbPoseMap;

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //printf("vio_callback! \n");
    double t = pose_msg->header.stamp.toSec();
    // std::cout << std::setprecision(24) << "time: " << t << std::endl;
    // printf("time: %f", pose_msg->header.stamp.toSec() );
    if(last_t == 0){
        last_t = t;
        return;
    }
    if(t - last_t < 1)
        return;
    last_t = t;
    std::cout << std::setprecision(24) << "GET ENTER time: " << t << std::endl;
    // printf("Odometry: time: %f, t: %f %f %f q: %f %f %f %f \n", header.stamp.toSec(), 
    //                         estimator.Ps[WINDOW_SIZE].x(), estimator.Ps[WINDOW_SIZE].y(), estimator.Ps[WINDOW_SIZE].z(),
    //                                                       tmp_Q.w(), tmp_Q.x(), tmp_Q.y(), tmp_Q.z());

    std::vector<double> relocate_pose = RelocalizePoseMap[t];

    std::cout << "get relocate pose size:" << relocate_pose.size() << std::endl;

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    // std::cout << "1" << std::endl;
    odometry.header.frame_id = "relocal_world";
    odometry.child_frame_id = "relocal_world";
    // std::cout << "2" << std::endl;
    odometry.pose.pose.position.x = relocate_pose[0];
    odometry.pose.pose.position.y = relocate_pose[1];
    odometry.pose.pose.position.z = relocate_pose[2];
    // std::cout << "3" << std::endl;
    odometry.pose.pose.orientation.w = relocate_pose[3];
    odometry.pose.pose.orientation.x = relocate_pose[4];
    odometry.pose.pose.orientation.y = relocate_pose[5];
    odometry.pose.pose.orientation.z = relocate_pose[6];
    
    pub_global_odometry.publish(odometry);

    std::cout << "relocate x: " << relocate_pose[0] << std::endl;

    std::string VINS_RESULT_PATH = "/home/huaiyuan/catkin_fusion_ws/src/VINS-Fusion/output/vio_relocal.csv";
    std::ofstream foutC(VINS_RESULT_PATH, std::ios::app);
    foutC.setf(std::ios::fixed, std::ios::floatfield);
    foutC.precision(0);
    foutC << pose_msg->header.stamp.toSec() * 1e9 << " ";
    foutC.precision(5);
    foutC   << relocate_pose[0] << " "
            << relocate_pose[1] << " "
            << relocate_pose[2] << " "
            << relocate_pose[3] << " "
            << relocate_pose[4] << " "
            << relocate_pose[5] << " "
            << relocate_pose[6] << std::endl;
    foutC.close();
}

Eigen::Matrix3d SkewTransform(const Eigen::Vector3d& V) {
    Eigen::Matrix3d M;
    M(0, 0) = 0;
    M(0, 1) = -V(2, 0);
    M(0, 2) = V(1, 0);
    M(1, 0) = V(2, 0);
    M(1, 1) = 0;
    M(1, 2) = -V(0, 0);
    M(2, 0) = -V(1, 0);
    M(2, 1) = V(0, 0);
    M(2, 2) = 0;
    return M;
}

Eigen::Matrix3d LieAlgebraToR(const Eigen::Vector3d& eps) {
    Eigen::Matrix3d tmp_res = SkewTransform(eps);
    return tmp_res.exp();
}

Eigen::Matrix4d PerturbTransformRadM(const Eigen::Matrix4d& T_in,
                                     const Eigen::VectorXd& perturbations) {
    Eigen::Vector3d r_perturb = perturbations.block(0, 0, 3, 1);
    Eigen::Vector3d t_perturb = perturbations.block(3, 0, 3, 1);
    Eigen::Matrix3d R_in = T_in.block(0, 0, 3, 3);
    Eigen::Matrix3d R_out = LieAlgebraToR(r_perturb) * R_in;
    Eigen::Matrix4d T_out;
    T_out.setIdentity();
    T_out.block(0, 3, 3, 1) = T_in.block(0, 3, 3, 1) + t_perturb;
    T_out.block(0, 0, 3, 3) = R_out;
    return T_out;
}

Eigen::Matrix4d QuaternionAndTranslationToTransformMatrix(const std::vector<double>& pose) {
    Eigen::Quaternion<double> quaternion{pose[0], pose[1], pose[2], pose[3]};
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block(0, 0, 3, 3) = quaternion.toRotationMatrix();
    T(0, 3) = pose[4];
    T(1, 3) = pose[5];
    T(2, 3) = pose[6];
    return T;
}

// void loadPoses(std::string file_name) {
// //   std::vector<Matrix> poses;
//   FILE *fp = fopen(file_name.c_str(),"r");
//   if (!fp)
//     return;
//   while (!feof(fp)) {
//     // Matrix P = Matrix::eye(4);
//     double t, t_x, t_y, t_z, q_x, q_y, q_z, q_w;
//     // if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
//     //                &P.val[0][0], &P.val[0][1], &P.val[0][2], &P.val[0][3],
//     //                &P.val[1][0], &P.val[1][1], &P.val[1][2], &P.val[1][3],
//     //                &P.val[2][0], &P.val[2][1], &P.val[2][2], &P.val[2][3] )==12)
//     if (fscanf(fp, "%ld %d %d %d %d %d %d %d",
//                    &t, t_x, t_y, t_z, q_x, q_y, q_z, q_w )==12) {
//         std::vector<double> pose = {t_x, t_y, t_z, q_x, q_y, q_z, q_w};
//         pub_global_odometry[t] = pose;
//     }
//   }
//   fclose(fp);
//   return;
// }

void loadPoses(std::string file_name){
    std::ifstream fin1(file_name);
    int i=0;
    while (fin1)  //读取真实轨迹参数
    {
        double data1[8] = {0};  //其中data1[0]表示时间，跳过（略去）
        for (auto &d1:data1)
            fin1 >> d1;
        // Eigen::Vector3d t1(data1[1], data1[2], data1[3]);    //获取平移数据
        // Eigen::Quaterniond q1(data1[7], data1[4], data1[5], data1[6]);    //获取旋转数据，注意输入是实1,虚1,虚2,虚3
        std::vector<double> pose = {data1[1], data1[2], data1[3], data1[4], data1[5], data1[6], data1[7]};
        double t = data1[0]/ 1e9;
        RelocalizePoseMap[t] = pose;

        i++;
    }
    std::cout << "set up " << i << " relocalization poses." << std::endl;

}

void set_up(std::string file_name){

    // R = T_LV_pert.block(0, 0, 3, 3);
    // q = Eigen::Quaternion<double>(R);
    loadPoses(file_name);
    Eigen::VectorXd perturbation(6, 1);
    perturbation << 0.3, -0.3, 0.3, 0.5, -0.5, 0.3;


    std::map<double, std::vector<double>>::iterator iterPose;
    int i = 0;
    for (iterPose = RelocalizePoseMap.begin(); iterPose != RelocalizePoseMap.end(); iterPose++, i++){
        std::vector<double> input = iterPose->second;
        Eigen::Matrix4d T_gt = QuaternionAndTranslationToTransformMatrix(input);
        Eigen::Matrix4d T_pert = PerturbTransformRadM(T_gt, perturbation);
        PerturbPoseMap[iterPose->first] = T_pert;
    }

    // std::vector<double> input = {0, 0, 0, 0, 0, 0, 0};
    // // std::vector<double> tmp{input[0], };
    // // tmp = std::vector<double>{q.w(),          q.x(),           q.y(),
    // //                             q.z(),          T_LV_pert(0, 3), T_LV_pert(1, 3),
    // //                             T_LV_pert(2, 3)};
    // // results_perturbed_init.push_back(tmp);

    // Eigen::Matrix4d T_ceres_initial_camera =
    //   QuaternionAndTranslationToTransformMatrix(input);


    // Eigen::Matrix4d T_LV_pert;
    // Eigen::VectorXd perturbation(6, 1);
    // perturbation << 0.3, -0.3, 0.3, 0.5, -0.5, 0.3;
    // T_LV_pert = PerturbTransformRadM(T_ceres_initial_camera, perturbation);
}


int main(int argc, char **argv)
{   
    std::string groundtruth_addr = "/home/huaiyuan/Downloads/EurocMAV_dataset/vicon1_medium/mav0/state_groundtruth_estimate0/tum.csv";
    std::cout << "groundtruth_addr: " << groundtruth_addr << std::endl; 
    set_up(groundtruth_addr);

    std::string relocal_save_path = "/home/huaiyuan/catkin_fusion_ws/src/VINS-Fusion/output/vio_relocal.csv";
    std::ofstream foutC(relocal_save_path, std::ios::trunc);
    // foutC.setf(std::ios::fixed, std::ios::floatfield);
    if (foutC.is_open()){
        foutC << "#relocalization reuslts: " << std::endl;
        foutC.close();  
    }else{
        std::cout << "Error opening: " << relocal_save_path << std::endl;
    }
    

    ros::init(argc, argv, "relocalizationServer");
    ros::NodeHandle n("~");

    // global_path = &globalEstimator.global_path;
    

    // ros::Subscriber sub_GPS = n.subscribe("/gps", 100, GPS_callback);
    // ros::Subscriber sub_relocal = n.subscribe("/relocalization/odometry", 100, Relocalization_callback);
    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 100, vio_callback);

    // pub_global_path = n.advertise<nav_msgs::Path>("global_path", 100);
    pub_global_odometry = n.advertise<nav_msgs::Odometry>("odometry", 100);
    // pub_car = n.advertise<visualization_msgs::MarkerArray>("car_model", 1000);
    ros::spin();
    return 0;
}