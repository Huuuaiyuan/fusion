/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "relocalOpt.h"
#include "Factors.h"

GlobalOptimization::GlobalOptimization()
{
	initGPS = false;
    newGPS = false;
    initRelocalization = false;
    newRelocalization = false;
	WGPS_T_WVIO = Eigen::Matrix4d::Identity();
    WVIO_T_WMap = Eigen::Matrix4d::Identity();
    opt_save_path = "/home/huaiyuan/catkin_fusion_ws/src/VINS-Fusion/output/vio_opt.csv";
    threadOpt = std::thread(&GlobalOptimization::optimizeRelocalization, this);

    
    std::ofstream foutC(opt_save_path, std::ios::trunc);
    // foutC.setf(std::ios::fixed, std::ios::floatfield);
    if(foutC.is_open()){
        foutC   << "#optimization reuslts: " << std::endl;
        foutC.close();
    }else{
        std::cout << "ERROR In open " << opt_save_path << std::endl;
    }
    

}

GlobalOptimization::~GlobalOptimization()
{
    threadOpt.detach();
}

// void GlobalOptimization::GPS2XYZ(double latitude, double longitude, double altitude, double* xyz)
// {
//     if(!initGPS)
//     {
//         geoConverter.Reset(latitude, longitude, altitude);
//         initGPS = true;
//     }
//     geoConverter.Forward(latitude, longitude, altitude, xyz[0], xyz[1], xyz[2]);
//     //printf("la: %f lo: %f al: %f\n", latitude, longitude, altitude);
//     //printf("gps x: %f y: %f z: %f\n", xyz[0], xyz[1], xyz[2]);
// }

void GlobalOptimization::inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ)
{
	mPoseMap.lock();
    vector<double> localPose{OdomP.x(), OdomP.y(), OdomP.z(), 
    					     OdomQ.w(), OdomQ.x(), OdomQ.y(), OdomQ.z()};
    localPoseMap[t] = localPose;
    // if(sliding_window.size() >= 30){
    //     sliding_window.pop();
    // }
    // sliding_window.push(t);

    Eigen::Quaterniond globalQ;
    globalQ = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomQ;
    Eigen::Vector3d globalP = WGPS_T_WVIO.block<3, 3>(0, 0) * OdomP + WGPS_T_WVIO.block<3, 1>(0, 3);
    vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                              globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
    globalPoseMap[t] = globalPose;
    lastP = globalP;
    lastQ = globalQ;

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time(t);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = lastP.x();
    pose_stamped.pose.position.y = lastP.y();
    pose_stamped.pose.position.z = lastP.z();
    pose_stamped.pose.orientation.x = lastQ.x();
    pose_stamped.pose.orientation.y = lastQ.y();
    pose_stamped.pose.orientation.z = lastQ.z();
    pose_stamped.pose.orientation.w = lastQ.w();
    global_path.header = pose_stamped.header;
    global_path.poses.push_back(pose_stamped);

    mPoseMap.unlock();
}

void GlobalOptimization::getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ)
{
    odomP = lastP;
    odomQ = lastQ;
}

// void GlobalOptimization::inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy)
// {
// 	double xyz[3];
// 	GPS2XYZ(latitude, longitude, altitude, xyz);
// 	vector<double> tmp{xyz[0], xyz[1], xyz[2], posAccuracy};
//     //printf("new gps: t: %f x: %f y: %f z:%f \n", t, tmp[0], tmp[1], tmp[2]);
// 	GPSPositionMap[t] = tmp;
//     newGPS = true;

// }

void GlobalOptimization::inputRocalization(double t, Eigen::Vector3d vio_t_relocal, Eigen::Quaterniond vio_q_relocal)
{
    vector<double> relocalPose{vio_t_relocal.x(), vio_t_relocal.y(), vio_t_relocal.z(), 
    					     vio_q_relocal.w(), vio_q_relocal.x(), vio_q_relocal.y(), vio_q_relocal.z()};
	std::cout << "Got Relocaliztion input" << std::endl;
    if(!initRelocalization){
        mPoseMap.lock();
        map<double, vector<double>>::iterator iterVIO = localPoseMap.find(t);
        if (iterVIO != localPoseMap.end()){

            Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d mTi = Eigen::Matrix4d::Identity();
            wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                        iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
            wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
            mTi.block<3, 3>(0, 0) = vio_q_relocal.toRotationMatrix();
            mTi.block<3, 1>(0, 3) = Eigen::Vector3d(vio_t_relocal.x(), vio_t_relocal.y(), vio_t_relocal.z());
            WVIO_T_WMap = wTi.inverse() * mTi;

            map<double, vector<double>>::iterator localPoseMapIter, globalPoseMapIter;
            localPoseMapIter = localPoseMap.begin(); 
            globalPoseMapIter = globalPoseMap.begin();
            for (int i = 0; i < localPoseMap.size(); i++, localPoseMapIter++, globalPoseMapIter++){
                Eigen::Quaterniond globalQ, localQ;
                localQ  = Eigen::Quaterniond(localPoseMapIter->second[3], localPoseMapIter->second[4], 
                                            localPoseMapIter->second[5], localPoseMapIter->second[6]);
                globalQ = WVIO_T_WMap.block<3, 3>(0, 0) * localQ;
                Eigen::Vector3d localP, globalP;
                localP = Eigen::Vector3d(localPoseMapIter->second[0], localPoseMapIter->second[1], localPoseMapIter->second[2]);
                globalP = WVIO_T_WMap.block<3, 3>(0, 0) * localP + WVIO_T_WMap.block<3, 1>(0, 3);
                
                vector<double> globalPose{globalP.x(), globalP.y(), globalP.z(),
                                        globalQ.w(), globalQ.x(), globalQ.y(), globalQ.z()};
                globalPoseMapIter->second = globalPose;
            }
            cout << "set up WVIO_T_WMap; and also update previoud global poses" << endl;
        }else{
            std::cout << "Error of initialization T_Map_VIO" << std::endl;
            return;
        }
        initRelocalization = true;
        mPoseMap.unlock();
    }
    relocalizationPoseMap[t] = relocalPose;
    newRelocalization = true;

}

void GlobalOptimization::optimizeRelocalization()
{
    while(true)
    {
        if(newRelocalization)
        {
            newRelocalization = false;
            printf("global optimization\n");
            TicToc globalOptimizationTime;

            ceres::Problem problem;
            ceres::Solver::Options options;
            options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
            //options.minimizer_progress_to_stdout = true;
            //options.max_solver_time_in_seconds = SOLVER_TIME * 3;
            options.max_num_iterations = 3; // original: 5
            ceres::Solver::Summary summary;
            ceres::LossFunction *loss_function;
            loss_function = new ceres::HuberLoss(1.0);
            ceres::LocalParameterization* local_parameterization = new ceres::QuaternionParameterization();

            //add param
            mPoseMap.lock();
            std::cout << "start adding global poses" << std::endl;
            int length = localPoseMap.size();
            cout << "original length of the pose map: " << length << endl;
            // w^t_i   w^q_i
            
            map<double, vector<double>>::iterator iter;
            iter = globalPoseMap.begin();
            
            if(length > 30){
                int z = 0;
                while(z < (length - 30)){
                    iter++;
                    z++;
                }
                std::cout << "limit global pose length" << std::endl;
            }
            if(length > 30)
                length = 30;
            double t_array[length][3];
            double q_array[length][4];
            for (int i = 0; i < length; i++, iter++)
            {
                t_array[i][0] = iter->second[0];
                t_array[i][1] = iter->second[1];
                t_array[i][2] = iter->second[2];
                q_array[i][0] = iter->second[3];
                q_array[i][1] = iter->second[4];
                q_array[i][2] = iter->second[5];
                q_array[i][3] = iter->second[6];
                problem.AddParameterBlock(q_array[i], 4, local_parameterization);
                problem.AddParameterBlock(t_array[i], 3);
            }

            // printf("start adding factors\n");

            // std::queue<double> tmp = sliding_window;
            // while(!tmp.empty()){
            //     double time = tmp.front();
            //     tmp.pop();

            // }

            length = localPoseMap.size();
            map<double, vector<double>>::iterator iterVIO, iterVIONext, iterGPS, iterRelocal;
            int i = 0;
            iterVIO = localPoseMap.begin();
            
            // std::cout << "add relocalization factors" << std::endl;
            std::cout << "length: " << localPoseMap.size() << std::endl;
            if(length > 30){
                int z = 0;
                while(z < (length - 30)){
                    iterVIO++;
                    z++;
                }
            }

            for (; iterVIO != localPoseMap.end(); iterVIO++, i++)
            {
                //vio factor
                iterVIONext = iterVIO;
                iterVIONext++;
                if(iterVIONext != localPoseMap.end())
                {
                    Eigen::Matrix4d wTi = Eigen::Matrix4d::Identity();
                    Eigen::Matrix4d wTj = Eigen::Matrix4d::Identity();
                    wTi.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIO->second[3], iterVIO->second[4], 
                                                               iterVIO->second[5], iterVIO->second[6]).toRotationMatrix();
                    wTi.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIO->second[0], iterVIO->second[1], iterVIO->second[2]);
                    wTj.block<3, 3>(0, 0) = Eigen::Quaterniond(iterVIONext->second[3], iterVIONext->second[4], 
                                                               iterVIONext->second[5], iterVIONext->second[6]).toRotationMatrix();
                    wTj.block<3, 1>(0, 3) = Eigen::Vector3d(iterVIONext->second[0], iterVIONext->second[1], iterVIONext->second[2]);
                    Eigen::Matrix4d iTj = wTi.inverse() * wTj;
                    Eigen::Quaterniond iQj;
                    iQj = iTj.block<3, 3>(0, 0);
                    Eigen::Vector3d iPj = iTj.block<3, 1>(0, 3);

                    ceres::CostFunction* vio_function = RelativeRTError::Create(iPj.x(), iPj.y(), iPj.z(),
                                                                                iQj.w(), iQj.x(), iQj.y(), iQj.z(),
                                                                                0.1, 0.01);
                    problem.AddResidualBlock(vio_function, NULL, q_array[i], t_array[i], q_array[i+1], t_array[i+1]);

                }
                //relocalization factor
                double t = iterVIO->first;
                iterRelocal = relocalizationPoseMap.find(t);
                if (iterRelocal != relocalizationPoseMap.end())
                {
                    std::vector<double> relocalPose = iterRelocal->second;
                    std::cout << "add relocalization factors" <<  std::endl;
                    std::cout << "i: " << i << std::endl;
                    ceres::CostFunction* relocal_function = RelocalizationError::Create(relocalPose[0], relocalPose[1], relocalPose[2], 
                                                                            relocalPose[3], relocalPose[4], relocalPose[5], relocalPose[6],
                                                                            0.1, 0.01);
                    //printf("inverse weight %f \n", iterGPS->second[3]);
                    std::cout << "finish create the factors" << std::endl;
                    // problem.AddResidualBlock(relocal_function, loss_function, q_array[i], t_array[i]);
                    problem.AddResidualBlock(relocal_function, NULL, q_array[i], t_array[i]);
                    std::cout << "finish relocalization AddResidualBlock" << std::endl;


                }

            }
            std::cout << "finish adding the factors" << std::endl;
            //mPoseMap.unlock();
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.BriefReport() << "\n";

            // update global pose
            //mPoseMap.lock();
            iter = globalPoseMap.begin();
            length = globalPoseMap.size();
            if(length > 30){
                int z = 0;
                while(z < (length - 30)){
                    iter++;
                    z++;
                }
                length = 30;
            }
            

            for (int i = 0; i < length; i++, iter++)
            {
            	vector<double> globalPose{t_array[i][0], t_array[i][1], t_array[i][2],
            							  q_array[i][0], q_array[i][1], q_array[i][2], q_array[i][3]};
            	iter->second = globalPose;
            	if(i == length - 1)
            	{
            	    Eigen::Matrix4d WVIO_T_body = Eigen::Matrix4d::Identity(); 
            	    Eigen::Matrix4d WMAP_T_body = Eigen::Matrix4d::Identity();
            	    double t = iter->first;
            	    WVIO_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(localPoseMap[t][3], localPoseMap[t][4], 
            	                                                       localPoseMap[t][5], localPoseMap[t][6]).toRotationMatrix();
            	    WVIO_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(localPoseMap[t][0], localPoseMap[t][1], localPoseMap[t][2]);
            	    WMAP_T_body.block<3, 3>(0, 0) = Eigen::Quaterniond(globalPose[3], globalPose[4], 
            	                                                        globalPose[5], globalPose[6]).toRotationMatrix();
            	    WMAP_T_body.block<3, 1>(0, 3) = Eigen::Vector3d(globalPose[0], globalPose[1], globalPose[2]);
            	    // WGPS_T_WVIO = WGPS_T_body * WVIO_T_body.inverse();

                    WVIO_T_WMap = WVIO_T_body.inverse() * WMAP_T_body;

                    record_pose(opt_save_path, globalPose, iter->first);
            	}
            }
            updateGlobalPath();
            printf("global time %f \n", globalOptimizationTime.toc());
            mPoseMap.unlock();
        }
        std::chrono::milliseconds dura(200);
        std::this_thread::sleep_for(dura);
    }
	return;
}


void GlobalOptimization::updateGlobalPath()
{
    global_path.poses.clear();
    map<double, vector<double>>::iterator iter;
    for (iter = globalPoseMap.begin(); iter != globalPoseMap.end(); iter++)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time(iter->first);
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = iter->second[0];
        pose_stamped.pose.position.y = iter->second[1];
        pose_stamped.pose.position.z = iter->second[2];
        pose_stamped.pose.orientation.w = iter->second[3];
        pose_stamped.pose.orientation.x = iter->second[4];
        pose_stamped.pose.orientation.y = iter->second[5];
        pose_stamped.pose.orientation.z = iter->second[6];
        global_path.poses.push_back(pose_stamped);
    }
}

void GlobalOptimization::record_pose(std::string save_path, vector<double> globalPose, double t){
    // std::string VINS_RESULT_PATH = "home/huaiyuan/catkin_fusion_ws/src/VINS-Fusion/output/vio_relocal.csv";
    std::ofstream foutC(save_path, std::ios::app);
    foutC.setf(std::ios::fixed, std::ios::floatfield);
    foutC.precision(0);
    foutC << t * 1e9 << " ";
    foutC.precision(5);
    foutC   << globalPose[0] << " "
            << globalPose[1] << " "
            << globalPose[2] << " "
            << globalPose[3] << " "
            << globalPose[4] << " "
            << globalPose[5] << " "
            << globalPose[6] << std::endl;
    foutC.close();
}