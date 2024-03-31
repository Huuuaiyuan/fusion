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

#pragma once
#include <vector>
#include <map>
#include <iostream>
#include <mutex>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <ceres/ceres.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
// #include "LocalCartesian.hpp"
#include <GeographicLib/LocalCartesian.hpp>
#include "tic_toc.h"
#include <queue>
 #include <fstream>

using namespace std;

class GlobalOptimization
{
public:
	GlobalOptimization();
	~GlobalOptimization();
	// void inputGPS(double t, double latitude, double longitude, double altitude, double posAccuracy);
	void inputRocalization(double t, Eigen::Vector3d vio_t_relocal, Eigen::Quaterniond vio_q_relocal);
	void inputOdom(double t, Eigen::Vector3d OdomP, Eigen::Quaterniond OdomQ);
	void getGlobalOdom(Eigen::Vector3d &odomP, Eigen::Quaterniond &odomQ);
	nav_msgs::Path global_path;

private:
	// void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
	void optimizeRelocalization();
	void updateGlobalPath();
	void record_pose(std::string save_path, vector<double> globalPose, double t);

	// format t, tx,ty,tz,qw,qx,qy,qz
	map<double, vector<double>> localPoseMap;
	map<double, vector<double>> globalPoseMap;
	map<double, vector<double>> GPSPositionMap;
	map<double, vector<double>> relocalizationPoseMap;

	bool initGPS;
	bool newGPS;
	bool initRelocalization;
	bool newRelocalization;
	GeographicLib::LocalCartesian geoConverter;
	std::mutex mPoseMap;
	Eigen::Matrix4d WGPS_T_WVIO;
	Eigen::Matrix4d WVIO_T_WMap;
	Eigen::Vector3d lastP;
	Eigen::Quaterniond lastQ;
	std::thread threadOpt;

	std::queue<double> sliding_window;
	std::string opt_save_path;

};