#pragma once


#include <cmath>
#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <mc_control/mc_controller.h>

#include <mc_tasks/TrajectoryTask.h>

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/logging.h>

#include <Tasks/QPTasks.h>

using namespace std;
using namespace mc_control;
using namespace Eigen;

namespace mc_handover 
{

struct HandoverTrajectory
{

	public:

	HandoverTrajectory();

	~HandoverTrajectory();

	MatrixXd getPosA();
	MatrixXd getPosB();


	void constVelocity();
	
	void minJerkZeroBoundary(const MatrixXd & xi, const MatrixXd & xf, double tf);

	void minJerkPredictPos(const MatrixXd & xi, const MatrixXd & xc, double tc, double tf);

	void minJerkNonZeroBoundary(const MatrixXd & xi, const MatrixXd & vi, const MatrixXd & ai,
															 const MatrixXd & xc, double tf);

	MatrixXd wpPos;
	MatrixXd wpVel;
	MatrixXd wpAce;

	MatrixXd Pos;
	MatrixXd Vel;
	MatrixXd Ace;

};



} // namespace mc_handover
