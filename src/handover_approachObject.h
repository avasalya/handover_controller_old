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
#include <tuple>
#include <queue>
#include <utility>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <mc_control/mc_controller.h>
#include <mc_control/mc_global_controller.h>

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/logging.h>

#include <mc_tasks/TrajectoryTask.h>

#include <Tasks/QPTasks.h>

#include "handover_controller.h"
#include "handover_trajectories.h"


using namespace std;
using namespace Eigen;

namespace mc_handover 
{
	struct ApproachObject
	{
	public:
		ApproachObject();
		~ApproachObject();

		// ApproachObject(const ApproachObject & );

		void initials();

		bool checkFrameOfData(std::vector<Eigen::Vector3d>);

		bool handoverRun();

		std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> predictionController(const Eigen::Vector3d& curPosEf, const Eigen::Matrix3d & constRotLink6, std::vector<std::string> lShpMarkersName);

		bool goToHandoverPose(double min, double max, bool& enableHand, Eigen::Vector3d& curPosEf, std::shared_ptr<mc_tasks::PositionTask>& posTask, std::shared_ptr<mc_tasks::OrientationTask>& oriTask, std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> handPredict, Eigen::Vector3d fingerPos);

		bool forceController(bool& enableHand, Eigen::Vector3d initPos, Eigen::Matrix3d initRot, Eigen::Vector3d handForce, Eigen::Vector3d Th, std::shared_ptr<mc_tasks::PositionTask>& posTask, std::shared_ptr<mc_tasks::OrientationTask>& oriTask, std::string gripperName, std::vector<std::string> robotMarkersName, std::vector<std::string> lShpMarkersName);

		bool Flag_withoutRobot{false}; //TRUE, otherwise use ROBOT_Markers

		bool Flag_prediction{false}; //TRUE otherwise, use fingerPos
		
		Eigen::Vector3d tuner;

		int fps{200};
		int t_predict;
		int t_observe;
		int it;

		int i{0};
		int e{1};
		
		int totalMarkers;
		
		std::vector<Eigen::Vector3d> Markers;
		std::vector<Eigen::MatrixXd> markersPos;

		std::map<std::string, double> markers_name_index;
		std::vector<std::string> strMarkersBodyName, strMarkersName;
		std::vector<std::string> robotLtMarkers, subjLtMarkers, robotRtMarkers, subjRtMarkers;

		Eigen::Vector3d objectPos, fingerPosL, fingerPosR;
		Eigen::Matrix3d idtMat = Eigen::Matrix3d::Identity();
		double obj_rel_subjLtHand, obj_rel_subjRtHand, obj_rel_robotLtHand, obj_rel_robotRtHand;

		std::shared_ptr<mc_handover::HandoverTrajectory> handoverTraj;
		
		std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> lHandPredict, rHandPredict;

		bool useLeftEf{false};
		bool useRightEf{false};

		bool enableLHand{true};
		bool enableRHand{true};

		bool gOpen{false};
		bool gClose{false};
		bool openGripper{false};
		bool closeGripper{false};

		bool graspObject{true};
		bool takeBackObject{false};

		bool goBackInit{true};
		bool restartHandover{false};


	public:
		std::vector<double> Floadx, Floady, Floadz;
		double objMass;
		// double FNormAtClose{1.0};
		Eigen::Vector3d efAce, newTh, Finert, Fzero, Fclose, Fload, Fpull;

	};//strcut ApproachObject

}//namespace mc_handover
