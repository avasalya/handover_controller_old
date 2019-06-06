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

		void initials();

		bool checkFrameOfData(std::vector<Eigen::Vector3d>);

		bool handoverRun();

		std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> predictionController(const Eigen::Vector3d& curPosEf, const Eigen::Matrix3d & constRotLink6, std::vector<std::string> lShpMarkersName);

		bool goToHandoverPose(double min, double max, bool& enableHand, Eigen::Vector3d& curPosEf, std::shared_ptr<mc_tasks::PositionTask>& posTask, std::shared_ptr<mc_tasks::OrientationTask>& oriTask, std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> handPredict, Eigen::Vector3d fingerPos);

		bool forceController(bool& enableHand, Eigen::Vector3d initPos, Eigen::Matrix3d initRot, Eigen::Vector3d handForce, Eigen::Vector3d Th,  Eigen::Vector3d efAce, std::shared_ptr<mc_tasks::PositionTask>& posTask, std::shared_ptr<mc_tasks::OrientationTask>& oriTask, std::string gripperName, std::vector<std::string> robotMarkersName, std::vector<std::string> lShpMarkersName, double obj_rel_robotHand);


		bool Flag_withoutRobot{false}; //TRUE, otherwise use ROBOT_Markers

		bool Flag_prediction{false}; //TRUE otherwise, use fingerPos


		Eigen::Vector3d tuner;

		int fps{200};
		int t_predict;
		int t_observe;
		int it;

		int i{1};
		int e{1};
		
		int totalMarkers;

		std::vector<Eigen::Vector3d> object;
		std::vector<Eigen::Vector3d> Markers;
		std::vector<Eigen::MatrixXd> markersPos;

		std::map<std::string, double> markers_name_index;
		std::vector<std::string> strMarkersBodyName, strMarkersName;
		std::vector<std::string> robotLtMarkers, robotRtMarkers, objMarkers, subjRtMarkers, subjLtMarkers, subjMarkers;

		Eigen::Matrix3d idtMat = Eigen::Matrix3d::Identity();
		Eigen::Matrix3d handoverRot_ = idtMat;

		Eigen::Matrix3d subjLHandRot, subjRHandRot;

		Eigen::Vector3d fingerPosL, fingerPosR;
		Eigen::Vector3d objectPosL, objectPosC, objectPosR;
		Eigen::Vector3d obj_to_robotLtHand, obj_to_robotRtHand;
		double obj_rel_subjLtHand, obj_rel_subjRtHand, obj_rel_robotLtHand, obj_rel_robotRtHand;

		std::shared_ptr<mc_handover::HandoverTrajectory> handoverTraj;

		std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> lHandPredict, rHandPredict;

		bool useLeftEf{true};
		bool stopLtEf{true};

		bool useRightEf{true};
		bool stopRtEf{true};

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

		bool startNow{false};

	public:
		std::vector<double> Floadx, Floady, Floadz;
		double objMass{0.2};
		Eigen::Vector3d newTh = Eigen::Vector3d::Zero();
		Eigen::Vector3d Finert = Eigen::Vector3d::Zero();
		Eigen::Vector3d Fzero = Eigen::Vector3d::Zero();
		Eigen::Vector3d Fclose = Eigen::Vector3d::Zero();
		Eigen::Vector3d Fload = Eigen::Vector3d::Zero();
		Eigen::Vector3d Fpull = Eigen::Vector3d::Zero();

	};//strcut ApproachObject

}//namespace mc_handover