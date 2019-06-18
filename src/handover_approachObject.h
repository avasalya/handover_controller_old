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

#include <mc_control/fsm/Controller.h>

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

		std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> predictionController(
			const Eigen::Vector3d& curPosEf,
			const Eigen::Matrix3d & constRotLink6,
			std::vector<std::string> lShpMarkersName);

		void goToHandoverPose(
			double min,
			double max,
			bool& enableHand,
			Eigen::Vector3d& curPosEf,
			std::shared_ptr<mc_tasks::PositionTask>& posTask,
			std::shared_ptr<mc_tasks::OrientationTask>& oriTask,
			std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> handPredict,
			Eigen::Vector3d offsetPos);

		bool forceController(
			bool& enableHand,

			Eigen::Vector3d initPosR, Eigen::Matrix3d initRotR,
			Eigen::Vector3d initPosL, Eigen::Matrix3d initRotL,
			Eigen::Vector3d relaxPos, Eigen::Matrix3d relaxRot,
			Eigen::VectorXd thresh,
			Eigen::Vector3d leftForce, Eigen::Vector3d rightForce,
			Eigen::Vector3d leftForceLo, Eigen::Vector3d rightForceLo,
			Eigen::Vector3d efLAce, Eigen::Vector3d efRAce,
			std::shared_ptr<mc_tasks::PositionTask>& posTaskL,
			std::shared_ptr<mc_tasks::OrientationTask>& oriTaskL,
			std::shared_ptr<mc_tasks::PositionTask>& posTaskR,
			std::shared_ptr<mc_tasks::OrientationTask>& oriTaskR,
			std::shared_ptr<mc_tasks::EndEffectorTask>& objEfTask);

		std::shared_ptr<mc_control::fsm::Controller> ctl;

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

		Eigen::Matrix3d subjLHandRot, subjRHandRot, objRot;

		Eigen::Vector3d gripperEfL, gripperEfR;
		Eigen::Vector3d gripperLtEfA, gripperRtEfA;
		Eigen::Vector3d gripperLtEfB, gripperRtEfB;

		Eigen::Vector3d fingerPosL, fingerPosR;
		Eigen::Vector3d objectPosL, objectPosC, objectPosR, objectPosCx, objectPosCy;
		double obj_rel_subjLtHand, obj_rel_subjRtHand, obj_rel_robotLtHand, obj_rel_robotRtHand;

		std::shared_ptr<mc_handover::HandoverTrajectory> handoverTraj;

		std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> lHandPredict, rHandPredict;

		bool useLeftEf{true};
		bool useRightEf{true};

		bool subjHasObject{true};
		bool robotHasObject{false};

		bool enableHand{true};

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
		double objMass{0.5};

		std::vector<double> FloadLx, FloadLy, FloadLz;
		std::vector<double> FloadRx, FloadRy, FloadRz;

		Eigen::Vector3d local_FzeroL = Eigen::Vector3d::Zero();
		Eigen::Vector3d newThL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FinertL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FzeroL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FcloseL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FloadL = Eigen::Vector3d::Zero();
		Eigen::Vector3d FpullL = Eigen::Vector3d::Zero();

		Eigen::Vector3d local_FzeroR = Eigen::Vector3d::Zero();
		Eigen::Vector3d newThR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FinertR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FzeroR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FcloseR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FloadR = Eigen::Vector3d::Zero();
		Eigen::Vector3d FpullR = Eigen::Vector3d::Zero();

	};//strcut ApproachObject

}//namespace mc_handover