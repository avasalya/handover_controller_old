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

		bool predictionController(const sva::PTransformd& robotEf, const Eigen::Matrix3d & curRotLink6, std::vector<std::string> lShpMarkersName, std::vector<std::string> robotMarkersName);

		bool goToHandoverPose(double min, double max, const sva::PTransformd& robotEf, std::shared_ptr<mc_tasks::PositionTask>& posTask, std::shared_ptr<mc_tasks::VectorOrientationTask>& vecOriTask);

		bool handoverForceController(Eigen::Vector3d handForce, Eigen::Vector3d Th, std::shared_ptr<mc_tasks::PositionTask>& posTask, std::shared_ptr<mc_tasks::VectorOrientationTask>& vecOriTask, std::string gripperName, std::vector<std::string> robotMarkersName, std::vector<std::string> lShpMarkersName);


		bool Flag_withoutRobot{true}; // default True for using MOCAP without ROBOT_Markers
		
		Eigen::Vector3d tuner;

		int fps{200};
		int t_predict;
		int t_observe;
		int it;

		int i{0};
		int e{0};
		
		int totalMarkers{19};//14, 19
		
		std::vector<Eigen::MatrixXd> markersPos;
		std::vector<Eigen::Vector3d> Markers;

		std::vector<std::string> strMarkersBodyName, strMarkersName, robotLtMarkers, lShapeLtMarkers, robotRtMarkers, lShapeRtMarkers;
		std::map<std::string, double> markers_name_index; 

		Eigen::Vector3d ef_wA_O, ef_wA_wB, ef_wA_gA, ef_wA_gB, ef_wA_f;

		double ef_wAB_theta_wAO;
		double ef_wAB_theta_wAgA;
		double ef_wAB_theta_wAgB;
		double ef_wAB_theta_wAf;

		double ef_area_wAB_O;
		double ef_area_wAB_gA;
		double ef_area_wAB_gB;
		double ef_area_wAB_f;

		double obj_rel_subjLtHand, obj_rel_subjRtHand, obj_rel_robotLtHand, obj_rel_robotRtHand, subj_rel_ef;

		Eigen::Matrix3d curRotEf, curRotLink6;

		Eigen::Vector3d curPosEf, curPosEfMarker,  curPosLshp;
		Eigen::Vector3d x, y, z, lshp_X, lshp_Y, lshp_Z;


		sva::PTransformd X_R_ef;
		sva::PTransformd X_M_efMarker;
		sva::PTransformd X_R_M;
		sva::PTransformd X_M_Subj;

		Eigen::Matrix3d idtMat = Eigen::Matrix3d::Identity();
		
		sva::PTransformd X_ef_Subj;
		
		std::shared_ptr<mc_handover::HandoverTrajectory> handoverTraj;

		Eigen::MatrixXd newPosSubj, curVelSubj, wp;

		std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_efL_Subj;

		Eigen::Vector3d ithPosSubj, avgVelSubj, predictPos;
		Eigen::Vector3d refPos, refVel, refAcc, initRefPos, handoverPos, objectPos, fingerPos, gripperEf;


		bool useLeftEf{false};
		bool useRightEf{false};

		bool motion{true};

		bool gOpen{false};
		bool gClose{false};
		bool openGripper{false};
		bool closeGripper{false};
		
		bool graspObject{true};
		bool takeBackObject{false};
		
		bool printOnce{true};
		bool restartHandover{false};


		double efMass, FNormAtClose;
		std::vector<Eigen::Vector3d> efPos, efVel;
		std::vector<double> Floadx, Floady, Floadz;
		Eigen::Vector3d efAce, Finert, Fzero, Fload, Fpull, Force;
		Eigen::Vector3d centerPos, initBodyVec, initTargetVec;

	};//strcut ApproachObject

}//namespace mc_handover