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

		bool predictionController(Eigen::Vector3d p, Eigen::Quaterniond q, std::string subjHandReady, const sva::PTransformd& robotEf, const Eigen::Matrix3d & curRotLink6, std::vector<std::string> lShpMarkersName, std::vector<std::string> robotMarkersName);

		bool goToHandoverPose(double min, double max, const sva::PTransformd& robotEf, std::shared_ptr<mc_tasks::OrientationTask>& oriTask, std::shared_ptr<mc_tasks::PositionTask>& posTask);

		bool handoverForceController(Eigen::Vector3d handForce, Eigen::Vector3d Th, std::string gripperName, std::vector<std::string> robotMarkersName, std::vector<std::string> lShpMarkersName);



		double closeGrippers{0.13};
		double openGrippers{0.5};

		bool Flag_withoutRobot{true}; // default True for using MOCAP without ROBOT_Markers

		bool useRobotLeftHand{false};
		bool useRobotRightHand{false};

		bool motion{true};
		
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
		sva::PTransformd X_R_ef_const;
		sva::PTransformd X_e_l, X_M_lshp;

		Eigen::Matrix3d idtMat = Eigen::Matrix3d::Identity();
		Eigen::Matrix3d subjHandRot, handoverRot;
		
		sva::PTransformd X_ef_Subj;
		
		MCController * controller_ = nullptr;
		std::shared_ptr<mc_handover::HandoverTrajectory> handoverTraj;

		Eigen::MatrixXd newPosSubj, curVelSubj, wp;

		std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_efL_Subj;

		Eigen::Vector3d ithPosSubj, avgVelSubj, predictPos;
		Eigen::Vector3d refPos, refVel, refAcc, initRefPos, handoverPos, objectPos, fingerPos, gripperEf;

		
		bool openGripper{false};
		bool closeGripper{false};
		bool readyToGrasp{false};
		bool restartHandover{false};

		bool dum1{true};
		bool dum2{true};
		bool dum3{true};


		std::vector<Eigen::Vector3d> efPos, efVel;
		Eigen::Vector3d efAce, Finert, Fzero, Fload, Fpull, Force;
		double efMass, FNormAtClose;
		std::vector<double> Floadx, Floady, Floadz;

	};//strcut ApproachObject

}//namespace mc_handover