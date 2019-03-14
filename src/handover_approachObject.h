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
// #include "../states/StartMocapStep.h"



using namespace std;
using namespace Eigen;

namespace mc_handover 
{
	struct ApproachObject
	{
		ApproachObject();
		~ApproachObject();

		void initials();

		bool checkFrameOfData(std::vector<Eigen::Vector3d>);

		bool handoverRun();

		bool predictionController(const sva::PTransformd& robotEf, const Eigen::Matrix3d & curRotLink6, std::vector<std::string> lShpMarkersName, std::vector<std::string> robotLtMarkers, double subjHandReady);

		void goToHandoverPose(const sva::PTransformd& robotEf, const std::shared_ptr<mc_tasks::OrientationTask>& oriTask, const std::shared_ptr<mc_tasks::PositionTask>& posTask);

		bool handoverForceController(Eigen::Vector3d handForce, Eigen::Vector3d Th, std::string gripperName, std::vector<std::string> lShpMarkersName, std::vector<std::string> robotMarkersName);

		Eigen::Vector3d p_r, p_l;
		Eigen::Quaterniond ql, qr, q1l, q1r;

		std::vector<Eigen::MatrixXd> markersPos;
		std::vector<Eigen::Vector3d> Markers;

		bool Flag_withoutRobot{true}; // default True for using MOCAP without ROBOT_Markers

		bool motion{true};
		bool collected{false};

		Eigen::Vector3d tuner;

		int it;
		int fps{200};
		int t_predict;
		int t_observe;

		int i{1};
		int e{0};

		int body{0};

		int totalMarkers{19};//14, 19

		int wristLtEfA{0}, wristLtEfB{1};
		int gripperLtEfA{2}, gripperLtEfB{3};

		int wristRtEfA{4}, wristRtEfB{5};
		int gripperRtEfA{6}, gripperRtEfB{7};

		int object{8};

		int fingerSubjLt{9};
		int lShapeLtA{10}, lShapeLtB{11}, lShapeLtC{12}, lShapeLtD{13};

		int fingerSubjRt{14};
		int lShapeRtA{15}, lShapeRtB{16}, lShapeRtC{17}, lShapeRtD{18};

		std::vector<std::string> strMarkersName, robotLtMarkers, lShapeLtMarkers, robotRtMarkers, lShapeRtMarkers;
		std::map<std::string, double> markers_name_index; 

		Eigen::Vector3d ef_wA_O, ef_wA_wB, ef_wA_gA, ef_wA_gB, ef_wA_lf;

		double ef_wAB_theta_wAO;
		double ef_wAB_theta_wAgA;
		double ef_wAB_theta_wAgB;
		double ef_wAB_theta_wAf;

		double ef_area_wAB_O;
		double ef_area_wAB_gA;
		double ef_area_wAB_gB;
		double ef_area_wAB_f;

		double obj_rel_subjLtHand, obj_rel_subjRtHand;
		double subjHandReady;

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
		std::vector<sva::PTransformd> X_ef_S;
		
		std::shared_ptr<mc_handover::HandoverTrajectory> handoverTraj;
		MCController * controller_ = nullptr;

		Eigen::MatrixXd newPosSubj, curVelSubj, wp;

		std::vector<Eigen::Vector3d> predictedPositions;
		std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_efL_Subj;

		Eigen::Vector3d ithPosSubj, avgVelSubj, predictPos;
		Eigen::Vector3d refPos, refVel, refAcc, initRefPos, handoverPos, objectPos;



		double closeGrippers{0.13};
		double openGrippers{0.5};
		
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
	};
}//namespace mc_handover