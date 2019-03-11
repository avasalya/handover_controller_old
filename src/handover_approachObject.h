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

		bool handoverPresets();

		bool predictionController(const Eigen::Vector3d& efGripperPos, double subjHandReady, const sva::PTransformd& robotEf, const Eigen::Matrix3d & curRotLink6);
		
		// void ForceControl();

		// initial orientation
		ql = {0.95, 0.086, -0.25, -0.15};
		qr = {0.95, 0.086, -0.25, -0.15};
		
		//for mocap_temp
		q1l = {0.64, -0.01, -0.76, -0.06};
		q1r = {0.64, -0.01, -0.76, -0.06};

		p_l<< 0.3,0.3,1.1;
		p_r<<0.3, -0.3,1.1;




		std::vector<Eigen::MatrixXd> markersPos;
		std::vector<Eigen::Vector3d> Markers;

		bool Flag_withoutRobot{true}; // default True for using MOCAP without ROBOT_Markers

		bool collected{false};
		bool motion{true};

		bool restartHandover{false};
		bool openGripper{false};
		bool closeGripper{false};
		bool readyToGrasp{false};

		bool dum1{true};
		bool dum2{true};
		bool dum3{true};

		
		Eigen::Vector3d tuner;
		
		int it;
		int fps{200};
		int t_predict;
		int t_observe;

		int i{0};
		int e{0};

		int body{0};
		
		int totalMarkers{19};//14, 17

		int wristLtEfA{0}, wristLtEfB{1};
		int gripperLtEfA{2}, gripperLtEfB{3};

		int wristRtEfA{4}, wristRtEfB{5};
		int gripperRtEfA{6}, gripperRtEfB{7};

		int object{8};

		int fingerSubjLt{9};
		int lShapeLtA{10}, lShapeLtB{11}, lShapeLtC{12}, lShapeLtD{13};

		int fingerSubjRt{14};
		int lShapeRtA{15}, lShapeRtB{16}, lShapeRtC{17}, lShapeRtD{18};


		std::vector<std::string> strMarkers, robotLtMarkers, lShapeLtMarkers, robotRtMarkers, lShapeRtMarkers;

		Eigen::Vector3d rEf_wA_O, lEf_wA_O;
		Eigen::Vector3d rEf_wA_wB, lEf_wA_wB;
		Eigen::Vector3d rEf_wA_gA, lEf_wA_gA;
		Eigen::Vector3d rEf_wA_gB, lEf_wA_gB;
		Eigen::Vector3d rEf_wA_lf, lEf_wA_lf;

		Eigen::Vector3d rEf_wAB_theta_wAO, lEf_wAB_theta_wAO;
		Eigen::Vector3d rEf_wAB_theta_wAgA, lEf_wAB_theta_wAgA;
		Eigen::Vector3d rEf_wAB_theta_wAgB, lEf_wAB_theta_wAgB;
		Eigen::Vector3d rEf_wAB_theta_wAf, lEf_wAB_theta_wAf;

		Eigen::Vector3d rEf_area_wAB_O, lEf_area_wAB_O;
		Eigen::Vector3d rEf_area_wAB_gA, lEf_area_wAB_gA;
		Eigen::Vector3d rEf_area_wAB_gB, lEf_area_wAB_gB;
		Eigen::Vector3d rEf_area_wAB_f, lEf_area_wAB_f;

		double obj_rel_subjLtHand, obj_rel_subjRtHand;
		double subjHandReady;

		Eigen::Vector3d efLGripperPos, efRGripperPos;

		
		Eigen::Matrix3d curRotEf, curRotLink6;
		
		Eigen::Vector3d curPosEf;
		Eigen::Vector3d curPosLshp;

		Eigen::Vector3d curPosEfMarker;
		
		Eigen::Vector3d initPosSubj, ithPosSubj, avgVelSubj, predictPos;
		Eigen::Vector3d refPos, refVel, refAcc, initRefPos, handoverPos, objectPos;


		sva::PTransformd X_R_ef;
		sva::PTransformd X_M_efMarker;
		sva::PTransformd X_R_M;
		sva::PTransformd X_M_Subj;
		sva::PTransformd X_R_ef_const;
		sva::PTransformd X_e_l, X_M_lshp;

		sva::PTransformd X_ef_Subj;
		std::vector<sva::PTransformd> X_ef_S;

		std::vector<Eigen::Vector3d> predictedPositions;
		std::vector<Eigen::Vector3d> efPos, efVel;

		Eigen::MatrixXd newPosSubj;



		Eigen::Matrix3d idtMat = Eigen::Matrix3d::Identity();
		Eigen::Matrix3d curRotLeftEfMarker;
		Eigen::Matrix3d initRotLtLshp;
		Eigen::Matrix3d idt, subjLtHandRot,subjRtHandRot, handoverRot;

		Eigen::Vector3d x, y, z, lshp_X, lshp_Y, lshp_Z;

		Eigen::MatrixXd curVelSubj, wp;

		std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_efL_Subj;


		std::vector<double> Floadx, Floady, Floadz;
		Eigen::Vector3d efAce, Finert, Fzero, Fload, Fpull, Force;
		
		double efMass, FNormAtClose;


	};
}//namespace mc_handover