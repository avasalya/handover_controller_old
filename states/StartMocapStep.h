#pragma once

#include <iostream>
#include <fstream>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <mc_rbdyn/Robot.h>

#include "handover_controller.h"
#include "../cortex/cortex.h"

// Generated by cmake
#include "datapath.h"

namespace mc_handover
{
	namespace states
	{
		struct StartMocapStep : mc_control::fsm::State
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		public:
			void configure(const mc_rtc::Configuration & config) override;

			void start(mc_control::fsm::Controller&) override;

			bool run(mc_control::fsm::Controller&) override;

			void teardown(mc_control::fsm::Controller&) override;


			int fps{200};
			int t_observe{20}; //100ms
			int t_predict{100};//1sec


			/*mocap_simulaton*/
			double pt;
			std::vector<Eigen::MatrixXd> pos;
			std::vector<double> pts;
			std::string name;
			int  s{0};

			/*mocap*/
			bool Flag_CORTEX{true}; // default True for MOCAP

			std::vector<Eigen::Vector3d> Markers;
			std::vector<Eigen::MatrixXd> markersPos;

			int maxMarkers{12};

			int body{0}, headR{0}, chestR{1};
			int elbowR{2}, wristR{3};			
			int gripperLA{4}, gripperLB{5}, gripperLC{6}, gripperLD{7};

			int object{8};

			int wristS{9}, knuckleS{10}, elbowS{11};


			Eigen::Vector3d AB, CD, AC, AD, AO, AK, PQ;
			Eigen::Vector3d curPosLeftEf, curPosLeftEfMarker;
			Eigen::Vector3d initPosObj, ithPosObj, avgVelObj, predictPos;			

			/*Eigen::Matrix3d::Identity();*/
			Eigen::Matrix3d curRotLeftEfMarker;
			Eigen::Matrix3d curRotLeftEf;
			Eigen::Matrix3d rotObj;

			Eigen::MatrixXd curVelObj, wp;
			Eigen::MatrixXd newPosObj = Eigen::MatrixXd::Zero(3,t_observe);
			// Eigen::MatrixXd vecGripperL12;

			std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_efL_obj;

			sva::PTransformd Obj_X_efL;
			sva::PTransformd ltHand;

			std::shared_ptr<mc_tasks::PositionTask> chestPosTask;
			std::shared_ptr<mc_tasks::OrientationTask> chestOriTask;


			double closeGrippers = 0.0;
			double openGrippers = 1.0;


		private:

			std::shared_ptr<mc_tasks::CoMTask> comTask;

			std::vector<bool> handsWrenchDir;

			Eigen::VectorXd thresh = Eigen::VectorXd::Zero(12);
			Eigen::VectorXd baseTh = Eigen::VectorXd::Zero(12);

			Eigen::Vector3d move, target;
			Eigen::Vector3d initialCom = Eigen::Vector3d::Zero();
			Eigen::Vector3d refPos, refVel, refAcc, initRefPos, handoverPos, initPos;

			/*mocap*/			

			bool startCapture{false};
			bool collected{false};
			bool prediction{true}; // default true

			bool openGripper{true};
			bool closeGripper{false};

			/*cortex*/
			sBodyDefs* pBodyDefs{NULL};
			sBodyDef* pBody{NULL};
			sFrameOfData* getCurFrame{NULL};
			sFrameOfData FrameofData;

			std::vector<int> bodyMarkers;

			void *pResponse;
			int nBytes;
			int retval = RC_Okay;
			int totalBodies;
			int i{1};
			
			double del{0};

			};


	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("StartMocapStep", mc_handover::states::StartMocapStep)