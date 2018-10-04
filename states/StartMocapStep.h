#pragma once

#include <iostream>
#include <fstream>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <mc_rbdyn/Robot.h>

#include "handover_controller.h"
#include "helper_functions.h"


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
			void configure(const mc_rtc::Configuration & config) override {}

			void start(mc_control::fsm::Controller&) override;

			bool run(mc_control::fsm::Controller&) override;

			void teardown(mc_control::fsm::Controller&) override {}

			bool onceTrue{true};
			bool plotSize{true};

			int body{0};
			int markerO{2};
			int markerR{3};

			int robotBody{0};
			int robotMarkerNo{0};

			int objBody{1};
			int objMarkerNo{0};

			int fps{200};

			int tune1{50}; //100ms
			int tune2{400};//1sec

			/*mocap*/
			Eigen::Vector3d robotBodyMarker, objectBodyMarker;
			Eigen::Vector3d curPosLeftEf, curPosLeftEfMarker;
			Eigen::Vector3d initPosObjMarkerA, ithPosObjMarkerA, avgVelObjMarkerA, predictPos;

			// Eigen::MatrixXd dummyPredictPos	 = Eigen::MatrixXd::Zero(3,5);
			Eigen::MatrixXd posLeftEfMarker  = Eigen::MatrixXd::Zero(3,60000);
			Eigen::MatrixXd posObjMarkerA    = Eigen::MatrixXd::Zero(3,60000);
			
			Eigen::MatrixXd newPosObjMarkerA = Eigen::MatrixXd::Zero(3,tune1);

			Eigen::MatrixXd curVelObjMarkerA, wp;//, curPosObjMarkerA;

			Eigen::Matrix3d curRotLeftEf;
			Eigen::Matrix3d curRotLeftEfMarker  = Eigen::Matrix3d::Identity();
			Eigen::Matrix3d rotObjMarkerA = Eigen::Matrix3d::Identity();

			std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_efL_objMarkerA;


			std::vector<std::string> activeJointsLeftArm =
			{
				"LARM_JOINT0",
				"LARM_JOINT1",
				"LARM_JOINT2",
				"LARM_JOINT3",
				"LARM_JOINT4",
				"LARM_JOINT5",
				"LARM_JOINT6",
				"LARM_JOINT7"};


			private:

				
				double pt;
				Eigen::MatrixXd pos;
				std::vector<double> pts;
				std::string name;

				bool Flag_CORTEX{false};

				bool startPrediction{true}; // default false for simData

				bool startCapture{false};

				bool collected{false};
				bool removePrevTask{true};

				Eigen::Vector3d refPos, refVel, refAcc, initRefPos;

				sBodyDefs* pBodyDefs{NULL};
				sFrameOfData* getCurFrame{NULL};
				sFrameOfData FrameofData;

				std::vector<int> bodyMarkers;

				void *pResponse;
				int nBytes;
				int retval = RC_Okay;
				int totalBodies;
				int i{1};
				int dm{0};

				double del{0};

			};


	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("StartMocapStep", mc_handover::states::StartMocapStep)