#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <mc_rbdyn/Robot.h>

#include "handover_controller.h"

#include "../cortex/cortex.h"

namespace plt = matplotlibcpp;


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

				void teardown(mc_control::fsm::Controller&) override {}

				/*helper function*/
				void plotPos(Eigen::MatrixXd m, int d);
				void plotVel(Eigen::MatrixXd m, int d);

				bool onceTrue = true;
				bool plotSize = true;

				int robotBody=0;
				int objBody=1;
				int fps = 200;
				int tunParam1 = 200;
				int tunParam2 = 1000;

				
				/*mocap*/
				Eigen::Vector3d robotBodyMarker, objectBodyMarker;
				Eigen::Vector3d curPosLeftEf, curPosLeftEfMarker;
				Eigen::Vector3d initPosObjMarkerA, ithPosObjMarkerA, avgVelObjMarkerA;

				Eigen::MatrixXd posLeftEfMarker  = Eigen::MatrixXd::Zero(3,60000);
				Eigen::MatrixXd posObjMarkerA    = Eigen::MatrixXd::Zero(3,60000);
				Eigen::MatrixXd newPosObjMarkerA = Eigen::MatrixXd::Zero(3,tunParam1);
				Eigen::MatrixXd curVelObjMarkerA; //, curPosObjMarkerA;

				Eigen::Matrix3d curRotLeftEf;
				Eigen::Matrix3d curRotLeftEfMarker  = Eigen::Matrix3d::Identity();
				Eigen::Matrix3d rotObjMarkerA = Eigen::Matrix3d::Identity();
				
				sva::PTransformd leftHandPosW;
				sva::PTransformd rightHandPosW;

				sva::MotionVecd leftHandVelW;
				sva::MotionVecd rightHandVelW;



			private:

				/*cortex*/
				bool Flag_CORTEX = true;
				Eigen::MatrixXd bot, obj, eflrot, eflpos;

				sBodyDefs* pBodyDefs = NULL;
				sFrameOfData* getCurFrame = NULL;
				sFrameOfData FrameofData;

				std::vector<int> bodyMarkers;

				void *pResponse;
				int nBytes;
				int retval = RC_Okay;
				int totalBodies;
				int i=1;


				bool startCapture = false;
				
				double del = 0;

		        std::shared_ptr<mc_tasks::CoMTask> comt_;
		        shared_ptr<mc_control::fsm::Executor> execut;
		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("StartMocapStep", mc_handover::states::StartMocapStep)
