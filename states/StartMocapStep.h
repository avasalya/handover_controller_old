#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <mc_rbdyn/Robot.h>

#include "handover_controller.h"

#include "../cortex/cortex.h"
// #include "../cortex/graph.h"


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
				
				/*robot*/
				Eigen::Vector3d robotBodyMarker, objectBodyMarker;

				Eigen::MatrixXd posLeftEfMarker = Eigen::MatrixXd::Zero(3,60000);
				Eigen::MatrixXd posObjMarkerA   = Eigen::MatrixXd::Zero(3,60000);

				Eigen::Vector3d curPosLeftEf, curPosLeftEfMarker;
				Eigen::Matrix3d curRotLeftEf, curRotLeftEfMarker;

				Eigen::Matrix3d rotObjMarkerA;


				bool onceTrue = true;

				sva::PTransformd leftHandPosW;
				sva::MotionVecd leftHandVelW;
				sva::PTransformd rightHandPosW;
				sva::MotionVecd rightHandVelW;

			private:

				/*cortex*/
				sBodyDefs* pBodyDefs = NULL;
				sFrameOfData* getCurFrame = NULL;
				sFrameOfData FrameofData;

				void *pResponse;
				int nBytes;
				int retval = RC_Okay;

				int totalBodies;
				std::vector<int> bodyMarkers;

				bool startCapture = false;
				double del = 0;
				int i=0;				

		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("StartMocapStep", mc_handover::states::StartMocapStep)
