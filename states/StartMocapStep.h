#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <mc_rbdyn/Robot.h>

#include "handover_controller.h"
#include "helper_functions.h"


#include "../cortex/cortex.h"

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

			bool onceTrue{true};
			bool plotSize{true};

			int robotBody{0};
			int robotMarkerNo{1};

			int objMarkerNo{0};
			int objBody{1};

			int fps{200};

			int tune1{0};
			int tune2{0};

			/*mocap*/
			Eigen::Vector3d robotBodyMarker, objectBodyMarker;
			Eigen::Vector3d curPosLeftEf, curPosLeftEfMarker;
			Eigen::Vector3d initPosObjMarkerA, ithPosObjMarkerA, avgVelObjMarkerA, predictPos;

			Eigen::MatrixXd posLeftEfMarker  = Eigen::MatrixXd::Zero(3,60000);
			Eigen::MatrixXd posObjMarkerA    = Eigen::MatrixXd::Zero(3,60000);
			
			Eigen::MatrixXd newPosObjMarkerA;
			Eigen::MatrixXd curVelObjMarkerA, wp;//, curPosObjMarkerA;

			Eigen::Matrix3d curRotLeftEf;
			Eigen::Matrix3d curRotLeftEfMarker  = Eigen::Matrix3d::Identity();
			Eigen::Matrix3d rotObjMarkerA = Eigen::Matrix3d::Identity();

			std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_efL_objMarkerA;


		private:

			bool startCapture{false};
			bool startTraj{true};
			bool wpReady{false};

			bool Flag_CirTraj{false};
			bool Flag_PosTask{false};			
			bool Flag_CORTEX{false};

			Eigen::MatrixXd bot, obj, eflrot, eflpos;

			sBodyDefs* pBodyDefs{NULL};
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
