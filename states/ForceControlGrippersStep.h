#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include "handover_controller.h"

namespace mc_handover
{
	namespace states
	{
		struct ForceControlGrippersStep : mc_control::fsm::State
		{
			public:
				void configure(const mc_rtc::Configuration & config) override;

				void start(mc_control::fsm::Controller&) override;

				bool run(mc_control::fsm::Controller&) override;

				void teardown(mc_control::fsm::Controller&) override {};

				Eigen::MatrixXd handForces  = Eigen::MatrixXd::Zero(4,3);
				Eigen::MatrixXd handTorques = Eigen::MatrixXd::Zero(4,3);

			private:
				
				//configs 
				Eigen::Vector3d leftHandForcesTh, rightHandForcesTh;
				Eigen::Vector3d leftHandForcesDir, rightHandForcesDir;

				Eigen::Vector3d leftHandTorquesTh, rightHandTorquesTh;
				Eigen::Vector3d leftHandTorquesDir, rightHandTorquesDir;

				double openGrippers{0.5};
				double CloseGrippers{0.2};

				std::shared_ptr<mc_tasks::CoMTask> comTask;
				Eigen::Vector3d initialCom = Eigen::Vector3d::Zero();
				Eigen::Vector3d move, target;

		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("ForceControlGrippersStep", mc_handover::states::ForceControlGrippersStep)