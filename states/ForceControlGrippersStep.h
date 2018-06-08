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

				void teardown(mc_control::fsm::Controller&) override;

				// Eigen::MatrixXd handForces  = Eigen::MatrixXd::Zero(4,3);
				// Eigen::MatrixXd handTorques = Eigen::MatrixXd::Zero(4,3);

			private:
				
				double openGrippers{0.5};
				double CloseGrippers{0.2};

				std::shared_ptr<mc_tasks::CoMTask> comTask;
				Eigen::Vector3d initialCom = Eigen::Vector3d::Zero();
				Eigen::Vector3d move, target;
				 
				Eigen::VectorXd handsWrenchTh	= Eigen::VectorXd::Zero(12);
				Eigen::VectorXd handsWrenchDir	= Eigen::VectorXd::Zero(12);

				Eigen::Vector6d leftHandWrenchTh, rightHandWrenchTh;
				Eigen::Vector6d leftHandWrenchDir, rightHandWrenchDir;


		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("ForceControlGrippersStep", mc_handover::states::ForceControlGrippersStep)