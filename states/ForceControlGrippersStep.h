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
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			public:
				void configure(const mc_rtc::Configuration & config) override;

				void start(mc_control::fsm::Controller&) override;

				bool run(mc_control::fsm::Controller&) override;

				void teardown(mc_control::fsm::Controller&) override;

				// Eigen::MatrixXd handForces  = Eigen::MatrixXd::Zero(4,3);
				// Eigen::MatrixXd handTorques = Eigen::MatrixXd::Zero(4,3);

			private:
				
				double openGrippers{0.5};
				double closeGrippers{0.2};

				std::shared_ptr<mc_tasks::CoMTask> comTask;
				Eigen::Vector3d initialCom = Eigen::Vector3d::Zero();
				Eigen::Vector3d move, target;
				 

        Eigen::VectorXd thresh = Eigen::VectorXd::Zero(12);

				std::vector<bool> handsWrenchDir;
				std::vector<bool> leftHandWrenchDir;
				std::vector<bool> rightHandWrenchDir;



		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("ForceControlGrippersStep", mc_handover::states::ForceControlGrippersStep)
