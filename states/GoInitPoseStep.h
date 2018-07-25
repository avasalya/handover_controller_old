#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <mc_rbdyn/Robot.h>
// #include <mc_rbdyn/Surface.h>
// #include <mc_rbdyn/surface_utils.h>

#include "handover_controller.h"

namespace mc_handover
{
	namespace states
	{
		struct GoInitPoseStep : mc_control::fsm::State
		{
			public:
				void configure(const mc_rtc::Configuration & config) override;

				void start(mc_control::fsm::Controller&) override;

				bool run(mc_control::fsm::Controller&) override;

				void teardown(mc_control::fsm::Controller&) override {}

			private:
				// Configs
				double threshold_eval_;
				double threshold_speed_;
				double stiffness_;
				double weight_;

				Eigen::Vector3d initPosR, initPosL;
				sva::PTransformd BodyPosW;

				std::shared_ptr<mc_tasks::PositionTask> chestPosTask;
				std::shared_ptr<mc_tasks::OrientationTask> chestOriTask;



		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("GoInitPoseStep", mc_handover::states::GoInitPoseStep)