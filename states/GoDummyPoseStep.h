#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include "handover_controller.h"

namespace mc_handover
{
	namespace states
	{
		struct GoDummyPoseStep : mc_control::fsm::State
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

				Eigen::Vector3d posL, posR;
				Eigen::Matrix3d getCurRotL, getCurRotR;				

		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("GoDummyPoseStep", mc_handover::states::GoDummyPoseStep)