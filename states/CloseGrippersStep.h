#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include "handover_controller.h"

namespace mc_handover
{
	namespace states
	{
		struct CloseGrippersStep : mc_control::fsm::State
		{
			public:
				void configure(const mc_rtc::Configuration & config) override;

				void start(mc_control::fsm::Controller&) override;

				bool run(mc_control::fsm::Controller&) override;

				void teardown(mc_control::fsm::Controller&) override {}

			private:
				// Configs

				double closeGrippers =  -0.25;
								
		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("closeGrippersStep", mc_handover::states::closeGrippersStep)