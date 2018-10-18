#pragma once

#include <cmath>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include "handover_controller.h"

namespace mc_handover
{
	namespace states
	{
		struct OpenGrippersStep : mc_control::fsm::State
		{
			public:
				void configure(const mc_rtc::Configuration & config) override;

				void start(mc_control::fsm::Controller&) override;

				bool run(mc_control::fsm::Controller&) override;
				// {
				// 	output("OK");
				// 	return true;
				// }

				void teardown(mc_control::fsm::Controller&) override {}

			private:
				// Configs
				double openGrippers{0.5};
				bool target{true};


		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("OpenGrippersStep", mc_handover::states::OpenGrippersStep)