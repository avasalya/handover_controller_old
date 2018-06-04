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

				std::shared_ptr<mc_rbdyn::Contact> contact;

			private:

				//configs 
				double ForceThresh_eval_;
				double ForceThresh_speed_;
								
		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("ForceControlGrippersStep", mc_handover::states::ForceControlGrippersStep)