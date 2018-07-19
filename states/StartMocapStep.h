#pragma once

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include "handover_controller.h"

#include "../cortex/cortex.h"
#include "../cortex/graph.h"


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

			private:

			sBodyDefs* pBodyDefs = NULL;
			sFrameOfData* FrameofData = NULL;

			std::vector<int> bodyMarkers;

			void *pResponse;
			int nBytes;
			int retval = RC_Okay;
			double del = 0;  

			// live plot //
			plot p;
			vector<double> xm, xm1, xm2;
				



		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("StartMocapStep", mc_handover::states::StartMocapStep)
