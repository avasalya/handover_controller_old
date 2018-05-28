#include "openGrippersStep.h"

namespace mc_handover
{
	namespace states {

		void openGrippersStep::configure(const mc_rtc::Configuration & config)
		{
			config("openGrippers", openGrippers);
		}

		void openGrippersStep::start(mc_control::fsm::Controller & controller)
		{
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

    		auto  gripper = ctl.grippers["l_gripper"].get();
    		gripper->setTargetQ({openGrippers});

    		gripper = ctl.grippers["r_gripper"].get();
    		gripper->setTargetQ({openGrippers});
		}

	} // namespace states

} // namespace mc_torquing_controller
