#include "OpenGrippersStep.h"

namespace mc_handover
{
	namespace states {

		void OpenGrippersStep::configure(const mc_rtc::Configuration & config)
		{	
			cout << "config " <<endl;
			config("openGrippers", openGrippers);
		}

		void OpenGrippersStep::start(mc_control::fsm::Controller & controller)
		{	
			cout << "start -- opening grippers " <<endl;
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

    		auto  gripper = ctl.grippers["l_gripper"].get();
    		gripper->setTargetQ({openGrippers});

    		gripper = ctl.grippers["r_gripper"].get();
    		gripper->setTargetQ({openGrippers});
		}

		bool OpenGrippersStep::run(mc_control::fsm::Controller & controller)
		{	
			cout << "run " <<endl;
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
		
			auto  gripperL = ctl.grippers["l_gripper"].get();
			auto  gripperR = ctl.grippers["r_gripper"].get();


			if( (gripperL->curPosition()[0] < 0.5) && (gripperR->curPosition()[0] < 0.5) )
			{
				output("OK");
				return true;
			}
			else
			{
				output("Repeat");
				return true;
			}

		}

	} // namespace states

} // namespace mc_torquing_controller
