#include "CloseGrippersStep.h"

namespace mc_handover
{
	namespace states {

		void CloseGrippersStep::configure(const mc_rtc::Configuration & config)
		{
			config("closeGrippers", closeGrippers);
		}

		void CloseGrippersStep::start(mc_control::fsm::Controller & controller)
		{
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
		}

		bool CloseGrippersStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
			
			ctl.solver().setContacts({
	        {ctl.robots(), 0, 1, "l_gripper", "cylinder1"}, // define cylinder1? ?
	        {ctl.robots(), 0, 1, "r_gripper", "cylinder1"}
	        });

			if(ctl.checkContact->isFixed())
			{
				/* return true if contact with "object" established */
				if(get L grpper target  >=1.0 &&  get R grpper target  >=1.0 )
				{
					
					auto  gripper = ctl.grippers["l_gripper"].get();
		    		gripper->setTargetQ({closeGrippers});

		    		gripper = ctl.grippers["r_gripper"].get();
		    		gripper->setTargetQ({closeGrippers});

		    		return true;
				}
				else
				{
					output("gipper not closed, try again");
					return false;
				}
				output("contact not fixed, try again");
				return false;
			}
		    return false;
		}

	} // namespace states

} // namespace mc_torquing_controller
