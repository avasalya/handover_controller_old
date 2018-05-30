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
			
			ctl.solver().setContacts({
	        {ctl.robots(), 0, 1, "l_gripper", "handoverPipe"},
	        {ctl.robots(), 0, 1, "r_gripper", "handoverPipe"}
	        });
			
			hasContacts = ctl.hasContact({ctl.solver().robot(contact->r1Index()).name(), ctl.solver().robot(contact->r2Index()).name(),contact->r1Surface()->name(), contact->r2Surface()->name()});
		}

		bool CloseGrippersStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			if(hasContacts) //(ctl.checkContact->isFixed())
			{
				auto  gripperL = ctl.grippers["l_gripper"].get();
				auto  gripperR = ctl.grippers["r_gripper"].get();
				
				/* return true if contact with "object" established */
				if( gripperL->curPosition()[0] >= 1.0 && gripperR->curPosition()[0] >= 1.0 )
				{					
		    		gripperL->setTargetQ({closeGrippers});
		    		gripperR->setTargetQ({closeGrippers});

		    		output("contact with the object established");
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
