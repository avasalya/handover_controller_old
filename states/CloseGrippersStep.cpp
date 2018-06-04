#include "CloseGrippersStep.h"


namespace mc_handover
{
	namespace states {

		void CloseGrippersStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config " <<endl;
			config("closeGrippers", closeGrippers);
		}

		void CloseGrippersStep::start(mc_control::fsm::Controller & controller)
		{				

			cout <<"start -- closing grippers" << endl;

			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.addContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});
			ctl.addContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});

			ctl.solver().setContacts({
              mc_rbdyn::Contact(ctl.robots(), "LFullSole", "AllGround"),
              mc_rbdyn::Contact(ctl.robots(), "RFullSole", "AllGround"),
              mc_rbdyn::Contact(ctl.robots(), "Butthock", "AllGround"),
              mc_rbdyn::Contact(ctl.robots(), "LowerBack","AllGround")
              });
		}

		bool CloseGrippersStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
							
				cout << "run " <<endl;				
				auto  gripperL = ctl.grippers["l_gripper"].get();
				auto  gripperR = ctl.grippers["r_gripper"].get();
				
				/* return true if contact with "object" established */
				if( (gripperL->curPosition()[0] >= 0.5) && (gripperR->curPosition()[0] >= 0.5) )
				{					
		    		gripperL->setTargetQ({closeGrippers});
		    		gripperR->setTargetQ({closeGrippers});

		    		cout <<"contact with the object established" <<endl;
		    		output("OK");
		    		return true;
				}
				else
				{
					cout <<"gipper not closed, trying again" <<endl;
		    		output("Repeat");
					return false;
				}			
			
		    return false;
		}

		// void CloseGrippersStep::teardown(mc_control::fsm::Controller& controller)
		// {

		// 	auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
							
		// 	cout << "teardown " <<endl;

		// 	ctl.removeContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});
		// 	ctl.removeContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});

		// }


	} // namespace states

} // namespace mc_torquing_controller
