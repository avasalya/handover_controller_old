#include "OpenGrippersStep.h"

namespace mc_handover
{
	namespace states {

		void OpenGrippersStep::configure(const mc_rtc::Configuration & config)
		{	
			config("openGrippers", openGrippers);
		}

		void OpenGrippersStep::start(mc_control::fsm::Controller & controller)
		{	
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			target = true;

		// 	// auto  gripper = ctl.grippers["l_gripper"].get();
		// 	// for(int k=0; k<=10;k++)
		// 	// {
		// 	// 	gripper->setTargetQ({openGrippers*k*0.01});
		// 	// 	// gripper->setTargetOpening(1.8);

		// 	// 	cout << "openGrippers*k*0.01 " << openGrippers*k*0.01 << endl;
		// 	// 	if(k==10)
		// 	// 	{
		// 	// 		gripper->setTargetQ({1-openGrippers*k*0.01});
		// 	// 		LOG_WARNING("reverse")
		// 	// 	}
		// 	// }

		// std::vector<double> Ps = gripper->curPosition();
		// double cur = Ps.at(0);

		// 	// gripper = ctl.grippers["r_gripper"].get();
		// 	// gripper->setTargetQ({openGrippers});
		}

		bool OpenGrippersStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			auto gripperL = ctl.grippers["l_gripper"].get();
			auto gripperR = ctl.grippers["r_gripper"].get();

			if(target)
			{
				for(int k=0; k<=10;k++)
				{
					gripperL->setTargetQ({sin(k*0.1)});
					// cout << k <<" L sin " <<sin(k*0.1) << endl;
				}

				for(int k=10; k>=0;k--)
				{
					gripperR->setTargetQ({sin(k*0.1)});
					// cout << k <<" R sin " <<sin(k*0.1) << endl;
				}

				target = false;
				
				cout << gripperL->curPosition()[0] <<endl; //0.8
				cout << gripperR->curPosition()[0] <<endl; //0.0
			}
			// else
			// {
			// 	if( (gripperL->curPosition()[0] > 0.5) && (gripperR->curPosition()[0] < 0.5) )
			// 	{
			// 		for(int k=0; k<=10;k++)
			// 		{
			// 			gripperL->setTargetQ({-sin(1-(k*0.1))});
			// 		}
					
			// 		for(int k=10; k>=0;k--)
			// 		{
			// 			gripperR->setTargetQ({-sin(1+(k*0.1))});
			// 		}					
	
			// 		target = true;
	
			// 		cout <<"L " << gripperL->curPosition()[0] <<endl; //0.5
			// 		cout <<"R " << gripperR->curPosition()[0] <<endl; //0.0
			// 	}
			// }
			
			output("OK");
			return false;


		}

	} // namespace states

} // namespace mc_handover



		// if( (gripperL->curPosition()[0] < 0.5) && (gripperR->curPosition()[0] < 0.5) )
		// {
		// 	output("OK");
		// 	return true;
		// }
		// else
		// {
		// 	output("Repeat");
		// 	return true;
		// }
