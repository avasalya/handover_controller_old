
#include "ForceControlGrippersStep.h"

namespace mc_handover
{
	namespace states {

		void ForceControlGrippersStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config" << endl;

			handsWrenchTh		= config("handsWrenchTh");
			handsWrenchDir		= config("handsWrenchDir");
			
		}
		void ForceControlGrippersStep::start(mc_control::fsm::Controller & controller)
		{
			cout << "start -- ForceControlGrippersStep " << endl;
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
        

			/* handover gui elements */
			ctl.gui()->addElement({"FSM", "HandoverElements"},

				// mc_rtc::gui::NumberSlider("move COM pos",
				// 	[this](){ return move[0]; },
				// 	[this](double v) { move[0] = v; },
				// 	-10.0, 10.0),

				mc_rtc::gui::Button("publish_current_wrench", [&ctl]() {  
				std::cout << "left hand wrench:: Torques, Forces " <<
				ctl.wrenches.at("LeftHandForceSensor")/*.force().transpose()*/ << endl;
				std::cout << "right hand wrench:: Torques, Forces " <<
				ctl.wrenches.at("RightHandForceSensor")/*.force().transpose()*/ << endl;
				}),

				mc_rtc::gui::ArrayInput("Move Com Pos", {"x", "y", "z"},
					[this]() { return move; },
					[this](const Eigen::Vector3d v) { move = v;
					cout << " com pos set to:\n" << initialCom + move << endl;}),

				mc_rtc::gui::ArrayInput("change Hands Wrench thresholds",
					{"lFx", "lFy", "lFz", "lTx", "lTy", "lTz", "rFx", "rFy", "rFz", "rTx", "rTy", "rTz"},
					[this]() { return handsWrenchTh; },
					[this](const std::vector<double> lrw){handsWrenchTh = lrw;
					cout<<" Hands Wrench thresholds set to:\n";
					for (auto i: handsWrenchTh)
					cout << i << endl;
				})
			);
			
			/*add com task -- position it lower and bit backward */
			comTask = std::make_shared<mc_tasks::CoMTask>
				(ctl.robots(), ctl.robots().robotIndex(), 2., 10000.);
			// comTask->dimWeight(Eigen::Vector3d(1., 1., 1.));
			ctl.solver().addTask(comTask);
			initialCom = rbd::computeCoM(ctl.robot().mb(),ctl.robot().mbc());
						
    		cout << "initial_com X " << endl << initialCom[0] << endl;
    		cout << "initial_com Y " << endl << initialCom[1] << endl;
    		cout << "initial_com Z " << endl << initialCom[2] << endl;
		}




		bool ForceControlGrippersStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/* set com pose */
			target = initialCom + move;
			comTask->com(target);
      

			leftHandWrenchTh.force()  <<  handsWrenchTh.at(0), handsWrenchTh.at(1), handsWrenchTh.at(2);
			leftHandWrenchTh.couple() <<  handsWrenchTh.at(3), handsWrenchTh.at(4), handsWrenchTh.at(5);
    
			rightHandWrenchTh.force()  << handsWrenchTh.at(6), handsWrenchTh.at(7), handsWrenchTh.at(8);
			rightHandWrenchTh.couple() << handsWrenchTh.at(9), handsWrenchTh.at(10), handsWrenchTh.at(11);

			
			if(ctl.runOnce
				&& fabs(ctl.wrenches.at("LeftHandForceSensor").force()[0]) > leftHandWrenchTh.force()[0]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[0]) > rightHandWrenchTh.force()[0])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: Fx Forces crossed thresholds " << endl;
				ctl.publishWrench(); output("Repeat"); return true;// always return true to repeat state
			}

			if(ctl.runOnce
				&& fabs(ctl.wrenches.at("LeftHandForceSensor").force()[1]) > leftHandWrenchTh.force()[1]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[1]) > rightHandWrenchTh.force()[1])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: Fy Forces crossed thresholds " << endl;
				ctl.publishWrench(); output("Repeat"); return true;// always return true to repeat state	
			}

			if(ctl.runOnce
				&& fabs(ctl.wrenches.at("LeftHandForceSensor").force()[2]) > leftHandWrenchTh.force()[2]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[2] > rightHandWrenchTh.force()[2]))
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: Fz Forces crossed thresholds " << endl;
				ctl.publishWrench(); output("Repeat"); return true;// always return true to repeat state 
			}




			if(ctl.runOnce
			&& fabs(ctl.wrenches.at("LeftHandForceSensor").force()[0])  > leftHandWrenchTh.force()[0]	
			&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[0]) < rightHandWrenchTh.force()[0])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening left gripper: lFx > threshold, rFx < threshold " << endl;

				ctl.publishWrench(); output("Repeat"); return true;// always return true to repeat state 
			}

			if(ctl.runOnce
			&& fabs(ctl.wrenches.at("LeftHandForceSensor").force()[1])  > leftHandWrenchTh.force()[1]	
			&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[1]) < rightHandWrenchTh.force()[1])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening left gripper: lFy > threshold, rFy < threshold " << endl;

				ctl.publishWrench(); output("Repeat"); return true;// always return true to repeat state 
			}

			if(ctl.runOnce
			&& fabs(ctl.wrenches.at("LeftHandForceSensor").force()[2]) > leftHandWrenchTh.force()[2]	
			&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[2] < rightHandWrenchTh.force()[2]))
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening left gripper: lFz > threshold, rFz < threshold " << endl;
				
				ctl.publishWrench(); output("Repeat"); return true;// always return true to repeat state 
			}




			if(ctl.runOnce
			&& fabs(ctl.wrenches.at("LeftHandForceSensor").force()[0])  < leftHandWrenchTh.force()[0]	
			&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[0]) > rightHandWrenchTh.force()[0])
			{
				auto  gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening right gripper: lFx < threshold, rFx > threshold " << endl;

				ctl.publishWrench(); output("Repeat"); return true;// always return true to repeat state 
			}

			if(ctl.runOnce
			&& fabs(ctl.wrenches.at("LeftHandForceSensor").force()[1])  < leftHandWrenchTh.force()[1]	
			&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[1]) > rightHandWrenchTh.force()[1])
			{
				auto  gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening right gripper: lFy < threshold, rFy > threshold " << endl;

				ctl.publishWrench(); output("Repeat"); return true;// always return true to repeat state 
			}

			if(ctl.runOnce
			&& fabs(ctl.wrenches.at("LeftHandForceSensor").force()[2]) < leftHandWrenchTh.force()[2]	
			&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[2] > rightHandWrenchTh.force()[2]))
			{
				auto  gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening right gripper: lFz < threshold, rFz > threshold " << endl;
				
				ctl.publishWrench(); output("Repeat"); return true;// always return true to repeat state 
			}


 			else
 			{	
 				ctl.runOnce = true;
	 			output("Repeat");
				return false;	// to repeat within state
 			}	

		}


		void ForceControlGrippersStep::teardown(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
			
			ctl.solver().removeTask(comTask);

			ctl.gui()->removeElement({"FSM", "HandoverElements"},"publish_current_wrench");		
			ctl.gui()->removeElement({"FSM", "HandoverElements"}, "Move Com Pos");
			ctl.gui()->removeElement({"FSM", "HandoverElements"}, "change Hands Wrench thresholds");

		}


	} // namespace states

} // namespace mc_torquing_controller



// cout << leftHandWrenchTh.force()[0] << " "<< leftHandWrenchTh.force()[1] << " "<< leftHandWrenchTh.force()[2] << " "<< endl;
// cout  << handsWrenchTh[0] << " " <<  handsWrenchTh[1] << " " <<  handsWrenchTh[2] << endl;
// cout << leftHandWrenchTh.couple()[0] << " "<< leftHandWrenchTh.couple()[1] << " "<< leftHandWrenchTh.couple()[2] << " "<< endl;
// cout  << handsWrenchTh[3] << " " <<  handsWrenchTh[4] << " " <<  handsWrenchTh[5] << endl;
// cout << rightHandWrenchTh.force()[0] << " "<< rightHandWrenchTh.force()[1] << " "<< rightHandWrenchTh.force()[2] << " "<< endl;
// cout << handsWrenchTh[6] << " " <<  handsWrenchTh[7] << " " <<  handsWrenchTh[8] << endl;
// cout << rightHandWrenchTh.couple()[0] << " "<< rightHandWrenchTh.couple()[1] << " "<< rightHandWrenchTh.couple()[2] << " "<< endl;
// cout << handsWrenchTh[9] << " " <<  handsWrenchTh[10] << " " <<  handsWrenchTh[11] << endl;
