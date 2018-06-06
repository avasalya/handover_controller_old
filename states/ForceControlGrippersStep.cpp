
#include "ForceControlGrippersStep.h"

namespace mc_handover
{
	namespace states {

		void ForceControlGrippersStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config" << endl;


			leftHandForcesTh   = config("leftHandForcesTh");
			leftHandForcesDir  = config("leftHandForcesDir");

			leftHandTorquesTh  = config("leftHandTorquesTh");
			leftHandTorquesDir = config("leftHandTorquesDir");

			rightHandForcesTh  = config("rightHandForcesTh");
			rightHandForcesDir = config("rightHandForcesDir");

			rightHandTorquesTh  = config("rightHandTorquesTh");			
			rightHandTorquesDir = config("rightHandTorquesDir");
			
			
		}
		void ForceControlGrippersStep::start(mc_control::fsm::Controller & controller)
		{	

			cout << "start -- ForceControlGrippersStep " << endl;
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/* remove  Relative EF Tasks */
			ctl.solver().removeTask(ctl.relEfTaskL);
			ctl.solver().removeTask(ctl.relEfTaskR);
			ctl.solver().removeTask(ctl.oriTaskL);
			ctl.solver().removeTask(ctl.oriTaskR);

			if(ctl.addGUIonlyOnce)
			{			
				ctl.addGUIonlyOnce = false;

				/* handover gui elements */ 				
				ctl.gui()->addElement({"HandoverElements"},
			
					mc_rtc::gui::NumberInput("leftHand_ForceX_Th",
											[this]() { return leftHandForcesTh[0]; },
											[this](double lFx){ leftHandForcesTh[0] = lFx; 
											cout << "lFx set to : " << leftHandForcesTh[0] << endl;}),
	
					mc_rtc::gui::NumberInput("leftHand_ForceY_Th",
											[this]() { return leftHandForcesTh[1]; },
											[this](double lFy){ leftHandForcesTh[1] = lFy; 
											cout << "lFy set to : " << leftHandForcesTh[1] << endl;}),
	
					mc_rtc::gui::NumberInput("leftHand_ForceZ_Th",
											[this]() { return leftHandForcesTh[2]; },
											[this](double lFz){ leftHandForcesTh[2] = lFz;
											cout << "lFz set to : " << leftHandForcesTh[2] << endl;}),
	
	
					mc_rtc::gui::NumberInput("rightHand_ForceX_Th",
											[this]() { return rightHandForcesTh[0]; },
											[this](double rFx){ rightHandForcesTh[0] = rFx;
											cout << "rFx set to : " << rightHandForcesTh[0] << endl;}),
	
					mc_rtc::gui::NumberInput("rightHand_ForceY_Th",
											[this]() { return rightHandForcesTh[1]; },
											[this](double rFy){ rightHandForcesTh[1] = rFy;
											cout << "rFy set to : " << rightHandForcesTh[1] << endl;}),
	
					mc_rtc::gui::NumberInput("rightHand_ForceZ_Th",
											[this]() { return rightHandForcesTh[2]; },
											[this](double rFz){ rightHandForcesTh[2] = rFz;
											cout << "rFz set to : " << rightHandForcesTh[2] << endl;}),
	
	
	
					
					mc_rtc::gui::NumberInput("leftHand_TorqueX_Th",
											[this]() { return leftHandTorquesTh[0]; },
											[this](double lTx){ leftHandTorquesTh[0] = lTx;
											cout << "lTx set to : " << leftHandTorquesTh[0] << endl;}),
	
					mc_rtc::gui::NumberInput("leftHand_TorqueY_Th",
											[this]() { return leftHandTorquesTh[1]; },
											[this](double lTy){ leftHandTorquesTh[1] = lTy;
											cout << "lTy set to : " << leftHandTorquesTh[1] << endl;}),
	
					mc_rtc::gui::NumberInput("leftHand_TorqueZ_Th",
											[this]() { return leftHandTorquesTh[2]; },
											[this](double lTz){ leftHandTorquesTh[2] = lTz;
											cout << "lTz set to : " << leftHandTorquesTh[2] << endl;}),
	
	
					mc_rtc::gui::NumberInput("rightHand_TorqueX_Th",
											[this]() { return rightHandTorquesTh[0]; },
											[this](double rTx){ rightHandTorquesTh[0] = rTx;
											cout << "rTx set to : " << rightHandTorquesTh[0] << endl;}),
	
					mc_rtc::gui::NumberInput("rightHand_TorqueY_Th",
											[this]() { return rightHandTorquesTh[1]; },
											[this](double rTy){ rightHandTorquesTh[1] = rTy;
											cout << "rTy set to : " << rightHandTorquesTh[1] << endl;}),
	
					mc_rtc::gui::NumberInput("rightHand_TorqueZ_Th",
											[this]() { return rightHandTorquesTh[2]; },
											[this](double rTz){ rightHandTorquesTh[2] = rTz;
											cout << "rTz set to : " << rightHandTorquesTh[2] << endl;})
			
				);
			}
		}

		bool ForceControlGrippersStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
			
			
			
			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[0]) > leftHandForcesTh[0]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[0]) > rightHandForcesTh[0])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: Fx Forces crossed thresholds " << endl;
				cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
				cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;
			}

			else if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[1]) > leftHandForcesTh[1]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[1]) > rightHandForcesTh[1])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: Fy Forces crossed thresholds " << endl;
				cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
				cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;
			}

			else if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[2]) > leftHandForcesTh[2]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[2] > rightHandForcesTh[2]))
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: Fz Forces crossed thresholds " << endl;
				cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
				cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;
			}




			else if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[0]) > leftHandForcesTh[0]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[0]) < rightHandForcesTh[0])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: lFx > threshold, rFx < threshold " << endl;
				cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
				cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;
			}

			else if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[1]) > leftHandForcesTh[1]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[1]) < rightHandForcesTh[1])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: lFy > threshold, rFy < threshold " << endl;
				cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
				cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;
			}

			else if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[2]) > leftHandForcesTh[2]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[2] < rightHandForcesTh[2]))
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: lFz > threshold, rFz < threshold " << endl;
				cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
				cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;
			}




			else if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[0]) < leftHandForcesTh[0]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[0]) > rightHandForcesTh[0])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: lFx < threshold, rFx > threshold " << endl;
				cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
				cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;
			}

			else if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[1]) < leftHandForcesTh[1]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[1]) > rightHandForcesTh[1])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: lFy < threshold, rFy > threshold " << endl;
				cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
				cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;
			}

			else if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[2]) < leftHandForcesTh[2]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[2] > rightHandForcesTh[2]))
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				cout << "opening grippers: lFz < threshold, rFz > threshold " << endl;
			    cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
				cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;

			}

			output("Repeat");
			if(leftHandForcesTh[0] == 0)
			{	
				return true;				
			}
	    	return false;
		}

	} // namespace states

} // namespace mc_torquing_controller





// mc_rtc::gui::Button("open_Grippers", [&ctl]() { std::string msg = "openGrippers"; ctl.read_msg(msg); 
// std::cout << "at grippers opening: right hand wrench:: Torques, Forces " << ctl.wrenches.at("RightHandForceSensor")/*.force().transpose()*/ << endl;
// std::cout << "at grippers opening: left hand wrench:: Torques, Forces " << ctl.wrenches.at("LeftHandForceSensor")/*.force().transpose()*/ << endl;
// }),

// mc_rtc::gui::Button("close_Grippers",[&ctl]() { std::string msg = "closeGrippers"; ctl.read_msg(msg); 
// std::cout << "at grippers closing: right hand wrench:: Torques, Forces " << ctl.wrenches.at("RightHandForceSensor")/*.force().transpose()*/ << endl;
// std::cout << "at grippers closing: left hand wrench:: Torques, Forces " << ctl.wrenches.at("LeftHandForceSensor")/*.force().transpose()*/ << endl;
// }),

// mc_rtc::gui::Button("open_Right_Gripper",[&ctl]() { std::string msg = "openGripperR"; ctl.read_msg(msg); 
// std::cout << "at right gripper opening: right hand wrench:: Torques, Forces " << ctl.wrenches.at("RightHandForceSensor")/*.force().transpose()*/ << endl;
// }),

// mc_rtc::gui::Button("close_Right_Gripper",[&ctl]() { std::string msg = "closeGripperR"; ctl.read_msg(msg); 
// std::cout << "at right gripper closing: right hand wrench:: Torques, Forces " << ctl.wrenches.at("RightHandForceSensor")/*.force().transpose()*/ << endl;          
// }),

// mc_rtc::gui::Button("open_Left_Gripper",[&ctl]() { std::string msg = "openGripperL"; ctl.read_msg(msg); 
// std::cout << "at left gripper opening: left hand wrench:: Torques, Forces " << ctl.wrenches.at("LeftHandForceSensor")/*.force().transpose()*/ << endl;
// }),

// mc_rtc::gui::Button("close_Left_Gripper",[&ctl]() { std::string msg = "closeGripperL"; ctl.read_msg(msg); 
// std::cout << "at left gripper closing: left hand wrench:: Torques, Forces " << ctl.wrenches.at("LeftHandForceSensor")/*.force().transpose()*/ << endl;
// }),








		
			// ctl.wrenches.at("RightHandForceSensor").force() = Eigen::Vector3d (4,5,6);

			// ctl.wrenches.at("LeftHandForceSensor").force() = Eigen::Vector3d (1,2,3);


			// ctl.wrenches["RightHandForceSensor"] =
			//         ctl.robots().robot().forceSensor("RightHandForceSensor").wrench();          
			// ctl.wrenchRt = ctl.wrenches.at("RightHandForceSensor");
			// ctl.wrenches.at("RightHandForceSensor").couple() << ctl.wrenchRt.couple()[2], ctl.wrenchRt.couple()[1], -ctl.wrenchRt.couple()[0];
			// ctl.wrenches.at("RightHandForceSensor").force() << ctl.wrenchRt.force()[2], ctl.wrenchRt.force()[1], -ctl.wrenchRt.force()[0];
			
			// cout <<"right hand force sensor" << ctl.wrenches.at("RightHandForceSensor").force().transpose() << endl;



			// ctl.wrenches["LeftHandForceSensor"] =
			//         ctl.robots().robot().forceSensor("LeftHandForceSensor").wrench();          
			// ctl.wrenchRt = ctl.wrenches.at("LeftHandForceSensor");
			// ctl.wrenches.at("LeftHandForceSensor").couple() << ctl.wrenchRt.couple()[2], ctl.wrenchRt.couple()[1], -ctl.wrenchRt.couple()[0];
			// ctl.wrenches.at("LeftHandForceSensor").force() << ctl.wrenchRt.force()[2], ctl.wrenchRt.force()[1], -ctl.wrenchRt.force()[0];

			// cout <<"left hand force sensor" << ctl.wrenches.at("LeftHandForceSensor").force().transpose() << endl;












		// void ForceControlGrippersStep::teardown(mc_control::fsm::Controller & controller)
		// {
		// 	auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
		
		// 	cout << "teardown" << endl;

		// 	// ctl.gui()->removeElement({"HandoverElements"}, "openGrippers");
		// 	// ctl.gui()->removeElement({"HandoverElements"}, "closeGrippers");
			
		// }
