
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

				/*add com task -- position it lower and bit backward */
				comTask = std::make_shared<mc_tasks::CoMTask>
												(ctl.robots(), ctl.robots().robotIndex(), 2.0, 100.);
				ctl.solver().addTask(comTask);

				// comTask->dimWeight(Eigen::Vector3d(1., 1., 1.));    		
			
				/* handover gui elements */ 			
				ctl.gui()->addElement({"HandoverElements"},


					mc_rtc::gui::NumberInput("move COM X",
											[this]() { return move[0]; },
											[this](double comX){ move[0] = comX; 
											cout << " com X set to : " << initialCom[0] + move[0] << endl;}),

					mc_rtc::gui::NumberInput("move COM Y",
											[this]() { return move[1]; },
											[this](double comY){ move[1] = comY; 
											cout << " com Y set to : " << initialCom[1] + move[1] << endl;}),

					mc_rtc::gui::NumberInput("move COM Z",
											[this]() { return move[2]; },
											[this](double comZ){ move[2] = comZ; 
											cout << " com Z set to : " << initialCom[2] + move[2] << endl;}),




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
			
			initialCom = rbd::computeCoM(ctl.robot().mb(),ctl.robot().mbc());
			
    		cout << "initial_com X " << endl << initialCom[0] << endl;
    		cout << "initial_com Y " << endl << initialCom[1] << endl;
    		cout << "initial_com Z " << endl << initialCom[2] << endl;
		}




		bool ForceControlGrippersStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
						
    		
    		target = initialCom + move;
    		comTask->com(target);
			

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
				// cout << "final_com X " << endl << target[0] << endl;
				// cout << "final_com Y " << endl << target[1] << endl;
				// cout << "final_com Z " << endl << target[2] << endl;

				return true;
			}
	    	return false;
		}

	} // namespace states

} // namespace mc_torquing_controller