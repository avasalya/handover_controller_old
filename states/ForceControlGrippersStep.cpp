
#include "ForceControlGrippersStep.h"

namespace mc_handover
{
	namespace states {

		void ForceControlGrippersStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config" << endl;

			leftHandWrenchTh   = config("leftHandWrenchTh");
			rightHandWrenchTh   = config("rightHandWrenchTh");

			leftHandWrenchDir  = config("leftHandWrenchDir");
			rightHandWrenchDir = config("rightHandWrenchDir");
			
		}
		void ForceControlGrippersStep::start(mc_control::fsm::Controller & controller)
		{
			cout << "start -- ForceControlGrippersStep " << endl;
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);


			if(ctl.addGUIonlyOnce)
			{

				ctl.addGUIonlyOnce = false;

				/* remove  Relative EF Tasks */
				ctl.solver().removeTask(ctl.relEfTaskL);
				ctl.solver().removeTask(ctl.relEfTaskR);
				ctl.solver().removeTask(ctl.oriTaskL);
				ctl.solver().removeTask(ctl.oriTaskR);


				/*add com task -- position it lower and bit backward */
				comTask = std::make_shared<mc_tasks::CoMTask>
					(ctl.robots(), ctl.robots().robotIndex(), 2.0, 10000.);
				// comTask->dimWeight(Eigen::Vector3d(1., 1., 1.));
				ctl.solver().addTask(comTask);
			
				/* handover gui elements */
				ctl.gui()->addElement({"HandoverElements"},

					mc_rtc::gui::ArrayInput("Move Com Pos", {"x", "y", "z"},
						[this]() { return move; },
						[this](const Eigen::Vector3d v) { move = v;
						cout << " com pos set to:\n" << initialCom + move << endl;}),

					mc_rtc::gui::ArrayInput("change Left Hand Wrench threshold",
						{"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"},
						[this]() { return leftHandWrenchTh; },
						[this](const Eigen::Vector6d lw) { leftHandWrenchTh = lw;
						cout<<"Left Hand Wrench threshold set to:\n" <<leftHandWrenchTh<<endl;}),

					mc_rtc::gui::ArrayInput("change Right Hand Wrench threshold",
						{"Fx", "Fy", "Fz", "Tx", "Ty", "Tz"},
						[this]() { return rightHandWrenchTh; },
						[this](const Eigen::Vector6d rw) { rightHandWrenchTh = rw;
						cout<<"Right Hand Wrench threshold set to:\n" <<rightHandWrenchTh<<endl;})
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

			/* set com pose */
			target = initialCom + move;
			comTask->com(target);

			// bool isTrue = true;

			/* may be make a "for loop" for same conditions */

			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[0]) > leftHandWrenchTh[0]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[0]) > rightHandWrenchTh[0])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				// 	cout << "opening grippers: Fx Forces crossed thresholds " << endl;
				return false;
			}

			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[1]) > leftHandWrenchTh[1]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[1]) > rightHandWrenchTh[1])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				// cout << "opening grippers: Fy Forces crossed thresholds " << endl;
				return false;
			}

			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[2]) > leftHandWrenchTh[2]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[2] > rightHandWrenchTh[2]))
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				// cout << "opening grippers: Fz Forces crossed thresholds " << endl;
				return false;
			}





			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[0]) > leftHandWrenchTh[0]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[0]) < rightHandWrenchTh[0])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				// 	cout << "opening grippers: lFx > threshold, rFx < threshold " << endl;
				return false;
			}

			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[1]) > leftHandWrenchTh[1]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[1]) < rightHandWrenchTh[1])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				// cout << "opening grippers: lFy > threshold, rFy < threshold " << endl;
				return false;
			}

			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[2]) > leftHandWrenchTh[2]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[2] < rightHandWrenchTh[2]))
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				// cout << "opening grippers: lFz > threshold, rFz < threshold " << endl;
				return false;
			}




			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[0]) < leftHandWrenchTh[0]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[0]) > rightHandWrenchTh[0])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				// cout << "opening grippers: lFx < threshold, rFx > threshold " << endl;
				return false;
			}

			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[1]) < leftHandWrenchTh[1]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[1]) > rightHandWrenchTh[1])
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				// cout << "opening grippers: lFy < threshold, rFy > threshold " << endl;
				return false;
			}

			if(fabs(ctl.wrenches.at("LeftHandForceSensor").force()[2]) < leftHandWrenchTh[2]	
				&& fabs(ctl.wrenches.at("RightHandForceSensor").force()[2] > rightHandWrenchTh[2]))
			{
				auto  gripper = ctl.grippers["l_gripper"].get();
				gripper->setTargetQ({openGrippers});

				gripper = ctl.grippers["r_gripper"].get();
				gripper->setTargetQ({openGrippers});

				// cout << "opening grippers: lFz < threshold, rFz > threshold " << endl;
				return false;
	
			}

			// cout << "left hand wrenches " << ctl.wrenches.at("LeftHandForceSensor") <<  endl;
			// cout << "right hand wrenches " << ctl.wrenches.at("RightHandForceSensor") <<  endl;

			output("Repeat");
			if(leftHandWrenchTh[0] == 0)
			{	
				return true;
				cout << "repeat this state " << endl;
 			}
			// return false;
		}

	} // namespace states

} // namespace mc_torquing_controller
