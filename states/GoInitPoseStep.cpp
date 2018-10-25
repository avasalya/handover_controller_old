#include "GoInitPoseStep.h"

namespace mc_handover
{
	namespace states {

		void GoInitPoseStep::configure(const mc_rtc::Configuration & config)
		{
			// cout << "config" << endl;

			threshold_eval_ = config("threshold_eval", 0.005);
			threshold_speed_ = config("threshold_speed", 0.005);
			stiffness_ = config("stiffness", 2.);
			weight_ = config("weight", 1500.);
			config("closeGrippers", closeGrippers);
		}

		void GoInitPoseStep::start(mc_control::fsm::Controller & controller)
		{	
			
			// cout << "start" << endl;

			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			// ctl.set_joint_pos("HEAD_JOINT1",  -0.7); //+ve to move head down

			// ctl.relEfTaskL.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), "", 1.0,1e3));
			// ctl.relEfTaskR.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), "BODY", 1.0,1e3));
			// cout << "ef pos R" << ctl.relEfTaskR->get_ef_pose().translation().transpose() << endl;
			// cout << "ef pos L" << ctl.relEfTaskL->get_ef_pose().translation().transpose() << endl;

			ctl.efTaskR.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), 1.0, 1e3));
			ctl.efTaskL.reset(new mc_tasks::EndEffectorTask("LARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), 1.0, 1e3));
			
			// ctl.oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK6", ctl.robots(), ctl.robots().robotIndex(),1.0,1e2));
			// ctl.oriTaskL.reset(new mc_tasks::OrientationTask("LARM_LINK6", ctl.robots(), ctl.robots().robotIndex(),1.0,1e2));

			/* CHEST */
			chestPosTask.reset(new mc_tasks::PositionTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			chestOriTask.reset(new mc_tasks::OrientationTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));



			/*close grippers for safety*/
			auto  gripperL = ctl.grippers["l_gripper"].get();
			auto  gripperR = ctl.grippers["r_gripper"].get();

			gripperL->setTargetQ({closeGrippers});
			gripperR->setTargetQ({closeGrippers});


			// Eigen::Matrix3d ori; 
			// ori = chestOriTask->orientation();
			// cout <<"CHEST_LINK1 orientation\n"<< ori << endl;

			// Eigen::VectorXd dimW(3);
			// dimW << 1., 1., 1.;
			// chestPosTask->dimWeight(dimW);

			
		}

		bool GoInitPoseStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			cout << "moving to initial pose" <<endl;			

			// BodyPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("BODY")];

			ctl.solver().addTask(chestPosTask);
			ctl.solver().addTask(chestOriTask);


			auto ltHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")];
			auto rtHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")];

			cout << "ltHand " << ltHand.translation().transpose() 
			<< "\nrtHand " << rtHand.translation().transpose() << endl;

			// ctl.solver().addTask(ctl.oriTaskL);
			// ctl.solver().addTask(ctl.oriTaskR);

			initPosL <<  0.3, 0.3,1.1;
			ctl.efTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX((M_PI/180)*90), initPosL));

			initPosR <<  0.3, -0.3,1.1; //0.30, -0.35, 0.3;
			ctl.efTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90), initPosR));

			// ctl.solver().addTask(ctl.efTaskL);
			// ctl.solver().addTask(ctl.efTaskR);



			// initPosL <<  0.30, 0.35, 0.3;
			// auto rotL = sva::RotY(-(M_PI/180)*90);
			// //sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyPosW.rotation();
			// ctl.relEfTaskL->set_ef_pose(sva::PTransformd(rotL, initPosL));
			

			// initPosR <<  0.30, -0.35, 0.3;
			// auto rotR = sva::RotY(-(M_PI/180)*90);
			// //sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyPosW.rotation();
			// ctl.relEfTaskR->set_ef_pose(sva::PTransformd(rotR, initPosR));


			// ctl.solver().addTask(ctl.relEfTaskL);
			// ctl.solver().addTask(ctl.relEfTaskR);

			// cout << "ef pos R" << ctl.relEfTaskR->get_ef_pose().translation().transpose() << endl;
			// cout << "ef pos L" << ctl.relEfTaskL->get_ef_pose().translation().transpose() << endl;



			output("OK");

			return true;
		}

	} // namespace states

} // namespace mc_handover



			// Eigen::Vector3d pos = robot().mbc().bodyPosW[robot().bodyIndexByName("RARM_LINK7")].translation();
			// Eigen::Vector3d vel = robot().mbc().bodyVelW[robot().bodyIndexByName("RARM_LINK7")].linear();


			// if(ctl.relEfTaskL->eval().norm() < threshold_eval_ &&
			//  	ctl.relEfTaskR->eval().norm() < threshold_eval_ &&
			// 	ctl.relEfTaskL->speed().norm() < threshold_speed_ &&
			// 	ctl.relEfTaskR->speed().norm() < threshold_speed_)
			// {
			// 	ctl.solver().removeTask(ctl.relEfTaskL);
			// 	ctl.solver().removeTask(ctl.relEfTaskR);
			// 	ctl.solver().removeTask(ctl.oriTaskL);
			// 	ctl.solver().removeTask(ctl.oriTaskR);
			// }	




			// /* gripper control */
			// ctl.gui()->addElement(
			// {"HandoverElements"},
			// mc_rtc::gui::Button("open_Grippers", [&ctl]() { std::string msg = "openGrippers"; 
			// 	ctl.read_msg(msg); }),
			// mc_rtc::gui::Button("close_Grippers",[&ctl]() { std::string msg = "closeGrippers"; ctl.read_msg(msg); }),
			// mc_rtc::gui::Button("open_Right_Gripper",[&ctl]() { std::string msg = "openGripperR"; ctl.read_msg(msg); }),
			// mc_rtc::gui::Button("close_Right_Gripper",[&ctl]() { std::string msg = "closeGripperR"; ctl.read_msg(msg); }),
			// mc_rtc::gui::Button("open_Left_Gripper",[&ctl]() { std::string msg = "openGripperL"; ctl.read_msg(msg); }),
			// mc_rtc::gui::Button("close_Left_Gripper",[&ctl]() { std::string msg = "closeGripperL"; ctl.read_msg(msg); })
			// );
