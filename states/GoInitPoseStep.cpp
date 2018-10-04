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
		}

		void GoInitPoseStep::start(mc_control::fsm::Controller & controller)
		{	
			
			// cout << "start" << endl;

			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			// ctl.set_joint_pos("HEAD_JOINT1",  -0.7); //+ve to move head down

			// ctl.relEfTaskR.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK7", ctl.robots(), ctl.robots().robotIndex(), "", 1.0,1e3));
			// ctl.relEfTaskL.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK7", ctl.robots(), ctl.robots().robotIndex(), "", 1.0,1e3));

			ctl.efTaskR.reset(new mc_tasks::EndEffectorTask("RARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), 1.0, 1e3));
			ctl.efTaskL.reset(new mc_tasks::EndEffectorTask("LARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), 1.0, 1e3));
			
			ctl.oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK6", ctl.robots(), ctl.robots().robotIndex(),1.0,1e2));
			ctl.oriTaskL.reset(new mc_tasks::OrientationTask("LARM_LINK6", ctl.robots(), ctl.robots().robotIndex(),1.0,1e2));

			/* CHEST */
			chestPosTask.reset(new mc_tasks::PositionTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			chestOriTask.reset(new mc_tasks::OrientationTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));


			Eigen::Matrix3d ori; 
			ori = chestOriTask->orientation();
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


			auto ltHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")];
			auto rtHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")];

			cout << "ltHand " << ltHand.translation().transpose() 
			<< "\n rtHand " << rtHand.translation().transpose() << endl;



			initPosL <<  0.3, 0.3,1.1; //.7, .6, 1.5;      //0.30, 0.35, 0.3;      
			ctl.efTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX((M_PI/180)*90), initPosL));
			// ctl.relEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyPosW.rotation(), initPosL));

			initPosR <<  0.3, -0.3,1.1; //0.30, -0.35, 0.3;
			ctl.efTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90), initPosR));
			// ctl.relEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyPosW.rotation(), initPosR));


			// ctl.solver().addTask(ctl.relEfTaskL);
			// ctl.solver().addTask(ctl.relEfTaskR);

			ctl.solver().addTask(ctl.efTaskL);
			ctl.solver().addTask(ctl.efTaskR);

			ctl.solver().addTask(chestPosTask);
			ctl.solver().addTask(chestOriTask);

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
