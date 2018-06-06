#include "GoInitPoseStep.h"

namespace mc_handover
{
	namespace states {

		void GoInitPoseStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config" << endl;

			threshold_eval_ = config("threshold_eval", 0.005);
			threshold_speed_ = config("threshold_speed", 0.005);
			stiffness_ = config("stiffness", 2.);
			weight_ = config("weight", 1500.);
		}

		void GoInitPoseStep::start(mc_control::fsm::Controller & controller)
		{	
			
			cout << "start" << endl;

			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.solver().setContacts({
			mc_rbdyn::Contact(ctl.robots(), "LFullSole", "AllGround"),
			mc_rbdyn::Contact(ctl.robots(), "RFullSole", "AllGround"),
			mc_rbdyn::Contact(ctl.robots(), "Butthock", "AllGround"),
			mc_rbdyn::Contact(ctl.robots(), "LowerBack","AllGround")              
			});              


			ctl.relEfTaskR.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK7", ctl.robots(), ctl.robots().robotIndex(), "", 2.0,1e3));
			ctl.oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK7", ctl.robots(), ctl.robots().robotIndex(),3.0,1e2));

			ctl.relEfTaskL.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK7", ctl.robots(), ctl.robots().robotIndex(), "", 2.0,1e3));
			ctl.oriTaskL.reset(new mc_tasks::OrientationTask("LARM_LINK7", ctl.robots(), ctl.robots().robotIndex(),3.0,1e2));
			
		}

		bool GoInitPoseStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			cout << "run" << endl;
	        cout << "moving to initial pose" <<endl;
		
			controller.set_joint_pos("HEAD_JOINT1",  0.7); //+ve to move head down	      

	        BodyPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("BODY")];

	        initPosL <<  0.30, 0.35, 0.3;      
	        ctl.relEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyPosW.rotation(), initPosL));

	        initPosR <<  0.30, -0.35, 0.3;
	        ctl.relEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyPosW.rotation(), initPosR));
	        
	        			
	        ctl.solver().addTask(ctl.relEfTaskL);
	        ctl.solver().addTask(ctl.relEfTaskR);
		
	        output("OK");
	        
		    return true;
		}

	} // namespace states

} // namespace mc_torquing_controller

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
