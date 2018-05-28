#include "GoInitPoseStep.h"

namespace mc_handover
{
	namespace states {

		void GoInitPoseStep::configure(const mc_rtc::Configuration & config)
		{
			threshold_eval_ = config("threshold_eval", 0.005);
			threshold_speed_ = config("threshold_speed", 0.005);
			stiffness_ = config("stiffness", 2.);
			weight_ = config("weight", 1500.);
		}

		void GoInitPoseStep::start(mc_control::fsm::Controller & controller)
		{
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.relEfTaskR.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(), "", 50.0,1e3));
			ctl.oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK7", robots(), robots().robotIndex(),3.0,1e2));

			ctl.relEfTaskL.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK7", robots(), robots().robotIndex(), "", 50.0,1e3));
			ctl.oriTaskL.reset(new mc_tasks::OrientationTask("LARM_LINK7", robots(), robots().robotIndex(),3.0,1e2));
		}

		bool GoInitPoseStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
			
			if(ctl.relEfTaskL->eval().norm() < threshold_eval_ && ctl.relEfTaskR->eval().norm() < threshold_eval_ &&
				ctl.relEfTaskL->speed().norm() < threshold_speed_ && ctl.relEfTaskR->speed().norm() < threshold_speed_ && )
			{
				ctl.relEfTaskL->reset();
				ctl.relEfTaskR->reset();
				output("not moving yet");
				return false;
			}	
			else
			{
				MCController::set_joint_pos("HEAD_JOINT1",  0.4); //+ve to move head down	      

		        BodyPosW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];

		        initPosL <<  0.30, 0.35, 0.45;      
		        ctl.relEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyPosW.rotation(), initPosL));
		        ctl.solver().addTask(relEfTaskL);

		        initPosR <<  0.30, -0.35, 0.45;
		        ctl.relEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyPosW.rotation(), initPosR));
		        ctl.solver().addTask(relEfTaskR);
		        output("moving to initial pose");
		        return true;
		    }
		    return false;
		}

		void GoInitPoseStep::teardown(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
			
				ctl.solver().removeTask(ctl.relEfTaskL);
				ctl.solver().removeTask(ctl.relEfTaskR);
				ctl.solver().removeTask(ctl.oriTaskL);
				ctl.solver().removeTask(ctl.oriTaskR);
		}

	} // namespace states

} // namespace mc_torquing_controller
