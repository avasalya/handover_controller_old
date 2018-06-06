
#include "GoDummyPoseStep.h"

namespace mc_handover
{
	namespace states {

		void GoDummyPoseStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config" << endl;

			threshold_eval_ = config("threshold_eval", 0.005);
			threshold_speed_ = config("threshold_speed", 0.005);
			stiffness_ = config("stiffness", 2.);
			weight_ = config("weight", 1500.);
		}
		void GoDummyPoseStep::start(mc_control::fsm::Controller & controller)
		{	

			cout << "start" << endl;
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.relEfTaskR.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK7", ctl.robots(), ctl.robots().robotIndex(), "", 5.0,1e3));
			// ctl.oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK7", ctl.robots(), ctl.robots().robotIndex(),3.0,1e2));

			ctl.relEfTaskL.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK7", ctl.robots(), ctl.robots().robotIndex(), "", 5.0,1e3));
			// ctl.oriTaskL.reset(new mc_tasks::OrientationTask("LARM_LINK7", ctl.robots(), ctl.robots().robotIndex(),3.0,1e2));

	        // ctl.solver().addTask(ctl.oriTaskL);
	        // ctl.solver().addTask(ctl.oriTaskR);
        
	        posL << 0.6, 0.35, .4;
	        getCurRotL = ctl.relEfTaskL->get_ef_pose().rotation();

	        posR << 0.6, -0.35, .4;
	        getCurRotR = ctl.relEfTaskR->get_ef_pose().rotation();

		}

		bool GoDummyPoseStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			cout << "run -- dummy pose" << endl;
			
			controller.set_joint_pos("HEAD_JOINT1",  1);// 0.4 //+ve to move head down     

	        sva::PTransformd dtrL(getCurRotL, posL);	
	        ctl.relEfTaskL->set_ef_pose(dtrL);
	        
	        sva::PTransformd dtrR(getCurRotR, posR);
	        ctl.relEfTaskR->set_ef_pose(dtrR);
	
	        ctl.solver().addTask(ctl.relEfTaskR);
    		ctl.solver().addTask(ctl.relEfTaskL);

	        output("OK");	       

		    return true;
		}



	} // namespace states

} // namespace mc_torquing_controller
