
#include "ApproachObjectStep.h"

namespace mc_handover
{
	namespace states {

		void ApproachObjectStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config" << endl;

			threshold_eval_ = config("threshold_eval", 0.005);
			threshold_speed_ = config("threshold_speed", 0.005);
			stiffness_ = config("stiffness", 2.);
			weight_ = config("weight", 1500.);
		}
		void ApproachObjectStep::start(mc_control::fsm::Controller & controller)
		{	 

			cout << "start" << endl;
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			// try translate_ef(const Eigen::Vector3d & t)
			posL << 0.45, 0.35, 1.0;// 0.37, 0.7, 0.8; 
			// getCurRotL = ctl.relEfTaskL->get_ef_pose().rotation();
			getCurRotL = ctl.efTaskL->get_ef_pose().rotation();

			posR << 0.45, -0.35, 1.0;
			// getCurRotR = ctl.relEfTaskR->get_ef_pose().rotation();
			getCurRotR = ctl.efTaskR->get_ef_pose().rotation();

			// ctl.set_joint_pos("HEAD_JOINT1",  0.4); //+ve to move head down
			// controller.set_joint_pos("HEAD_JOINT0",  0.4); //+ve to move head down
		}

		bool ApproachObjectStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			sva::PTransformd dtrL(getCurRotL, posL);
			// ctl.relEfTaskL->set_ef_pose(dtrL);
			ctl.efTaskL->set_ef_pose(dtrL);

			sva::PTransformd dtrR(getCurRotR, posR);
			// ctl.relEfTaskR->set_ef_pose(dtrR);
			ctl.efTaskR->set_ef_pose(dtrR);
 
			output("OK");

			return true;
		}



	} // namespace states

} // namespace mc_handover
		


			// controller.getPostureTask(ctl.robot().name())->target({{"HEAD_JOINT0",{0.4}}});

			// unsigned int pan_i = ctl.robot().jointIndexByName("HEAD_JOINT0");
			// unsigned int tilt_i = ctl.robot().jointIndexByName("HEAD_JOINT1");
			// auto p = ctl.postureTask->posture();
			// p[pan_i][0] = -0.4;
			// p[tilt_i][0] = 0.4;
			// ctl.postureTask->posture(p);



		// void ApproachObjectStep::teardown(mc_control::fsm::Controller & controller)
		// {
		// 	auto & ctl = static_cast<mc_handover::HandoverController&>(controller);


		// 	if(ctl.relEfTaskL->eval().norm() < threshold_eval_ &&
		// 	 	ctl.relEfTaskR->eval().norm() < threshold_eval_ &&
		// 		ctl.relEfTaskL->speed().norm() < threshold_speed_ &&
		// 		ctl.relEfTaskR->speed().norm() < threshold_speed_)
		// 	{
		// 		ctl.solver().removeTask(ctl.relEfTaskL);
		// 		ctl.solver().removeTask(ctl.relEfTaskR);
		// 		ctl.solver().removeTask(ctl.oriTaskL);
		// 		ctl.solver().removeTask(ctl.oriTaskR);				
		// 	}	
		
		// 	cout << "teardown" << endl;
			
		// }
