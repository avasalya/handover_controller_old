#include "GoHalfSittingStep.h"

namespace mc_handover
{
	namespace states
	{
		void GoHalfSittingStep::configure(const mc_rtc::Configuration & config)
		{
			threshold_eval_ = config("threshold_eval", 0.005);
			threshold_speed_ = config("threshold_speed", 0.005);
			stiffness_ = config("stiffness", 2.);
			weight_ = config("weight", 1500.);
		}

		void GoHalfSittingStep::start(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.getPostureTask(ctl.robot().name())->target(ctl.robot().module().stance());
		}

		bool GoHalfSittingStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			if (ctl.getPostureTask(ctl.robot().name())->eval().norm() < threshold_eval_ && ctl.getPostureTask(ctl.robot().name())->speed().norm() < threshold_speed_){
			return true;
			}
			// Check if the thresholds are reached
			return false;
		}

	} // namespace states

} // namespace mc_torquing_controller
