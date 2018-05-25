#include <GoInitPoseStep.h>

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
			
		}

		bool GoInitPoseStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

		}

	} // namespace states

} // namespace mc_torquing_controller

EXPORT_SINGLE_STATE("GoInitPoseStep", mc_handover::states::GoInitPoseStep)