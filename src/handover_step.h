#pragma once
#include "handover_controller.h"

#include <mc_solver/BoundedSpeedConstr.h>
#include <mc_solver/CoMIncPlaneConstr.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_rbdyn/polygon_utils.h>

namespace mc_control
{
	struct HandoverStep
	{
		public:
			HandoverStep(const std::string & name);
			virtual ~HandoverStep() {}
			HandoverStep * update(HandoverController & ctl);
			std::string name;

		protected:
			virtual void init_(HandoverController & ctl) = 0;			
			virtual HandoverStep * update_(HandoverController & ctl) = 0;
			bool firstCall =  true;

	};
	
	
	#define CREATE_Step(NAME, DESC, MEMBERS)\
	struct NAME : public HandoverStep\
	{\
		NAME() : HandoverStep(DESC) {}\
		virtual void init_(HandoverController & ctl) override;\
		virtual HandoverStep * update_(HandoverStep & ctl) override;\
		MEMBERS\
	};	

	CREATE_Step(InitStep, "initializeStep",)

	CREATE_Step(InitialHandPoseStep, "move hands to initial pose",)

	CREATE_Step(GrippersOpenStep, "open both grippers step", double targetOpen; double openSpeed;)

	CREATE_Step(GrippersAddStep, "Add grippers to controller step", std::shared_ptr<mc_tasks::ComplianceTask> complianceTask; mc_rbdyn::ForceSensor forceSensor;)

	CREATE_Step(GrippersCloseStep, "close both grippers step", double targetClose; double CloseSpeed;)

	CREATE_Step(AdjustHandStep, "Adjust hand step", std::shared_ptr<mc_tasks::ComplianceTask> complianceTask; mc_rbdyn::ForceSensor forceSensor; tasks::qp::ContactId cId;)

	// CREATE_Step 

	CREATE_Step(ExperimentStep, "Constraint experiment step", std::shared_ptr<mc_solver::CoMIncPlaneConstr> comIncPlaneConstr; std::shared_ptr<mc_rbdyn::QuadraticGenerator> generator; std::unique_lock<std::mutex> interpolator_lock; std::shared_ptr<mc_rbdyn::PolygonInterpolator> interpolator; unsigned int interp_index; std::shared_ptr<mc_tasks::ComplianceTask> complianceTask;)

	#undef CREATE_Step

} // namespace mc_control




