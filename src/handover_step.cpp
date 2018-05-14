//
/// ... mc_handover_complianceTask ...
///

#include "handover_FSM.h"

namespace mc_control
{
	handoverStep::handoverStep(const std::string & name) : name(name)
	{}


	HandoverStep * HandoverStep::update(HandoverController & ctl)
	{
		if(firstCall)
		{
			init_(ctl);
			firstCall = false;
			return this;
		}
		return update_(ctl);
	}

	void InitStep::init_(HandoverController & ctl)
	{
		ctl.comTask = std::make_shared<mc_tasks::CoMTask>(ctl.robots(),
		ctl.robots().robotIndex(), 3., 100.);
		auto mbc = ctl.robot().mbc();
		mbc.q = ctl.postureTask->posture();
		auto comT = rbd::computeCoM(ctl.robot().mb(), mbc);
		ctl.comTask->set_com(comT);
		ctl.comTask->addToSolver(ctl.solver());
	}


	HandoverStep * InitStep::update_(HandoverController & ctl)
	{
		if(ctl.comTask->comTask->speed().norm() < 1e-2)
		{
			return new InitialHandPoseStep;
		}
		return this;
	}


	void InitialHandPoseStep::init_(HandoverController & ctl)
	{

	}

	HandoverStep * InitialHandPoseStep::update_(HandoverController & ctl)
	{

	}

	void GrippersOpenStep::init_(HandoverController & ctl)
	{

	}

	HandoverStep * GrippersOpenStep::update_(HandoverController & ctl)
	{

	}

	void GrippersAddStep::init_(HandoverController & ctl)
	{

	}

	HandoverStep * GrippersAddStep::update_(HandoverController & ctl)
	{

	}
	
	void GrippersCloseStep::init_(HandoverController & ctl)
	{

	}

	HandoverStep * GrippersCloseStep::update_(HandoverController & ctl)
	{

	}

	void AdjustHandStep::init_(HandoverController & ctl)
	{

	}
    HandoverStep * AdjustHandStep::update_(HandoverController & ctl)
    {

    }

} // namespace mc_control