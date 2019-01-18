#pragma once

#include <iostream>
#include <fstream>

#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <mc_control/mc_controller.h>
#include <mc_control/fsm/Controller.h>
#include <mc_control/api.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/GUIState.h>

#include <Eigen/Core>

#include <mc_tasks/CoMTask.h>
#include <mc_tasks/MetaTask.h>
#include <mc_tasks/RelativeEndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/PostureTask.h>

#include <Tasks/QPContactConstr.h>
#include <Tasks/QPTasks.h>

#include <mc_rbdyn/ForceSensor.h>
#include <mc_rbdyn/Robot.h>
#include <mc_rbdyn/RobotLoader.h>
#include <mc_rbdyn/RobotModule.h>
#include <mc_rbdyn/Contact.h>


#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <boost/filesystem/fstream.hpp>

#include <typeinfo>

#include "handover_complianceTask.h"
#include "handover_trajectories.h"

#include "helper_functions.h"
// #include "matplotlibcpp.h"
// namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;
using namespace mc_control;

namespace mc_handover
{
	struct MC_CONTROL_DLLAPI HandoverController : public mc_control::fsm::Controller
	{
	public:
		HandoverController(std::shared_ptr<mc_rbdyn::RobotModule> RobotModule,
			double dt,
			const mc_rtc::Configuration & config);

		virtual ~HandoverController() {}

		virtual bool run() override;

		virtual void reset(const ControllerResetData & reset_data) override;

		virtual bool read_msg(std::string & msg) override;

		virtual bool read_write_msg(std::string & msg, std::string & out) override;


		std::map<std::string, sva::ForceVecd> wrenches;
		sva::ForceVecd wrenchLt, wrenchRt;
		
		std::map<std::string, mc_rbdyn::SurfacePtr> surf;

		std::shared_ptr<mc_tasks::RelativeEndEffectorTask> relEfTaskL;
		std::shared_ptr<mc_tasks::RelativeEndEffectorTask> relEfTaskR;
		
		std::shared_ptr<mc_tasks::EndEffectorTask> efTaskL;
		std::shared_ptr<mc_tasks::EndEffectorTask> efTaskR;

		std::shared_ptr<mc_tasks::OrientationTask> oriTaskL;
		std::shared_ptr<mc_tasks::OrientationTask> oriTaskR;

		std::shared_ptr<mc_tasks::PositionTask> posTaskL;
		std::shared_ptr<mc_tasks::PositionTask> posTaskR;

		std::shared_ptr<mc_handover::HandoverComplianceTask> compliTaskL;
		std::shared_ptr<mc_handover::HandoverComplianceTask> compliTaskR;

		std::shared_ptr<mc_handover::HelperFunctions> helpFun;
		std::shared_ptr<mc_handover::CircularTrajectory> cirTraj;
		std::shared_ptr<mc_handover::HandoverTrajectory> handoverTraj;
		std::shared_ptr<mc_handover::HandoverTrajectoryTask>handoverTrajTask = nullptr;


		bool publishWrench();

		bool getHostInfo();
		bool Flag_ROBOT{false}; // default True for robot
		
		bool runOnce = true;

    int iter=0;

	private:

		double openG{0.5};
		double closeG{0.2};
		
	};
} // namespace mc_control
