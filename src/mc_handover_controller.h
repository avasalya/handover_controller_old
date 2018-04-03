#ifndef _H_mc_handover_controller_H_
#define _H_mc_handover_controller_H_

#include <mc_control/mc_controller.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/TrajectoryTask.h>
#include <mc_tasks/CoMTask.h>

#include<Tasks/QPContactConstr.h>

#include <mc_rtc/logging.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>


namespace mc_control
{

    struct MCHandoverController : public MCController
    {
      public:

      MCHandoverController(const std::shared_ptr<mc_rbdyn::RobotModule> & robot, const double & dt);

      virtual void reset(const ControllerResetData & reset_data) override;

      virtual bool run() override;

      
      /* traj_task */
     // void traj_task(const std::string & name);

      void init_pos();
      
      /* Generic message passing */
      virtual bool read_msg(std::string & msg) override;
      
      
      public:
      std::shared_ptr<mc_tasks::EndEffectorTask> efTask;
      std::shared_ptr<mc_tasks::OrientationTask> oriTask;
      std::shared_ptr<mc_tasks::CoMTask> comTask;
      // std::shared_ptr<mj_traj::mj_trajectory> trajectoryTask = nullptr;
      std::shared_ptr<mc_tasks::PositionTask> elbow_pos;

      private:
      bool ready = true;
      bool first_traj = true;

    
      unsigned int head_joint_index;
      double head_joint_target;

    };
}

SIMPLE_CONTROLLER_CONSTRUCTOR("Handover", mc_control::MCHandoverController)

#endif
