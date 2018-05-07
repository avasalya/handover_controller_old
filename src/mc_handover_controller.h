#ifndef _H_mc_handover_controller_H_
#define _H_mc_handover_controller_H_

#include <mc_control/mc_controller.h>
#include <mc_control/mc_controller.h>

#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/TrajectoryTask.h>
#include <mc_tasks/CoMTask.h>
#include <mc_tasks/TrajectoryTask.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/logging.h>

#include <Tasks/QPContactConstr.h>
#include <Tasks/QPTasks.h>

// #include <mc_rbdyn_urdf/robot.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include "handover_minJerk.h"
#include "handover_traj.h"




namespace mc_control
{
    class minJerk;
    struct MCHandoverController : public MCController
    {
      public:

      MCHandoverController(const std::shared_ptr<mc_rbdyn::RobotModule> & RobotModule, const double & dt);

      virtual void reset(const ControllerResetData & reset_data) override;

      virtual bool run() override;


      void init_pos();
      
      void createWaypoints();


      /* Generic message passing */
      virtual bool read_msg(std::string & msg) override;
      
            
      std::shared_ptr<mc_tasks::CoMTask> comTask;

      std::shared_ptr<mc_tasks::EndEffectorTask> efTaskL;
      std::shared_ptr<mc_tasks::EndEffectorTask> efTaskR;
      
      std::shared_ptr<mc_tasks::OrientationTask> oriTaskL;
      std::shared_ptr<mc_tasks::OrientationTask> oriTaskR;

      minJerk  mjObj;
    
    };
}

SIMPLE_CONTROLLER_CONSTRUCTOR("Handover", mc_control::MCHandoverController)

#endif
