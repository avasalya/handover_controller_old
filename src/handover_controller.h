#pragma once
#include <mc_control/mc_controller.h>

#include <mc_tasks/CoMTask.h>
// #include <mc_tasks/AdmittanceTask.h>
// #include <mc_tasks/MetaTask.h>
// #include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/RelativeEndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/ComplianceTask.h>

#include <mc_rtc/logging.h>

#include <Tasks/QPContactConstr.h>
#include <Tasks/QPTasks.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include "handover_minJerk.h"
#include "handover_FSM.h"


namespace mc_control
{   
    class minJerk;
    struct HandoverStep;
    struct InitStep;

    struct HandoverController : public MCController
    {
      public:
        HandoverController(const std::shared_ptr<mc_rbdyn::RobotModule> & RobotModule, const double & dt);

        virtual ~HandoverController() {}

        virtual bool run() override;

        virtual void reset(const ControllerResetData & reset_data) override;

        virtual bool read_msg(std::string& msg) override;

        virtual bool play_next_step() override;


        void init_pos();       

        void createWaypoints();

        std::vector<std::string> activeJointsLeftArm =
          {
               "LARM_JOINT0",
               "LARM_JOINT1",
               "LARM_JOINT2",
               "LARM_JOINT3",
               "LARM_JOINT4",
               "LARM_JOINT5",
               "LARM_JOINT6",
               "LARM_JOINT7",
               };
           std::vector<std::string> activeJointsRightArm =
          {    "RARM_JOINT0",
               "RARM_JOINT1",
               "RARM_JOINT2",
               "RARM_JOINT3",
               "RARM_JOINT4",
               "RARM_JOINT5",
               "RARM_JOINT6",
               "RARM_JOINT7" };
          std::vector<std::string> activeJointsLegs =
          {"Root", "RLEG_JOINT0", "RLEG_JOINT1", "RLEG_JOINT2",
           "RLEG_JOINT3", "RLEG_JOINT4", "RLEG_JOINT5",
           "LLEG_JOINT0", "LLEG_JOINT1", "LLEG_JOINT2",
           "LLEG_JOINT3", "LLEG_JOINT4", "LLEG_JOINT5"};
          std::vector<std::string> activeJointsHead =
          {"HEAD_JOINT0", "HEAD_JOINT1"};

                      
      private:
        std::shared_ptr<mc_tasks::CoMTask> comTask;

        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> relEfTaskL;
        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> relEfTaskR;
        
        std::shared_ptr<mc_tasks::OrientationTask> oriTaskL;
        std::shared_ptr<mc_tasks::OrientationTask> oriTaskR;

        std::shared_ptr<mc_tasks::ComplianceTask> compliTaskL;
        std::shared_ptr<mc_tasks::ComplianceTask> compliTaskR;

        mc_rbdyn::ForceSensorsCalibrator calibrator;

        std::shared_ptr<MinJerk>  mjTask;

        handoverStep * step = nullptr;

        bool runOnlyOnce = true;
        bool paused = false;

    private:
        void addToLogger(mc_rtc::Logger & logger) override;
        void removeFromLogger(mc_rtc::Logger & logger) override;
        
    };
} // namespace mc_control

SIMPLE_CONTROLLER_CONSTRUCTOR("Handover", mc_control::HandoverController)




        // std::shared_ptr<mc_tasks::AdmittanceTask> AdmittanceTaskL;

        // std::shared_ptr<mc_tasks::PostureTask> postureTaskL;

        // std::shared_ptr<tasks::qp::PostureTask> postureTask;

