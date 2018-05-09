#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/mc_controller.h>

// #include <mc_tasks/MetaTask.h>
// #include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/RelativeEndEffectorTask.h>
#include <mc_tasks/OrientationTask.h>
#include <mc_tasks/ComplianceTask.h>
#include <mc_tasks/CoMTask.h>

#include <mc_rtc/logging.h>
#include <mc_rtc/logging.h>

#include <Tasks/QPContactConstr.h>
#include <Tasks/QPTasks.h>

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include "handover_minJerk.h"
#include "handover_complianceTask.h"


namespace mc_control
{
    class minJerk;
    // struct MCHandoverComplianceController;

    struct MCHandoverController : public MCController
    {
      public:
        MCHandoverController(const std::shared_ptr<mc_rbdyn::RobotModule> & RobotModule, const double & dt);

        virtual void reset(const ControllerResetData & reset_data) override;

        virtual bool run() override;

        virtual bool read_write_msg(std::string& msg, std::string& out) override;

        void init_pos();        
        void createWaypoints();

        bool enableSensor(const std::string& sensorName);
        bool disableSensor(const std::string& sensorName);

        double forceGain;
        double torqueGain;

        double forceTh;
        double torqueTh;

                      
      private:
        std::shared_ptr<mc_tasks::CoMTask> comTask;

        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> rEfTaskL;
        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> rEfTaskR;
        
        std::shared_ptr<mc_tasks::OrientationTask> oriTaskL;
        std::shared_ptr<mc_tasks::OrientationTask> oriTaskR;

        std::shared_ptr<mc_tasks::ComplianceTask> compTaskL;
        std::shared_ptr<mc_tasks::ComplianceTask> compTaskR;

        // std::shared_ptr<mc_tasks::AdmittanceTask> AdmittanceTaskL;
        // std::shared_ptr<mc_tasks::AdmittanceTask> AdmittanceTaskR;

        // std::shared_ptr<mc_tasks::PostureTask> postureTaskL;
        // std::shared_ptr<mc_tasks::PostureTask> postureTaskR;

        // std::shared_ptr<tasks::qp::PostureTask> postureTask;

        std::shared_ptr<minJerk>  mjTask;


        std::vector<bool> enabled_;
        std::vector<mc_rbdyn::ForceSensor> sensors_;
        std::map<std::string, std::size_t> sensorIndexByName_;
    
        
    };
}

SIMPLE_CONTROLLER_CONSTRUCTOR("Handover", mc_control::MCHandoverController)