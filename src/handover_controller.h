#pragma once
#include <mc_control/mc_controller.h>
#include <mc_control/api.h>

#include <mc_rtc/logging.h>

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

#include <RBDyn/EulerIntegration.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <boost/filesystem/fstream.hpp>

#include <typeinfo>


#include "handover_complianceTask.h"

#define initComplianceTask 1        //1 to initialize complianceTask
#define initForceSensor    1        //1 to  enable ForceSensor code

using namespace std;

namespace mc_control
{   
    class minJerk;

    struct MC_CONTROL_DLLAPI HandoverController : public MCController
    {
      public:
        HandoverController(const std::shared_ptr<mc_rbdyn::RobotModule> & RobotModule,
                         const double & dt);
                         // , const Configuration & conf);

        virtual ~HandoverController() {}

        virtual bool run() override;

        virtual void reset(const ControllerResetData & reset_data) override;

        virtual bool read_msg(std::string & msg) override;

        virtual bool read_write_msg(std::string & msg, std::string & out) override;

        
        void gripperControl();

      private:
        
        bool runOnlyOnce = true;
        bool GripperOpeningMsg = true;


        std::shared_ptr<mc_tasks::CoMTask> comTask;

        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> relEfTaskL;
        std::shared_ptr<mc_tasks::RelativeEndEffectorTask> relEfTaskR;
        
        std::shared_ptr<mc_tasks::OrientationTask> oriTaskL;
        std::shared_ptr<mc_tasks::OrientationTask> oriTaskR;

        std::shared_ptr<mc_handover::HandoverComplianceTask> compliTaskL;
        std::shared_ptr<mc_handover::HandoverComplianceTask> compliTaskR;

        std::map<std::string, sva::ForceVecd> wrenches;
        sva::ForceVecd wrenchLt, wrenchRt;
        
        // std::shared_ptr<mc_rbdyn::ForceSensor> forceSensor;

        
        // mc_rbdyn::detail::ForceSensorCalibData calibrator;



    };
} // namespace mc_control
    
    SIMPLE_CONTROLLER_CONSTRUCTOR("Handover", mc_control::HandoverController)
// CONTROLLER_CONSTRUCTOR("Handover", mc_control::HandoverController)