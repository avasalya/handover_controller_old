//
/// ... mc_handover_controller ...
///

#include "handover_controller.h"

using namespace std;
namespace mc_control
{
    //////////////
    //
    // Handover Controller constructor
    //
    //////////////
    HandoverController::HandoverController(const std::shared_ptr<mc_rbdyn::RobotModule> & robot_module, const double & dt):MCController(robot_module, dt)
    {
      
        selfCollisionConstraint.reset();

        qpsolver->addConstraintSet(contactConstraint);
        qpsolver->addConstraintSet(kinematicsConstraint);
        // qpsolver->addConstraintSet(dynamicsConstraint); //walking

        qpsolver->addConstraintSet(selfCollisionConstraint);
        selfCollisionConstraint.addCollisions(solver(), {
          mc_rbdyn::Collision("LARM_LINK3", "BODY", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("LARM_LINK4", "BODY", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("LARM_LINK5", "BODY", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("RARM_LINK3", "BODY", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("RARM_LINK4", "BODY", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("RARM_LINK5", "BODY", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("RARM_LINK3", "CHEST_LINK0", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("RARM_LINK4", "CHEST_LINK0", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("RARM_LINK5", "CHEST_LINK0", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("RARM_LINK4", "CHEST_LINK1", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("RARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("LARM_LINK3", "CHEST_LINK0", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("LARM_LINK4", "CHEST_LINK0", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK0", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("LARM_LINK4", "CHEST_LINK1", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.)
        });

        postureTask.reset(new mc_tasks::PostureTask(solver(), robots().robotIndex(), 3, 1e2));
        qpsolver->addTask(postureTask.get());

        qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
        mc_rbdyn::Contact(robots(), "LowerBack","AllGround")
        });

        comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
        solver().addTask(comTask);
        
        //      change oriTask to relEF orientation task      //
        relEfTaskR.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(), "", 50.0,1e3));
        oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK7", robots(), robots().robotIndex(),9.0,1e2));

        relEfTaskL.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK7", robots(), robots().robotIndex(), "", 50.0,1e3));
        oriTaskL.reset(new mc_tasks::OrientationTask("LARM_LINK7", robots(), robots().robotIndex(),9.0,1e2));

        /* Force Sensor*/
        if(initForceSensor)
        { 
          // (0, 0, -0.087);

          // forceSensor.reset(new mc_rbdyn::ForceSensor("LeftHandForceSensor", "LARM_LINK6", sva::PTransformd(Eigen::Vector3d(1., 1., 1.)))); 
          // fSensorVectL = robot_module->forceSensors();
        }

        /* Compliance Task*/
        if(initComplianceTask)
        { 
          compliTaskL.reset(new mc_tasks::ComplianceTask(robots(), robots().robotIndex(), "LARM_LINK6", 0.005, Eigen::Matrix6d::Identity(), 5, 1e3, 3, 1, {0.02, 0.005}, {0.2, 0.05}));          
        }

        LOG_SUCCESS("mc_handover_controller init done")
    }



    //////////////
    //
    // Handover Controller reset
    //
    //////////////
    void HandoverController::reset(const ControllerResetData & reset_data)
    {
        auto q = reset_data.q;
        MCController::reset({q});

        postureTask->posture(q);

        qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
        mc_rbdyn::Contact(robots(), "LowerBack","AllGround")
        });

        relEfTaskL->reset();
        relEfTaskR->reset();

        comTask->reset();

        /* Compliance Task*/
        if(initComplianceTask)
        {
          compliTaskL->reset();
          compliTaskL->resetJointsSelector(solver());
        }
    }



    //////////////
    //
    // Handover Controller run
    //
    //////////////
    bool HandoverController::run()
    {
      bool ret = MCController::run();

    if(runOnlyOnce)
    {
      runOnlyOnce = false;
      // init_pos();

      auto gripper = grippers["l_gripper"].get();
      gripper->setTargetQ({1});
      gripper = grippers["r_gripper"].get();
      gripper->setTargetQ({1});

      /* Force Sensor*/
      if(initForceSensor)
      {  
        wrenches["LeftHandForceSensor"] = 
                    this->robot().forceSensor("LeftHandForceSensor").wrench();
        wrenches["RightHandForceSensor"] =
                    this->robot().forceSensor("RightHandForceSensor").wrench();

        for(auto & i : wrenches){
          cout << i.first << ": " << i.second << '\n';
        }

        LOG_SUCCESS("ForceSensor enabled")
      }
      else
      {
        LOG_WARNING("ForceSensor disabled")
      }

      /* Compliance Task*/
      if(initComplianceTask)
      {
        LOG_SUCCESS("complianceTask initialized")          
      }
      else
      {
        LOG_WARNING("complianceTask not initialized")
      }        
    }

      cout << "\nforce readinig ----\n " << wrenches.at("RightHandForceSensor").force() <<endl<< "\ntorque readinig ---- \n" << wrenches.at("RightHandForceSensor").couple() << endl;
      
      return ret;
    }

       

    //////////////
    //
    // Handover Controller init_pos
    //
    //////////////
    void HandoverController::init_pos()
    {
      Eigen::Vector3d initPosR, initPosL;
      sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];

      initPosR <<  0.30, -0.35, 0.45;
      relEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosR));
      solver().addTask(relEfTaskR);


      initPosL <<  0.30, 0.35, 0.45;      
      relEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosL));
      solver().addTask(relEfTaskL);     
    }



    //////////////
    //
    // Handover Controller createWayPoints
    //
    //////////////
    // void HandoverController::createWaypoints()
    // {
    //   int sample = 10;
    //   for (int i=0; i<sample; i++)
    //   {
    //     mjTask->produceWp(MatrixXd::Random(sample,3), MatrixXd::Random(sample,3), i, sample);
    //     // std::cout << mjObj.yPos[i] << std::endl; // to acess any produced wp
    //   }
    // }



    //////////////
    //
    // Handover Controller read_msg
    //
    //////////////
    bool  HandoverController::read_msg(std::string & msg)
    {
      std::stringstream ss;
      std::string token;

      ss << msg;
      ss >> token;

      if(token == "step1")
      {
        // MCController::set_joint_pos("HEAD_JOINT0", -0.3); //-ve to move head to robot's right
        MCController::set_joint_pos("HEAD_JOINT1",  0.4); //+ve to move head down

        init_pos();
   
        return true;
      }


      if(token == "step2")
      {     
        
        if(initForceSensor)
        {

          cout << "force readinig ---- " << wrenches.at("RightHandForceSensor").force() <<endl<< "torque readinig ---- " << wrenches.at("RightHandForceSensor").couple() << endl;
        }

        if(initComplianceTask)
        {
          // solver().removeTask(relEfTaskL);          
          // compliTaskL->selectActiveJoints(solver(),activeJointsLeftArm);          
          // solver().addTask(compliTaskL);

        
          // if(v >= 10)
          // {
          //   auto gripper = grippers["l_gripper"].get();
          //   gripper->setTargetQ({-0.25});
          //   cout << "closing left gripper " << endl;
          // }

        }
        return true;
      }


      if(token == "step3")
      { 
        //set ef pose 
        Eigen::Vector3d tL( 0.30, 0.35, 1. );
        Eigen::Matrix3d getCurRotL =  relEfTaskL->get_ef_pose().rotation();
        sva::PTransformd dtrL(getCurRotL, tL);
        relEfTaskL->set_ef_pose(dtrL);


        Eigen::Vector3d tR( 0.30, -0.35, 1. );
        Eigen::Matrix3d getCurRotR =  relEfTaskR->get_ef_pose().rotation();
        sva::PTransformd dtrR(getCurRotR, tR);
        relEfTaskR->set_ef_pose(dtrR);

      }


      if(token == "open_rgripper")
      {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({1});
        return true;
      }
      if(token == "close_rgripper")
      {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({-0.25});
        return true;
      }



      if(token == "open_lgripper")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({1});
        return true;
      }
      if(token == "close_lgripper")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({-0.25});
        return true;
      }



      if(token == "open_grippers")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({1});
        gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({1});
        return true;
      }
      if(token == "close_grippers")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({-0.25});
        gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({-0.25});
        return true;
      }

      
      /// get RARM JOINTs
      std::vector<double>  get_RArm_Joints(8);
      if(token == "get_pose")
      {
        for (int i = 0; i<8; ++i)
        {
          ss << "RARM_JOINT" << i;
          std::cout << robot().mbc().q[robot().jointIndexByName(ss.str())][0] << ", ";
        }
        std::cout<<std::endl;
        return true;
      }
    
    LOG_WARNING("Cannot handle " << msg)
    return false;

    } //read_msg
   
} //namespace mc_control
