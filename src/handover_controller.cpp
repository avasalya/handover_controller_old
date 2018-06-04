//
/// ... mc_handover_controller ...
///

#include "handover_controller.h"

namespace mc_handover
{
    //////////////
    //
    // Handover Controller constructor
    //
    //////////////
    HandoverController::HandoverController(std::shared_ptr<mc_rbdyn::RobotModule> robot_module, 
      double dt, 
      const mc_rtc::Configuration & config) 
      :mc_control::fsm::Controller(
        robot_module,
        dt,
        config)
    {
    
        qpsolver->addConstraintSet(selfCollisionConstraint);
    
        selfCollisionConstraint.reset();
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
          mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.),
          mc_rbdyn::Collision("RARM_LINK5", "RLEG_LINK2", 0.05,0.01, 0.),
          mc_rbdyn::Collision("RARM_LINK6", "RLEG_LINK2", 0.05,0.01, 0.),
          mc_rbdyn::Collision("RARM_LINK7", "RLEG_LINK2", 0.05,0.01, 0.),
          mc_rbdyn::Collision("LARM_LINK5", "LLEG_LINK2", 0.05,0.01, 0.),
          mc_rbdyn::Collision("LARM_LINK6", "LLEG_LINK2", 0.05,0.01, 0.),
          mc_rbdyn::Collision("LARM_LINK7", "LLEG_LINK2", 0.05,0.01, 0.),

        });

        LOG_SUCCESS("mc_handover_controller init done")
    }



    //////////////
    //
    // Handover Controller reset
    //
    //////////////
    void HandoverController::reset(const ControllerResetData & reset_data)
    {   
        /* Force Sensor */
        if(initForceSensor){LOG_INFO("ForceSensor enabled")}else{LOG_WARNING("ForceSensor disabled")}
        /* Compliance Task */
        if(initComplianceTask){LOG_INFO("ComplianceTask initialized")}else{LOG_WARNING("complianceTask not initialized")}        
         /* FSM */
        if(initFSM){LOG_INFO("FSM initialized")}else{LOG_WARNING("FSM not initialized")}


        mc_control::fsm::Controller::reset(reset_data);

        auto q = reset_data.q;
        MCController::reset({q});

    }



    //////////////
    //
    // Handover Controller run
    //
    //////////////
    bool HandoverController::run()
    {
      bool ret = mc_control::fsm::Controller::run();

      if(ret)
      {
        /* Force Sensor*/
        if(initForceSensor)
        {        
          // transform from Vrep force sensor reference system to solver force sensor reference system
          wrenches["LeftHandForceSensor"] = 
                    this->robot().forceSensor("LeftHandForceSensor").wrench();
          wrenchLt = wrenches.at("LeftHandForceSensor");
          wrenches.at("LeftHandForceSensor").couple() << wrenchLt.couple()[2],wrenchLt.couple()[1],-wrenchLt.couple()[0];
          wrenches.at("LeftHandForceSensor").force() << wrenchLt.force()[2],wrenchLt.force()[1],-wrenchLt.force()[0];
          // cout << "left hand "<< wrenchLt << '\n';

          wrenches["RightHandForceSensor"] =
                    this->robot().forceSensor("RightHandForceSensor").wrench();
          wrenchRt = wrenches.at("RightHandForceSensor");   
          wrenches.at("RightHandForceSensor").couple() << wrenchRt.couple()[2],wrenchRt.couple()[1],-wrenchRt.couple()[0];
          wrenches.at("RightHandForceSensor").force() << wrenchRt.force()[2],wrenchRt.force()[1],-wrenchRt.force()[0];
          // cout <<  "right hand " << wrenchRt.force().eval().norm() << endl;

        }


         /* Compliance Task*/
        if(initComplianceTask)
        {
          if(relEfTaskR && relEfTaskR->eval().norm() < 1e-1 && relEfTaskR->speed().norm() < 1e-2)
          {
            cout << "initiating compliance task" <<  endl;

            solver().removeTask(relEfTaskR);
            relEfTaskR->reset();

            compliTaskR.reset(new mc_handover::HandoverComplianceTask(robots(), robots().robotIndex(), robots().robot().forceSensor("RightHandForceSensor").parentBody(), 0.005, Eigen::Matrix6d::Identity(),5, 1e3, 3, 1, {0.02, 0.005}, {0.2, 0.05}));
            compliTaskR->setTargetWrench(sva::ForceVecd(Eigen::Vector3d(0., 0., 0.),Eigen::Vector3d(0., 0., 50.)));
            compliTaskR->addToSolver(solver());
          }
          
          if(compliTaskR)
          {
            cout << "initialized compliance task" <<  endl;
            compliTaskR->update();
            if(compliTaskR->eval().norm() < 25 && compliTaskR->speed().norm() < 0.001)
            {
              LOG_SUCCESS("Contact established")
              compliTaskR->removeFromSolver(solver());
              compliTaskR->reset();
              qpsolver->setContacts({
              mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
              mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
              mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
              mc_rbdyn::Contact(robots(), "LowerBack","AllGround"),
              mc_rbdyn::Contact(robots(), "RightFingers","AllGround")
              });              
            }
          }

          // cout << compliTaskR->getTargetWrench() << endl;
          // cout << compliTaskR->eval().norm() << endl;
        }

        // gripperControl();
      }

      return ret;
    }






    //////////////
    //
    // Handover Controller check Gripper control
    //
    //////////////
    void HandoverController::gripperControl()
    {
      cout << "force().eval().norm()  " << wrenches.at("RightHandForceSensor").force().eval().norm() << '\n';
      if(wrenches.at("RightHandForceSensor").force().eval().norm() > 6){

        if(GripperOpeningMsg){
          GripperOpeningMsg = false;
          cout << "opening grippers" << endl;
        }        

        /* setTargetWrench */
        wrenches.at("RightHandForceSensor").force() = Eigen::Vector3d(0., 0., 0.);

        for(const auto & g : grippers)
        {
          g.second->setTargetOpening(1.0);
        } 
      }
      else
      {
        cout << "forces not enough to open grippers, pull harder" << endl;
      }
        cout <<"right hand force sensor" << wrenches.at("RightHandForceSensor").force().transpose() << endl;

      // cout << wrenches.at("RightHandForceSensor").force() << endl;
      // wrenches.at("RightHandForceSensor").force() = Eigen::Vector3d(0., 0., 0.);
    }







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

       if (onlyOnce)
        {
           onlyOnce = false;
           qpsolver->setContacts({
              mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
              mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
              mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
              mc_rbdyn::Contact(robots(), "LowerBack","AllGround")              
              });              

            relEfTaskR.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(), "", 5.0,1e3));
            oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK7", robots(), robots().robotIndex(),3.0,1e2));

            relEfTaskL.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK7", robots(), robots().robotIndex(), "", 5.0,1e3));
            oriTaskL.reset(new mc_tasks::OrientationTask("LARM_LINK7", robots(), robots().robotIndex(),3.0,1e2));
          }


        MCController::set_joint_pos("HEAD_JOINT1",  0.4); //+ve to move head down
        Eigen::Vector3d initPosR, initPosL;
        sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];

        initPosR <<  0.30, -0.35, 0.3;
        relEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosR));
        solver().addTask(relEfTaskR);


        initPosL <<  0.30, 0.35, 0.3;      
        relEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosL));
        solver().addTask(relEfTaskL);     
   
        return true;
      }



      if(token == "step2")
      { 
        //set ef pose 
        Eigen::Vector3d tL( 0.7, 0.35, .3 );
        Eigen::Matrix3d getCurRotL =  relEfTaskL->get_ef_pose().rotation();
        sva::PTransformd dtrL(getCurRotL, tL);
        relEfTaskL->set_ef_pose(dtrL);


        Eigen::Vector3d tR( 0.7, -0.35, .3 );
        Eigen::Matrix3d getCurRotR =  relEfTaskR->get_ef_pose().rotation();
        sva::PTransformd dtrR(getCurRotR, tR);
        relEfTaskR->set_ef_pose(dtrR);

        return true;
      }



      if(token == "robots")
      { 
        std::string robotName =  this->robot().name();

        cout << robotName <<  endl;

        return true;
      }



      if(token == "surfaces")
      { 
        surf =  this->robot().surfaces();

        for(auto elem : surf)
        {
          std::cout << elem.first << " " << elem.second << endl;
        }
        return true;
      }

      // gripper control actions //

      if(token == "openGripperR")
      {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({1});
        return true;
      }
      if(token == "closeGripperR")
      {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({-0.25});
        return true;
      }



      if(token == "openGripperL")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({1});
        return true;
      }
      if(token == "closeGripperL")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({-0.25});
        return true;
      }



      if(token == "openGrippers")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({1});
        gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({1});
        return true;
      }
      if(token == "closeGrippers")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({-0.25});
        gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({-0.25});
        return true;
      }

      
      // get LARM JOINTs //
      std::vector<double>  get_LArm_Joints(8);
      if(token == "getRtPose")
      {
        for (int i = 0; i<8; ++i)
        {
          ss << "RARM_JOINT" << i;
          std::cout << robot().mbc().q[robot().jointIndexByName(ss.str())][0] << ", ";
        }
        std::cout<<std::endl;
        return true;
      }

      // get RARM JOINTs //
      std::vector<double>  get_RArm_Joints(8);
      if(token == "getLtPose")
      {
        for (int i = 0; i<8; ++i)
        {
          ss << "LARM_JOINT" << i;
          std::cout << robot().mbc().q[robot().jointIndexByName(ss.str())][0] << ", ";
        }
        std::cout<<std::endl;
        return true;
      }
    
      LOG_WARNING("Cannot handle " << msg)
      return mc_control::fsm::Controller::read_msg(msg);

    }




    //////////////
    //
    // Handover Controller read_write_msg
    //
    //////////////    
    bool HandoverController::read_write_msg(std::string & msg, std::string & out)
    {
      // out = msg;
      // return true;
      return mc_control::fsm::Controller::read_write_msg(msg, out);
    }

   
} //namespace mc_control

CONTROLLER_CONSTRUCTOR("Handover", mc_handover::HandoverController)
