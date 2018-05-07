//
/// ... mc_handover_controller ...
///

#include "mc_handover_controller.h"

namespace mc_control
{
    using namespace rbd;

    ////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                        //
    //                      MCHandoverController::MCHandoverController()                      //
    //                                                                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////
    MCHandoverController::MCHandoverController(const std::shared_ptr<mc_rbdyn::RobotModule> & robot_module, const double & dt):MCController(robot_module, dt)
    {
      
        // add collision ConstraintSet
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK3", "LLEG_LINK2", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK4", "LLEG_LINK2", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK6", "LLEG_LINK2", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK3", "LLEG_LINK3", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK4", "LLEG_LINK3", 0.1, 0.05, 0});
        selfCollisionConstraint.addCollision(solver(), {"LARM_LINK6", "LLEG_LINK3", 0.1, 0.05, 0});

        qpsolver->addConstraintSet(contactConstraint);
        qpsolver->addConstraintSet(kinematicsConstraint);
        qpsolver->addConstraintSet(selfCollisionConstraint);

      
        qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
        mc_rbdyn::Contact(robots(), "LowerBack","AllGround")
        });


        // //////////////////////// RIGHT ARM ///////////////////// //
  
        rEfTaskR.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(), "", 2.0,1e3));
        oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK7", robots(), robots().robotIndex(),9.0,1e2));

        rEfTaskL.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK7", robots(), robots().robotIndex(), "", 2.0,1e3));
        oriTaskL.reset(new mc_tasks::OrientationTask("LARM_LINK7", robots(), robots().robotIndex(),9.0,1e2));

        qpsolver->addTask(postureTask.get());

        comTask.reset(new mc_tasks::CoMTask(robots(), robots().robotIndex()));
        solver().addTask(comTask);

        LOG_SUCCESS("mc_handover_controller init done")
    }


    ////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                        //
    //                            MCHandoverController::reset()                               //
    //                                                                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////
    void MCHandoverController::reset(const ControllerResetData & reset_data)
    {
        // auto q = reset_data.q;
        // q[0] = {0.991445, -0, -0, 0.130526, -0.275, -0.05, 0.825336};
        // MCController::reset({q});

        qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
        mc_rbdyn::Contact(robots(), "LowerBack","AllGround")
        });

        // efTaskL->reset();
        // efTaskR->reset();
        rEfTaskL->reset();
        rEfTaskR->reset();

        comTask->reset();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                        //
    //                              MCHandoverController::run()                               //
    //                                                                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////
    bool MCHandoverController::run()
    {

      bool ret = MCController::run();
      // init_pos();


      return ret;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                        //
    //                            MCHandoverController:init_pos()                             //
    //                                                                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////
    void MCHandoverController::init_pos()
    {
      
      Eigen::Vector3d initPosR, initPosL;
      sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];

      initPosR <<  0.30, -0.35, 0.2;
      rEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosR));
      solver().addTask(rEfTaskR);

      // std::cout << initPosR+BodyW.translation()<< "\n"<< std::endl;

      
      initPosL <<  0.30, 0.35, 0.2;      
      rEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosL));
      solver().addTask(rEfTaskL);

      // std::cout << initPosL+BodyW.translation()<< "\n"<< std::endl;
        

    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                        //
    //                      MCHandoverController::createWaypoints()                           //
    //                                                                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////
    void MCHandoverController::createWaypoints()
    {

      // std::cout << " producing way points " << std::endl << std::endl;

      int sample = 10;

      for (int i=0; i<sample; i++)
      {
        mjObj.produceWp(MatrixXd::Random(sample,3), MatrixXd::Random(sample,3), i, sample);
        // std::cout << mjObj.yPos[i] << std::endl; // to acess any produced wp
      }
    }


    ////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                        //
    //                      MCHandoverController::read_msg()                                  //
    //                                                                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////
    bool MCHandoverController::read_msg(std::string & msg)
    {
      std::stringstream ss; ss << msg;
      std::string token;
      ss >> token;

      /* very old init pos */  // doesn't replicate accurately 
      if(token == "init")  // set initial coordinates and orientation in the beginning of exp
      {                     // w,   x,      y,    z
        Eigen::Quaterniond q(0.55, 0.51, -0.52, -0.4); q.normalize(); //0.59,-0.008, 0.059, 0.80
        Eigen::Vector3d t( 0.66, -0.23, 0.27 );
        double z = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")].translation().z();
        t.z() +=z;
        sva:: PTransformd X(q.inverse(), t);
        rEfTaskR->set_ef_pose(X);
        solver().addTask(rEfTaskR); 

        return true;
      }


      /*sitting positon*/
      if(token == "sit")
      {
         MCController::set_joint_pos("RLEG_JOINT2",-1.7);
         MCController::set_joint_pos("LLEG_JOINT2",-1.7);

         MCController::set_joint_pos("RLEG_JOINT3", 1.6);
         MCController::set_joint_pos("LLEG_JOINT3", 1.6);

         MCController::set_joint_pos("RLEG_JOINT4", 0.0);
         MCController::set_joint_pos("LLEG_JOINT4", 0.0);

         return true;
      }

      // set initial pose
      if(token == "initial")
      {
        ///HEAD orientation in the beginning of exp
        MCController::set_joint_pos("HEAD_JOINT0", -0.3); //-ve to move head to robot's right
        MCController::set_joint_pos("HEAD_JOINT1",  0.4); //+ve to move head down

        ///initial orientation and pos in the beginning of exp
        init_pos();
        createWaypoints();

        return true;
      }
      /// open right gripper
      if(token == "open_rgripper")
      {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({0.5});
        return true;
      }
      /// close right gripper
      if(token == "close_rgripper")
      {
        auto gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({-0.25});
        return true;
      }


      /// open left gripper
      if(token == "open_lgripper")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({0.5});
        return true;
      }
      /// close left gripper
      if(token == "close_lgripper")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({-0.25});
        return true;
      }


      /// open both grippers
      if(token == "open_grippers")
      {
        auto gripper = grippers["l_gripper"].get();
        gripper->setTargetQ({0.5});
        gripper = grippers["r_gripper"].get();
        gripper->setTargetQ({0.5});
        return true;
      }
      /// close both grippers
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
          std::stringstream ss;
          ss << "RARM_JOINT" << i;
          std::cout << robot().mbc().q[robot().jointIndexByName(ss.str())][0] << ", ";
        }
        std::cout<<std::endl;
        return true;
      }
    
      LOG_WARNING("Cannot handle " << msg)
      return false;
    }
}
