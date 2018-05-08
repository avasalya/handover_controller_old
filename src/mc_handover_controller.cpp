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
          mc_rbdyn::Collision("LARM_LINK5", "CHEST_LINK1", 0.1, 0.05, 0.)
        });


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
  
        rEfTaskR.reset(new mc_tasks::RelativeEndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(), "", 10.0,1e3));
        oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK7", robots(), robots().robotIndex(),9.0,1e2));

        rEfTaskL.reset(new mc_tasks::RelativeEndEffectorTask("LARM_LINK7", robots(), robots().robotIndex(), "", 10.0,1e3));
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
        auto q = reset_data.q;
        // q[0] = {0.991445, -0, -0, 0.130526, -0.275, -0.05, 0.825336};
        MCController::reset({q});

        qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
        mc_rbdyn::Contact(robots(), "LowerBack","AllGround")
        });

        rEfTaskL->reset();
        rEfTaskR->reset();

        comTask->reset();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                        //
    //                              MCHandoverController::run()                               //
    //                                                                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////
    
    bool onlyOnce = true;
    bool MCHandoverController::run()
    {

      bool ret = MCController::run();
      
      if (onlyOnce)
      {
        onlyOnce = false;
        init_pos();
      }

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

      initPosR <<  0.30, -0.35, 0.45;
      rEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosR));
      solver().addTask(rEfTaskR);


      initPosL <<  0.30, 0.35, 0.45;      
      rEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosL));
      solver().addTask(rEfTaskL);
        
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
      std::stringstream ss;
      ss << msg;
      
      std::string token;
      ss >> token;

     

      if(token == "initial2")
      { 

          // Eigen::Vector3d t( 0.66, -0.23, 0.27 );
          // Eigen::Matrix3d m;
          // m << -1, 1, -1, 0, 1, 0, 1, 1, -1;
          // MCController::change_ef("LARM_JOINT6");
          // MCController::move_ef(t, m);

          // std::cout<< "I am working here " << std::endl;
          // postureTaskL.reset(new mc_tasks::PostureTask(solver(), robots().robotIndex(), 10, 1e3));

          // // postureTaskL->selectActiveJoints(solver(), "LARM_JOINT6");

          // unsigned int wrist_i = robot().jointIndexByName("LARM_JOINT6");
          // auto p = postureTaskL->posture();
          // p[wrist_i][0] = 1; //yaw;
          // postureTaskL->posture(p);
          // qpsolver->addTask(postureTask.get());

        // Eigen::Vector3d t( 0.66, -0.23, 0.27 );
        // sva::PTransformd dtr(Eigen::Matrix3d::Identity(), t);
        // rEfTaskL->add_ef_pose(dtr);

          return true;
      }

      // set initial pose
      if(token == "initial")
      {
        ///HEAD orientation in the beginning of exp
        // MCController::set_joint_pos("HEAD_JOINT0", -0.3); //-ve to move head to robot's right
        MCController::set_joint_pos("HEAD_JOINT1",  0.4); //+ve to move head down

        ///initial orientation and pos in the beginning of exp
        init_pos();
        // createWaypoints();

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



 // very old init pos   // doesn't replicate accurately 
      // if(token == "init")  // set initial coordinates and orientation in the beginning of exp
      // {                     // w,   x,      y,    z
      //   Eigen::Quaterniond q(0.55, 0.51, -0.52, -0.4); q.normalize(); //0.59,-0.008, 0.059, 0.80
      //   Eigen::Vector3d t( 0.66, -0.23, 0.27 );
      //   double z = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")].translation().z();
      //   t.z() +=z;
      //   sva:: PTransformd X(q.inverse(), t);
      //   rEfTaskR->set_ef_pose(X);
      //   solver().addTask(rEfTaskR); 

      //   return true;
      // }


      /*sitting positon*/
      // if(token == "sit")
      // {
      //    MCController::set_joint_pos("RLEG_JOINT2",-1.7);
      //    MCController::set_joint_pos("LLEG_JOINT2",-1.7);

      //    MCController::set_joint_pos("RLEG_JOINT3", 1.6);
      //    MCController::set_joint_pos("LLEG_JOINT3", 1.6);

      //    MCController::set_joint_pos("RLEG_JOINT4", 0.0);
      //    MCController::set_joint_pos("LLEG_JOINT4", 0.0);

      //    return true;
      // }



  