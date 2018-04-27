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


        Eigen::VectorXd dimW(3);
        dimW << 0., 0., 2.;//0., 0., 1.;
        
        // //////////////////////// RIGHT ARM ///////////////////// //

        // set RArm wrist jointGain
        // postureTask->jointsGains(robots().mbs(), {{"RARM_JOINT5", 1e3}});
        // qpsolver->addTask(postureTask.get());
  
        efTaskR.reset(new mc_tasks::EndEffectorTask("RARM_LINK7", robots(), robots().robotIndex(),5.0,1e3));
        oriTaskR.reset(new mc_tasks::OrientationTask("RARM_LINK7", robots(), robots().robotIndex(),9.0,1e2));

        // elbow_posR.reset(new mc_tasks::PositionTask("RARM_LINK3", robots(), 0, 1.0, 1e4));
        // elbow_posR->dimWeight(dimW);

        // //////////////////////// LEFT ARM ///////////////////// //

        // set LArm wrist jointGain
        // postureTask->jointsGains(robots().mbs(), {{"LARM_JOINT5", 1e3}});
        // qpsolver->addTask(postureTask.get());

        efTaskL.reset(new mc_tasks::EndEffectorTask("LARM_LINK7", robots(), robots().robotIndex(),5.0,1e3));
        oriTaskL.reset(new mc_tasks::OrientationTask("LARM_LINK7", robots(), robots().robotIndex(),9.0,1e2));

        // elbow_posL.reset(new mc_tasks::PositionTask("LARM_LINK3", robots(), 0, 1.0, 1e4));
        // elbow_posL->dimWeight(dimW);


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
        std::cout << "q[0] ";
        // for(const auto & qi : q[0])
        // {
        //   std::cout << qi << ", ";
        // }
        // std::cout << std::endl;
        q[0] = {0.991445, -0, -0, 0.130526, -0.275, -0.05, 0.825336};
        MCController::reset({q});

        qpsolver->setContacts({
        mc_rbdyn::Contact(robots(), "LFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "RFullSole", "AllGround"),
        mc_rbdyn::Contact(robots(), "Butthock", "AllGround"),
        mc_rbdyn::Contact(robots(), "LowerBack","AllGround")
        });

        efTaskL->reset();
        // elbow_posL->reset();
      
        efTaskR->reset();
        // elbow_posR->reset();

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
      init_pos();

      return ret;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                        //
    //                            MCHandoverController:init_pos()                             //
    //                                                                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////
    void MCHandoverController::init_pos()
    {
      /* Set the hands in their initial configuration */
      // std::vector<double> rArm_Joints_q =
      // // {-0.86, -0.37, 0.59, -0.56, -1.6, 0.46, -0.04, -0.0};
      // {-0.4, -0.55, 0.55, -0.65, -1.2, 0.37, -0.1, -0.00}; // old good working pose


      // std::vector<double> lArm_Joints_q =
      // // {-0.86, 0.28, -0.48, -0.68, 1.37, -0.21, -0.04, 0.02};
      // {-0.4, -0.55, 0.55, -0.65, -1.2, 0.37, -0.1, -0.00};

      // for(size_t i = 0; i < 8; ++i)
      // {
      //   std::stringstream ss1,  ss2;
      //   ss1 << "RARM_JOINT" << i;
      //   set_joint_pos(ss1.str(), rArm_Joints_q[i]);
      //   ss2 << "LARM_JOINT" << i;
      //   set_joint_pos(ss2.str(), lArm_Joints_q[i]);
      // }
      // solver().addTask(efTaskL);
      // solver().addTask(efTaskR);

      // Eigen::Matrix3d initOriR;
      // initOriR << -0.295298,  0.338036,  3.0605,
      //             -0.758028,  0.486418, -0.4345,
      //             -0.581542, -0.805685,  0.112602;

      Eigen::Vector3d initPosR;
      initPosR <<  0.35, -0.18, 1.2;

      sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];
      // efTaskR->set_ef_pose(sva::PTransformd(sva::RotX(1./2.*M_PI)*BodyW.rotation(), initPosR));
      
      efTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosR));
      // efTaskR->add_ef_pose(sva::PTransformd(sva::RotX(-(M_PI/180)*45)*BodyW.rotation(), initPosR));
      
      solver().addTask(efTaskR);

      
 

       // sva::RotX(M_PI/1.0 - phi)*sva::RotY(phi);

      // Eigen::Matrix3d initOriL;
      // initOriL <<  -0.295298,  0.338036,  3.0605,
      //             -0.758028,  0.486418, -0.4345,
      //             -0.581542, -0.805685,  0.112602;

      // Eigen::Vector3d initPosL;
      // initPosL <<  0.392299, 0.508818, 1.1509;
      // efTaskL->set_ef_pose(sva::PTransformd(initOriL, initPosL));

    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    //                                                                                        //
    //                      MCHandoverController::createWaypoints()                           //
    //                                                                                        //
    ////////////////////////////////////////////////////////////////////////////////////////////
    void MCHandoverController::createWaypoints()
    {

      std::cout << " producing way points " << std::endl << std::endl;

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
        efTaskR->set_ef_pose(X);
        solver().addTask(efTaskR);

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
