
#include "ForceControlGrippersStep.h"

namespace mc_handover
{
	namespace states {

		void ForceControlGrippersStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config" << endl;

			thresh         = config("handsWrenchTh");
			handsWrenchDir = config("handsWrenchDir");
			
		}
		void ForceControlGrippersStep::start(mc_control::fsm::Controller & controller)
		{
			cout << "start -- ForceControlGrippersStep " << endl;
    		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
        

			/* handover gui elements */
			ctl.gui()->addElement({"FSM", "HandoverElements"},

				// mc_rtc::gui::NumberSlider("move COM pos",
				// 	[this](){ return move[0]; },
				// 	[this](double v) { move[0] = v; },
				// 	-10.0, 10.0),

				mc_rtc::gui::Button("publish_current_wrench", [&ctl]() {  
				std::cout << "left hand wrench:: Torques, Forces " <<
				ctl.wrenches.at("LeftHandForceSensor")/*.force().transpose()*/ << endl;
				std::cout << "right hand wrench:: Torques, Forces " <<
				ctl.wrenches.at("RightHandForceSensor")/*.force().transpose()*/ << endl;
				}),

				mc_rtc::gui::ArrayInput("Move Com Pos", {"x", "y", "z"},
                      					[this]() { return move; },
                      					[this](const Eigen::Vector3d v) { move = v;
                      					cout << " com pos set to:\n" << initialCom + move << endl;}),


        mc_rtc::gui::ArrayInput("Threshold",
                                {"Left cx", "cy", "cz", "fx", "fy", "fz", "Right cx", "cy", "cz", "fx", "fy", "fz"},
                                [this]() { return thresh; },
                                [this](const Eigen::VectorXd & t)
                                {
                                  LOG_INFO("Changed threshold to:\nLeft: " << t.head(6).transpose() << "\nRight: " << t.tail(6).transpose() << "\n")
                                  thresh = t;
                                })
    );

			/*add com task -- position it lower and bit backward */
			comTask = std::make_shared<mc_tasks::CoMTask>
				(ctl.robots(), ctl.robots().robotIndex(), 2., 10000.);
			// comTask->dimWeight(Eigen::Vector3d(1., 1., 1.));
			ctl.solver().addTask(comTask);
			initialCom = rbd::computeCoM(ctl.robot().mb(),ctl.robot().mbc());
						
    		cout << "initial_com X " << endl << initialCom[0] << endl;
    		cout << "initial_com Y " << endl << initialCom[1] << endl;
    		cout << "initial_com Z " << endl << initialCom[2] << endl;

      ctl.runOnce = true;
		}




		bool ForceControlGrippersStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/* set com pose */
			target = initialCom + move;
			comTask->com(target);

      auto leftTh = thresh.head(6);
      auto rightTh = thresh.tail(6);
      auto leftForce = ctl.wrenches.at("LeftHandForceSensor").force();
      auto rightForce = ctl.wrenches.at("RightHandForceSensor").force();

      auto open_grippers = [&]()
      {
        ctl.publishWrench();
        auto gripper = ctl.grippers["l_gripper"].get();
        gripper->setTargetQ({openGrippers});

        gripper = ctl.grippers["r_gripper"].get();
        gripper->setTargetQ({openGrippers});

        output("Repeat");
        ctl.runOnce = false;
      };

      auto compareLogic = [&](const char * axis_name, int idx)
      {
        if(fabs(leftForce[idx]) > leftTh[idx+3])
        {
          if(fabs(rightForce[idx]) > rightTh[idx+3])
          {
            LOG_INFO("Opening grippers, threshold on " << axis_name << " reached on both hands")
          }
          else
          {
            LOG_INFO("Opening grippers, threshold on " << axis_name << " reached on left hand only")
          }
          open_grippers();
          return true;
        }
        else
        {
          if(fabs(rightForce[idx]) > rightTh[idx+3])
          {
            LOG_INFO("Opening grippers, threshold on " << axis_name << " reached on right hand only")
            open_grippers();
            return true;
          }
          return false;
        }
      };

      if(ctl.runOnce)
      {
        return compareLogic("x-axis", 0) || compareLogic("y-axis", 1) || compareLogic("z-axis", 2);
      }
      else
      {
        return true;
      }

		}


		void ForceControlGrippersStep::teardown(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
			
			ctl.solver().removeTask(comTask);

			ctl.gui()->removeElement({"FSM", "HandoverElements"},"publish_current_wrench");		
			ctl.gui()->removeElement({"FSM", "HandoverElements"}, "Move Com Pos");
			ctl.gui()->removeElement({"FSM", "HandoverElements"}, "Threshold");

		}


	} // namespace states

} // namespace mc_torquing_controller