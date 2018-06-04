
#include "ForceControlGrippersStep.h"

namespace mc_handover
{
	namespace states {

		void ForceControlGrippersStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config" << endl;

			ForceThresh_eval_ = config("ForceThresh_eval", 0.005);
			ForceThresh_speed_ = config("ForceThresh_speed", 0.005);
			
		}
		void ForceControlGrippersStep::start(mc_control::fsm::Controller & controller)
		{	

			cout << "start -- ForceControlGrippersStep " << endl;
    		// auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			// /* handover gui elements */ 
			// ctl.gui()->addElement({"HandoverElements"},			
			// mc_rtc::gui::Button("right hand force sensor readings",
			// 				 [&ctl]() { cout <<"right hand force sensor" << ctl.wrenches.at("RightHandForceSensor").force().transpose() << endl; }),

			// mc_rtc::gui::Button("left hand force sensor readings",
			// 				 [&ctl]() { cout <<"left hand force sensor" << ctl.wrenches.at("LeftHandForceSensor").force().transpose() << endl; })
			// );

		}

		bool ForceControlGrippersStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			cout << "run" << endl;
		
			// ctl.wrenches.at("RightHandForceSensor").force() = Eigen::Vector3d (4,5,6);

			// ctl.wrenches.at("LeftHandForceSensor").force() = Eigen::Vector3d (1,2,3);


			// ctl.wrenches["RightHandForceSensor"] =
			//         ctl.robots().robot().forceSensor("RightHandForceSensor").wrench();          
			// ctl.wrenchRt = ctl.wrenches.at("RightHandForceSensor");
			// ctl.wrenches.at("RightHandForceSensor").couple() << ctl.wrenchRt.couple()[2], ctl.wrenchRt.couple()[1], -ctl.wrenchRt.couple()[0];
			// ctl.wrenches.at("RightHandForceSensor").force() << ctl.wrenchRt.force()[2], ctl.wrenchRt.force()[1], -ctl.wrenchRt.force()[0];
			
			// cout <<"right hand force sensor" << ctl.wrenches.at("RightHandForceSensor").force().transpose() << endl;



			// ctl.wrenches["LeftHandForceSensor"] =
			//         ctl.robots().robot().forceSensor("LeftHandForceSensor").wrench();          
			// ctl.wrenchRt = ctl.wrenches.at("LeftHandForceSensor");
			// ctl.wrenches.at("LeftHandForceSensor").couple() << ctl.wrenchRt.couple()[2], ctl.wrenchRt.couple()[1], -ctl.wrenchRt.couple()[0];
			// ctl.wrenches.at("LeftHandForceSensor").force() << ctl.wrenchRt.force()[2], ctl.wrenchRt.force()[1], -ctl.wrenchRt.force()[0];

			// cout <<"left hand force sensor" << ctl.wrenches.at("LeftHandForceSensor").force().transpose() << endl;


	        output("OK");	       

		    return true;
		}

		void ForceControlGrippersStep::teardown(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
		
			cout << "teardown" << endl;

			ctl.gui()->removeElement({"HandoverElements"}, "openGrippers");
  			ctl.gui()->removeElement({"HandoverElements"}, "closeGrippers");
			
		}


	} // namespace states

} // namespace mc_torquing_controller
		