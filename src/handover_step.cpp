// //
// /// ... mc_handover_complianceTask ...
// ///

// #include "handover_step.h"

// namespace mc_control
// {
// 	Step::Step(const std::string & name) : name(name)
// 	{}


// 	HandoverStep * HandoverStep::update(HandoverController & ctl)
// 	{
// 		if(firstCall)
// 		{
// 			init_(ctl);
// 			firstCall = false;
// 			return this;
// 		}
// 		return update_(ctl);
// 	}

// 	void InitStep::init_(HandoverController & ctl)
// 	{
// 		ctl.comTask = std::make_shared<mc_tasks::CoMTask>(ctl.robots(),
// 		ctl.robots().robotIndex(), 3., 100.);
// 		auto mbc = ctl.robot().mbc();
// 		mbc.q = ctl.postureTask->posture();
// 		auto comT = rbd::computeCoM(ctl.robot().mb(), mbc);
// 		ctl.comTask->set_com(comT);
// 		ctl.comTask->addToSolver(ctl.solver());
// 	}


// 	HandoverStep * InitStep::update_(HandoverController & ctl)
// 	{
// 		if(ctl.comTask->comTask->speed().norm() < 1e-2)
// 		{
// 			return new InitialHandPoseStep;
// 		}
// 		return this;
// 	}


// 	void InitialHandPoseStep::init_(HandoverController & ctl)
// 	{
	
// 		Eigen::Vector3d initPosR, initPosL;
// 		sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];

// 		initPosR <<  0.30, -0.35, 0.45;
// 		ctl.relEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosR));
// 		ctl.solver().addTask(ctl.relEfTaskR);


// 		initPosL <<  0.30, 0.35, 0.45;      
// 		ctl.relEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosL));
// 		ctl.solver().addTask(ctl.relEfTaskL);     

// 	}

// 	HandoverStep * InitialHandPoseStep::update_(HandoverController & ctl)
// 	{
// 		if(
// 			ctl.relEfTaskL->speed().norm() < 1e-2 && 
// 			ctl.relEfTaskL->eval().norm() < 0.5 &&
// 			ctl.relEfTaskR->speed().norm() < 1e-2 && 
// 			ctl.relEfTaskR->eval().norm() < 0.5
// 			)
// 		{
// 			return GrippersOpenStep;
// 		}
// 		return this;
// 	}


// 	void GrippersOpenStep::init_(HandoverController & ctl)
// 	{
// 		targetOpen =  1.;
// 		openSpeed  =  0.002;
// 	}

// 	HandoverStep * GrippersOpenStep::update_(HandoverController & ctl)
// 	{
// 		auto gripperL = ctl.grippers.at("l_gripper");		
// 		auto gripperR = ctl.grippers.at("r_gripper");

// 		if(gripperL->percentOpen[0] >= targetOpen && gripperR->percentOpen[0] >= targetOpen)
// 		{
// 			gripperL->percentOpen[0] = targetOpen;
// 			gripperR->percentOpen[0] = targetOpen;
// 			return new AddGrippersStep;
// 		}
// 		else
// 		{
// 			gripperR->percentOpen[0] += openSpeed;
// 			gripperL->percentOpen[0] += openSpeed;
// 			return this;
// 		}		
// 	}


// 	void AddGrippersStep::init_(HandoverController & ctl)
// 	{	
// 		ctl.relEfTaskL->removeFromSolver(ctl.solver());
// 		ctl.relEfTaskR->removeFromSolver(ctl.solver());

// 		const mc_rbdyn::ForceSensor & forceSensorL = ctl.robot().forceSensorCalibData("LeftHandForceSensor");
//     	const mc_rbdyn::ForceSensor & forceSensorR = ctl.robot().forceSensorCalibData("RightHandForceSensor");

// 		complianceTaskL = std::shared_ptr<mc_tasks::ComplianceTask>(new mc_tasks::ComplianceTask(ctl.robots(), ctl.robots().robotIndex(), forceSensorL, ctl.getWrenches(), ctl.calibrator, ctl.timeStep, 3., 1e3));
//   		complianceTaskL->setTargetWrench(sva::ForceVecd(Eigen::Vector3d(0., 0., 0.),Eigen::Vector3d(0., 0., 50.)));
//   		complianceTaskL->addToSolver(ctl.solver());

//   		complianceTaskR = std::shared_ptr<mc_tasks::ComplianceTask>(new mc_tasks::ComplianceTask(ctl.robots(), ctl.robots().robotIndex(), forceSensorR, ctl.getWrenches(), ctl.calibrator, ctl.timeStep, 3., 1e3));
//   		complianceTaskR->setTargetWrench(sva::ForceVecd(Eigen::Vector3d(0., 0., 0.),Eigen::Vector3d(0., 0., 50.)));
//   		complianceTaskR->addToSolver(ctl.solver());
// 	}

// 	HandoverStep * AddGrippersStep::update_(HandoverController & ctl)
// 	{
// 		complianceTaskL->update();
// 		complianceTaskR->update();
// 		if(
// 			complianceTaskL->speed().norm < 5e-3 &&
// 			complianceTaskL->eval().norm() < 25 &&
// 			complianceTaskR->speed().norm < 5e-3 &&
// 			complianceTaskR->eval().norm() < 25
// 			)
// 		{
// 			complianceTaskL->removeFromSolver(ctl.solver());
// 			complianceTaskR->removeFromSolver(ctl.solver());
// 			return new CloseGrippersStep;
// 		}
// 		return this;
// 	}
	

// 	void CloseGrippersStep::init_(HandoverController & ctl)
// 	{
// 		targetClose =  -0.25;
// 		CloseSpeed  =  0.002;
// 	}

// 	HandoverStep * CloseGrippersStep::update_(HandoverController & ctl)
// 	{
// 		// auto gripperL = ctl.grippers.at("l_gripper");		
// 		// auto gripperR = ctl.grippers.at("r_gripper");

// 		// if(gripperL->percentOpen[0] <= targetOpen && gripperR->percentOpen[0] <= targetOpen)
// 		// {
// 		// 	gripperL-> targetClose;
// 		// 	gripperR-> targetClose;
// 		// 	return new AdjustHandsStep;
// 		// }
// 		// else
// 		// {
// 		// 	gripperR->percentOpen[0] += openSpeed;
// 		// 	gripperL->percentOpen[0] += openSpeed;
// 		// 	return this;
// 		// }


// 		//if( wrenchL >= && wrenchR >= )
// 		//{
// 			auto gripper = grippers["l_gripper"].get();
// 	        gripper->setTargetQ({-0.25});
// 	        gripper = grippers["r_gripper"].get();
// 	        gripper->setTargetQ({-0.25});
// 	        return new AdjustHandsStep;
//     	//}
// 	    // else
// 	    // {
// 	    	 // return this;
// 	    // }
// 	}





// } // namespace mc_control




// 	// void AdjustHandsStep::init_(HandoverController & ctl)
// 	// {
// 	// 	forceSensorL = ctl.robot().forceSensorData("RightHandForceSensor");


// 	// 	Eigen::Vector3d initPosR, initPosL;
// 	// 	sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];

// 	// 	initPosR <<  0.30, -0.35, 0.45;
// 	// 	ctl.relEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosR));
// 	// 	ctl.solver().addTask(ctl.relEfTaskR);


// 	// 	initPosL <<  0.30, 0.35, 0.45;      
// 	// 	ctl.relEfTaskL->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosL));
// 	// 	ctl.solver().addTask(ctl.relEfTaskL);     

// 	// }

//  //    HandoverStep * AdjustHandsStep::update_(HandoverController & ctl)
//  //    {

//  //    }

//  //    void ExperimentStep::init_(HandoverController & ctl)
//  //    {
//  //    	const mc_rbdyn::ForceSensor& forceSensorL = ctl.robot().forceSensorData("LeftHandForceSensor");
//  //    	const mc_rbdyn::ForceSensor& forceSensorR = ctl.robot().forceSensorData("RightHandForceSensor");
//  //    }

//  //    HandoverStep * ExperimentStep::update_(HandoverController & ctl)
//  //    {

//  //    }