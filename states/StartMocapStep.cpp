/*------------------------------TO DOs----------------------------------------------
--- if(wp_efL_objMarkerA-predictPos).eval().norm()> xxx -- pick another closet point on line
--- when to start handover motion
--- when should prediction be started again?
----------------------------------------------------------------------------------*/

#include "StartMocapStep.h"

namespace mc_handover
{

	namespace states
	{

		void StartMocapStep::configure(const mc_rtc::Configuration & config)
		{
			// config("closeGrippers", closeGrippers);
			// config("closeGrippers", openGrippers);
			thresh         = config("handsWrenchTh");
			handsWrenchDir = config("handsWrenchDir");
		}


		void MyErrorMsgHandler(int iLevel, const char *szMsg)
		{
			const char *szLevel = NULL;

			if (iLevel == VL_Debug) {
				szLevel = "Debug";
			} else if (iLevel == VL_Info) {
				szLevel = "Info";
			} else if (iLevel == VL_Warning) {
				szLevel = "Warning";
			} else if (iLevel == VL_Error) {
				szLevel = "Error";
			}
			printf("  %s: %s\n", szLevel, szMsg);
		}


		void StartMocapStep::start(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/*initialization*/
			{
				/*close grippers for safety*/
				auto  gripperL = ctl.grippers["l_gripper"].get();
				auto  gripperR = ctl.grippers["r_gripper"].get();
				gripperL->setTargetQ({closeGrippers});
				gripperR->setTargetQ({closeGrippers});


				/*chest task*/
				chestPosTask.reset(new mc_tasks::PositionTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
				chestOriTask.reset(new mc_tasks::OrientationTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
				ctl.solver().addTask(chestPosTask);
				ctl.solver().addTask(chestOriTask);
				

				/*position Task*/
				ctl.posTaskL = std::make_shared<mc_tasks::PositionTask>("LARM_LINK7", ctl.robots(), ctl.robots().robotIndex(), 5.0, 1000);
				// ctl.posTaskR = std::make_shared<mc_tasks::PositionTask>("RARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), 5.0, 1000);
				// cout << "posTaskL" << ctl.posTaskL->position().transpose()<<endl;


				/*publish wrench*/
				ctl.gui()->addElement({"Handover", "wrench"},
					
					mc_rtc::gui::Button("publish_current_wrench", [&ctl]() {  
						std::cout << "left hand wrench:: Torques, Forces " <<
						ctl.wrenches.at("LeftHandForceSensor")/*.force().transpose()*/ << endl;
						std::cout << "right hand wrench:: Torques, Forces " <<
						ctl.wrenches.at("RightHandForceSensor")/*.force().transpose()*/ << endl;
					}),

					mc_rtc::gui::ArrayInput("Threshold",
						{"Left cx", "cy", "cz", "fx", "fy", "fz", "Right cx", "cy", "cz", "fx", "fy", "fz"},
						[this]() { return thresh; },
						[this](const Eigen::VectorXd & t)
						{
							LOG_INFO("Changed threshold to:\nLeft: " << t.head(6).transpose() << "\nRight: " << t.tail(6).transpose() << "\n")
							thresh = t;
						}));


				/*move object using cursor or simData*/
				ctl.gui()->addElement({"Handover","move object"},
					mc_rtc::gui::Transform("Position", 
						[this,&ctl](){ return ctl.robots().robot(2).bodyPosW("base_link"); },
						[this,&ctl](const sva::PTransformd & pos) { 
							ctl.robots().robot(2).posW(pos);
							ctl.removeContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"});
							ctl.addContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"});
						})
					// mc_rtc::gui::Button("Replay", [this](){ i = 0;}),
					// mc_rtc::gui::Point3D("log data", [this,&ctl](){ ctl.robots().robot(2).posW({objectBodyMarker}); return objectBodyMarker;})
					);


				/*com Task*/
				ctl.gui()->addElement({"Handover", "com"},

					mc_rtc::gui::ArrayInput("Move Com Pos", {"x", "y", "z"},
						[this]() { return move; },
						[this](const Eigen::Vector3d v) { move = v;
							cout << " com pos set to:\n" << initialCom + move << endl;})
					);

				comTask = std::make_shared<mc_tasks::CoMTask>
				(ctl.robots(), ctl.robots().robotIndex(), 10., 1000.);//10, 1e3
				// comTask->dimWeight(Eigen::Vector3d(1., 1., 1.));

				initialCom = rbd::computeCoM(ctl.robot().mb(),ctl.robot().mbc());
				comTask->com(initialCom);
				ctl.solver().addTask(comTask);


				/*JUST FOR CREATING MOCAP TEMPLATE*/
				ctl.solver().addTask(ctl.posTaskL);

				ctl.gui()->addElement({"MOCAP", "temp"},
					mc_rtc::gui::Button( "init", [&ctl](){ ctl.posTaskL->position({0.06,0.37,0.72});
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({0.0}); } ),
					mc_rtc::gui::Button( "pos1", [&ctl](){ ctl.posTaskL->position({0.5,0.3,1.1});
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({0.5}); } ),
					mc_rtc::gui::Button( "pos2", [&ctl](){ ctl.posTaskL->position({0.3,0.5,0.9});
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({0.5});  } ),
					mc_rtc::gui::Button( "pos3", [&ctl](){ ctl.posTaskL->position({0.6,0.2,1.2}); 
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({0.5}); } ),
					mc_rtc::gui::Button( "pos4", [&ctl](){ ctl.posTaskL->position({0.3,0.3,1.3}); 
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({0.5}); } ),
					mc_rtc::gui::Button( "pos5", [&ctl](){ ctl.posTaskL->position({0.55,0.4,1.0}); 
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({0.5}); } ),
					mc_rtc::gui::Button( "pos6", [&ctl](){ ctl.posTaskL->position({0.35,0.2,1.1}); 
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({0.5}); } ) );
			}// initialization


			/*configure MOCAP*/
			if(Flag_CORTEX)
			{
				Cortex_SetVerbosityLevel(VL_Info);
				Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);

				retval = Cortex_Initialize("10.1.1.200", "10.1.1.190");
				// retval = Cortex_Initialize("10.1.1.180", "10.1.1.190"); //for robot local PC

				if (retval != RC_Okay)
				{
					printf("Error: Unable to initialize ethernet communication\n");
					retval = Cortex_Exit();
				}

				// cortex frame rate //
				printf("\n****** Cortex_FrameRate ******\n");
				retval = Cortex_Request("GetContextFrameRate", &pResponse, &nBytes);
				if (retval != RC_Okay)
					printf("ERROR, GetContextFrameRate\n");
				float *contextFrameRate = (float*) pResponse;
				printf("ContextFrameRate = %3.1f Hz\n", *contextFrameRate);


				// get name of bodies being tracked and its set of markers //
				printf("\n****** Cortex_GetBodyDefs ******\n");
				pBodyDefs = Cortex_GetBodyDefs();
				if (pBodyDefs == NULL) 
				{
					printf("Failed to get body defs\n");
				} 
				else 
				{  
					totalBodies = pBodyDefs->nBodyDefs;
					cout << "total no of bodies tracked " << totalBodies << endl;
					for(int iBody=0; iBody<totalBodies; iBody++)
					{
						bodyMarkers.push_back(pBodyDefs->BodyDefs[iBody].nMarkers);
						pBody = &pBodyDefs->BodyDefs[iBody];
						cout << "number of markers defined in body " << iBody+1 << " (\"" << pBody->szName << "\") : " << bodyMarkers.at(iBody) << endl;    

						for (int iMarker=0 ; iMarker<pBody->nMarkers; iMarker++)
						{
							cout << iMarker << " " << pBody->szMarkerNames[iMarker] << endl;
						}
					}
				}
				printf("\n*** start live mode ***\n");
				Cortex_Request("LiveMode", &pResponse, &nBytes);


				/*allocate memory for mocap markers*/
				markersPos.resize(pBody->nMarkers);
				for(int m=0; m<pBody->nMarkers; m++)
				{
					markersPos[m] = Eigen::MatrixXd::Zero(3,60000);
				}
			}
			else /*simData*/
			{
				startCapture = true;
				
				name = {"simData2"};

				std::string fn = std::string(DATA_PATH) + "/" + name + ".txt";
				std::ifstream file(fn);

				if(!file.is_open())
				{
					LOG_ERROR("Failed to open ")
				}

				while(file >> pt)
				{
					pts.push_back(pt);
				}

				pos = Eigen::MatrixXd(3, pts.size()/3);

				for(size_t i = 0; i < pts.size(); i += 3)
				{
					pos(0, i/3) = (pts[i]);
					pos(1, i/3) = (pts[i+1]);
					pos(2, i/3) = (pts[i+2]);
				}
				// plotPos(pos, pos.size()/3);

				// initRobotEfMarker << 0.61140, -0.32642, 0.46167 // simData
				initRobotEfMarker << -0.3681, 0.60182, 0.5287; // simData2
			}
		}// start




		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/*set com pose*/
			target = initialCom + move;
			comTask->com(target);


			/*Force sensor*/
			auto leftTh = thresh.head(6);
			auto leftForce = ctl.wrenches.at("LeftHandForceSensor").force();


			/*hand positions*/
			ltHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")];
			// cout << "ltHand "<< ltHand.translation().transpose() << endl;


			/*Get non-stop MOCAP Frame*/
			if(Flag_CORTEX)
			{
				getCurFrame =  Cortex_GetCurrentFrame();
				Cortex_CopyFrame(getCurFrame, &FrameofData);

				int ithFrame = FrameofData.iFrame;
				del+=FrameofData.fDelay;

				if(ithFrame == 0 || ithFrame == 1)
				{ 
					int firstFrame = ithFrame;
					startCapture = true;
					cout << "firstFrame " << firstFrame << endl;
				}
				else if(ithFrame <0)
				{ 
					int nthFrame = ithFrame;
					cout << "nthFrame "<< nthFrame << endl;
					Cortex_Request("Pause", &pResponse, &nBytes);
					Cortex_Exit();
					output("OK");
					return true;
				}
			}


			/* start only when ithFrame == 1 */
			if(startCapture)
			{
				/*get object marker pos*/
				if(Flag_CORTEX)
				{					
					cout << "check marker pos " << FrameofData.BodyData[body].Markers[wristMarkerR]<<endl;

					for(int m=0; m<pBody->nMarkers; m++)
					{
						Markers[m] <<
						FrameofData.BodyData[body].Markers[m][0], // X
						FrameofData.BodyData[body].Markers[m][1], // Y
						FrameofData.BodyData[body].Markers[m][2]; // Z
					}

					cout << "check marker pos again " << Markers[wristMarkerR].transpose()<<endl;

				}
				else /*for simulation*/
				{
					// objectBodyMarker << pos.col(i)-initRobotEfMarker;
					// robotWristMarker << initRobotEfMarker;
					// robotWristMarker << ltHand.translation();// +initRobotEfMarker; //leftEfmarkerPos same as leftEf

					// cout << "pos " << pos.col(i).transpose()<<endl;
					// cout << "robotWristMarker\n" << robotWristMarker.transpose() << endl;

					if(i==pos.size()/3)
					{
						LOG_WARNING("iter over")
						// output("Repeat");
						// return true;
						// startCapture = false;
						// i =0;
					}
				}



				/* check for non zero frame only and store them */ 

				// if( 	Markers[wristMarkerR](0)!= 0 && Markers[wristMarkerR](0)< 100
				// 	&&  Markers[markerObj](0)!= 0 && Markers[markerObj](0)< 100
				// 	&&  Markers[wristMarkerS](0)!= 0 && Markers[wristMarkerS](0)< 100
				// 	)
				// {

				// 	markersPos[wristMarkerR].col(i) << Markers[wristMarkerR];
				// 	markersPos[fingerMarkerR].col(i) << Markers[fingerMarkerR];

				// 	markersPos[markerObj].col(i)<< Markers[markerObj];

				// 	markersPos[wristMarkerS].col(i)<<Markers[wristMarkerS];
				// 	markersPos[knuckleMarkerS].col(i)<<Markers[knuckleMarkerS];

					
				// 	if( (i%t_observe == 0) && (prediction) )
				// 	{
				// 		/*get robot ef marker current pose*/
				// 		curPosLeftEfMarker << markersPos[fingerMarkerR].col((i-t_observe)+1);//1.162, -0.268, 1.074;
				// 		sva::PTransformd M_X_efLMarker(curRotLeftEfMarker, curPosLeftEfMarker);

				// 		/*get robot ef current pose*/
				// 		curRotLeftEf = ltHand.rotation();
				// 		curPosLeftEf = ltHand.translation();
				// 		sva::PTransformd R_X_efL(curPosLeftEf);

				// 		/*object marker pose w.r.t to robot EF frame*/
				// 		for(int j=1;j<=t_observe; j++)
				// 		{
				// 			sva::PTransformd M_X_ObjMarkerA(rotObjMarkerA, markersPos[markerObj].middleCols((i-t_observe)+j,i));
				// 			ObjMarkerA_X_efL = R_X_efL.inv()*M_X_ObjMarkerA*M_X_efLMarker.inv()*R_X_efL;

				// 			newPosObjMarkerA(0,j-1) = ObjMarkerA_X_efL.translation()(0);
				// 			newPosObjMarkerA(1,j-1) = ObjMarkerA_X_efL.translation()(1);
				// 			newPosObjMarkerA(2,j-1) = ObjMarkerA_X_efL.translation()(2);
							
				// 			/*get obj marker initials*/
				// 			if(j==1)
				// 			{
				// 				initPosObjMarkerA = newPosObjMarkerA.col(j-1);
				// 			}
				// 			if(j==t_observe)
				// 			{
				// 				ithPosObjMarkerA  = newPosObjMarkerA.col(t_observe-1);
				// 			}
				// 		}


				// 		/*get average velocity of previous *t_observe* sec obj motion*/
				// 		curVelObjMarkerA  = ctl.handoverTraj->diff(newPosObjMarkerA)*fps;//ignore diff > XXXX

				// 		avgVelObjMarkerA  << ctl.handoverTraj->takeAverage(curVelObjMarkerA);

				// 		/*get way points between obj inital motion*/ // get constant
				// 		auto actualPosObjMarkerA = ctl.handoverTraj->constVelocity(initPosObjMarkerA, ithPosObjMarkerA, t_observe);

				// 		/*predict position in straight line after t_predict time*/
				// 		predictPos = ctl.handoverTraj->constVelocityPredictPos(avgVelObjMarkerA, get<2>(actualPosObjMarkerA), t_predict);

				// 		/*get predicted way points between left ef and obj*/
				// 		wp_efL_objMarkerA=ctl.handoverTraj->constVelocity(ithPosObjMarkerA, predictPos, t_predict);
				// 		wp = get<0>(wp_efL_objMarkerA);
				// 		collected = true;
				// 	} //t_observe
					

				// 	/*gripper control*/
				// 	auto close_gripperL = [&]()
				// 	{
				// 		// // ctl.publishWrench();
				// 		// auto gripper = ctl.grippers["l_gripper"].get();
				// 		// gripper->setTargetQ({0.0});
				// 	};

				// 	auto open_gripperL = [&]()
				// 	{
				// 		// // ctl.publishWrench();
				// 		// auto gripper = ctl.grippers["l_gripper"].get();
				// 		// gripper->setTargetQ({0.5});
				// 	};


				// 	/*force control*/
				// 	auto compareForce = [&](const char * axis_name, int idx)
				// 	{
				// 		/*dont use fabs & check force direction to open and restart prediction*/
				// 		if( (fabs(leftForce[idx]) > leftTh[idx+3]) && gripperOpenTrue )
				// 		{
				// 			open_gripperL();
				// 			LOG_INFO("Opening grippers, threshold on " << axis_name << " reached on left hand")
				// 			// gripperOpenTrue = false;
				// 			// cout << "force detected, opening gripper" <<endl;
				// 			// prediction = false;
				// 			return false; //true
				// 		}
				// 	};


				// 	/*grasp object (close gripper)*/
				// 	auto compareRelPos = [&]()
				// 	{

				// 		// cout << "object & S_knuckle " << (objectBodyMarker-subjKnuckleMarker).norm() << endl;
				// 		// cout << "object & R_wrist " << (objectBodyMarker-robotWristMarker).norm() << endl;

				// 		/* ************************ CAL AREA HERE ************************ */
						
				// 		if( (Markers[markerObj]-Markers[knuckleMarkerS]).norm() > (Markers[markerObj]- Markers[wristMarkerR]).norm() )
				// 		{
				// 			if(gripperCloseTrue)
				// 			{
				// 				gripperOpenTrue = true;
				// 				gripperCloseTrue = false;
				// 				close_gripperL();
				// 				// cout << "object is inside gripper, closing gripper" <<endl;
				// 			}
							
				// 			return compareForce("x-axis", 0) || compareForce("y-axis", 1) || compareForce("z-axis", 2);
				// 			// prediction =false;
				// 		}
				// 		// else //start prediction again?
				// 		// {
				// 		// 	open_gripperL();
				// 		// 	cout << "compare rel pos --opening gripper" <<endl;
				// 		// 	prediction = true;
				// 		// 	ctl.runOnce = true;
				// 		// 	return false;
				// 		// }
				// 	};


				// 	if( collected )//&& prediction)
				// 	{
				// 		initPos = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")].translation();

				// 		if(taskAdded)
				// 		{
				// 			taskAdded = false;
				// 			// ctl.solver().addTask(ctl.posTaskL); //ENABLE LATER
				// 			// ctl.solver().addTask(ctl.posTaskR);
				// 		}

				// 		/* ***************** ATTENTION HERE ***************** */
				// 		initRefPos << wp(0,0), wp(1,0), wp(2,0);


				// 		for(int it=0; it<wp.cols(); it++)
				// 		{
				// 			/* ***************** ATTENTION HERE ***************** */
				// 			refPos << wp(0,it), wp(1,it), wp(2,it);
				// 			// cout << "wp " << wp.col(it).transpose()<<endl;

				// 			gothere = refPos + initPos -initRefPos;
				// 			// cout << "gothere " << gothere.transpose()<<endl;

				// 			refVel << Eigen::MatrixXd::Zero(3,1); //avgVelObjMarkerA;
				// 			refAcc << Eigen::MatrixXd::Zero(3,1);

				// 			auto curLEfPos = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")].translation();

				// 			/*robot constraint*/
				// 			if(	(gothere(0))<= 0.7 && (gothere(1))<= 0.6 && (gothere(2))<=1.5 &&
				// 				(gothere(0))>= 0.2 && (gothere(1))>= 0.25 && (gothere(2))>=0.9 ) 
				// 			{
				// 				/*control gripper*/
				// 				// cout << "(gothere - curLEfPos).norm() " << (gothere - curLEfPos).norm() << endl;
				// 				if( (gothere - curLEfPos).norm() <0.02 )
				// 				{
				// 					gripperOpenTrue = false;
				// 					open_gripperL();
									
				// 					compareRelPos();
				// 				}
				// 				// else
				// 				// {
				// 				// 	if(gripperCloseTrue && gripperOpenTrue)
				// 				// 	{
				// 				// 		gripperCloseTrue = false;
				// 				// 		close_gripperL();
				// 				// 	}
				// 				// }


				// 				/*control head*/
				// 				if(gothere(1) >.45){ctl.set_joint_pos("HEAD_JOINT0",  0.8);} //y //+ve to move head left
				// 				else{ctl.set_joint_pos("HEAD_JOINT0",  0.); }//+ve to move head left

				// 				if(gothere(2) < 1.1){ctl.set_joint_pos("HEAD_JOINT1",  0.4);} //z //+ve to move head down
				// 				else{ctl.set_joint_pos("HEAD_JOINT1",  -0.4);} //+ve to move head down


				// 				/*move end effector*/ //EANBLE LATER

				// 				// ctl.posTaskL->position(gothere);
				// 				// ctl.posTaskL->refVel(refVel);
				// 				// ctl.posTaskL->refAccel(refAcc);
				// 				// // cout << "posTaskL pos " << ctl.posTaskL->position().transpose()<<endl;


				// 			}
				// 		}
				// 		collected = false;
				// 	} // collected
				// 	i = i + 1;
				// }// check for non zero frame

			} // startCapture

			// output("OK");
			return false;
		}// run


		void StartMocapStep::teardown(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.solver().removeTask(comTask);
			ctl.solver().removeTask(ctl.posTaskL);
			// ctl.solver().removeTask(ctl.posTaskR);

			ctl.gui()->removeElement({"Handover","com"}, "Move Com Pos");
			ctl.gui()->removeElement({"Handover","wrench"},"publish_current_wrench");
			ctl.gui()->removeElement({"Handover","wrench"}, "Threshold");
			ctl.gui()->removeElement({"Handover", "object marker log"}, "log_data");
			// ctl.gui()->removeElement({"Handover", "move object"}, "Position");
		}

	} // namespace states

} // namespace mc_handover