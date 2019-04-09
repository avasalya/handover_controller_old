#include "StartMocapStep.h"

namespace mc_handover
{

	namespace states
	{
		void MyErrorMsgHandler(int iLevel, const char *szMsg)
		{
			const char *szLevel = NULL;

			if (iLevel == VL_Debug) { szLevel = "Debug"; }
			else if (iLevel == VL_Info) { szLevel = "Info"; }
			else if (iLevel == VL_Warning) { szLevel = "Warning"; }
			else if (iLevel == VL_Error) { szLevel = "Error"; }
			printf("  %s: %s\n", szLevel, szMsg);
		}


		void StartMocapStep::start(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/*allocate memory*/
			approachObj = std::make_shared<mc_handover::ApproachObject>();
			maxMarkers = approachObj->totalMarkers;
			approachObj->initials();


			/*close grippers for safety*/
			auto  gripperL = ctl.grippers["l_gripper"].get();
			auto  gripperR = ctl.grippers["r_gripper"].get();
			gripperL->setTargetQ({0.13});
			gripperR->setTargetQ({0.13});


			/*initial ef pos*/
			p1l << 0.3,0.3,1.1;
			p1r <<0.3,-0.3,1.1;


			/*initial force/torque threshold*/
			thresh << 10, 10, 10, 6, 6, 6, 10, 10, 10, 6, 6, 6;
			leftTh = thresh.segment(3,3);
			rightTh = thresh.segment(9,3);


			/*HeadTask*/
			headVector<<1., 0., 0.;
			headTarget<<1., 0., 0.;
			std::vector<std::string> activeJointsName = {"HEAD_JOINT0", "HEAD_JOINT1"};
			headTask.reset(new mc_tasks::LookAtTask("HEAD_LINK1", headVector, headTarget, ctl.robots(), ctl.robots().robotIndex(), 2., 500.));
			ctl.solver().addTask(headTask);
			headTask->selectActiveJoints(ctl.solver(), activeJointsName);



			/*chest pos task*/
			chestPosTask.reset(new mc_tasks::PositionTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(chestPosTask);
			chestPosTask->position({0.032, 0.0, 1.12});

			/*chest ori task*/
			chestOriTask.reset(new mc_tasks::OrientationTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(chestOriTask);
			chestOriTask->orientation(Eigen::Matrix3d::Identity());



			/*Ef pos Tasks*/
			posTaskL = make_shared<mc_tasks::PositionTask>("LARM_LINK7", ctl.robots(), 0, 4.0, 1e3);
			ctl.solver().addTask(posTaskL);
			initPosL = posTaskL->position();

			posTaskR = make_shared<mc_tasks::PositionTask>("RARM_LINK7", ctl.robots(), 0, 4.0, 1e3);
			ctl.solver().addTask(posTaskR);
			initPosR = posTaskR->position();



			/*relEF pos Tasks*/
			// relEfTaskL = make_shared<mc_tasks::RelativeEndEffectorTask>("LARM_LINK7", ctl.robots(), ctl.robots().robotIndex(), "", 2.0,1e3);
			// ctl.solver().addTask(relEfTaskL);

			// relEfTaskR = make_shared<mc_tasks::RelativeEndEffectorTask>("RARM_LINK7", ctl.robots(), ctl.robots().robotIndex(), "LARM_LINK7", 2.0,1e3);
			// ctl.solver().addTask(relEfTaskR);



			/*Ef VectorOrientationTasks*/
			initBodyVector<<0., 1., 0.; //body vector to be controlled
			initTargetVector<<0., 1., 0.; //target vector in world frame

			vecOriTaskL.reset(new mc_tasks::VectorOrientationTask("LARM_LINK6", initBodyVector, initTargetVector, ctl.robots(), ctl.robots().robotIndex(), 3.0, 500));
			ctl.solver().addTask(vecOriTaskL);

			vecOriTaskR.reset(new mc_tasks::VectorOrientationTask("RARM_LINK6", initBodyVector, initTargetVector, ctl.robots(), ctl.robots().robotIndex(), 3.0, 500));
			ctl.solver().addTask(vecOriTaskR);

			bodyVector<<1., 0., 0.;
			targetVector<<0., 0., 1.;



			/*restart mocapStep*/
			ctl.gui()->addElement({"Handover", "Restart"}, 
				mc_rtc::gui::Button("restartHandover", [this]()
					{restartEverything = true; cout << "restarting Everything"<<endl; }));

			/*Motion FOR CREATING MOCAP TEMPLATE*/
			ctl.gui()->addElement({"Handover", "randomPos"},
				mc_rtc::gui::Button("open_gripper & set flags", [this, &ctl]()
				{
					auto gripper = ctl.grippers["l_gripper"].get();
					gripper->setTargetQ({0.5});//open Gripper
					approachObj->enableLHand=true;
					approachObj->enableRHand=true;
					approachObj->gClose = false;
					approachObj->closeGripper = false;
				}),
				mc_rtc::gui::Button("init*", [this, &ctl]()
				{
					posTaskL->position(initPosL);
					vecOriTaskL->bodyVector(initBodyVector);
					vecOriTaskL->targetVector(initTargetVector);

					posTaskR->position(initPosR);
					vecOriTaskR->bodyVector(initBodyVector);
					vecOriTaskR->targetVector(initTargetVector);
				}),
				mc_rtc::gui::Button( "pos0", [this, &ctl](){
					posTaskL->position(p1l);
					vecOriTaskL->bodyVector(bodyVector);
					vecOriTaskL->targetVector(targetVector);

					posTaskR->position(p1r);
					vecOriTaskR->bodyVector(bodyVector);
					vecOriTaskR->targetVector(targetVector);
				} ),
				mc_rtc::gui::Button( "pos1", [this, &ctl](){
					posTaskL->position({0.24,0.3,0.8});
					vecOriTaskL->bodyVector(bodyVector);
					vecOriTaskL->targetVector(targetVector);

					posTaskR->position({0.24,-0.3,0.8});
					vecOriTaskR->bodyVector(bodyVector);
					vecOriTaskR->targetVector(targetVector);
				} ),
				mc_rtc::gui::Button( "pos2", [this, &ctl](){
					posTaskL->position({0.4,0.45,1.3});
					vecOriTaskL->bodyVector(bodyVector);
					vecOriTaskL->targetVector(targetVector);

					posTaskR->position({0.4,-0.45,1.3});
					vecOriTaskR->bodyVector(bodyVector);
					vecOriTaskR->targetVector(targetVector);
				} ),
				mc_rtc::gui::Button( "pos3", [this, &ctl](){
					posTaskL->position({0.3,0.25,1.1});
					vecOriTaskL->bodyVector(bodyVector);
					vecOriTaskL->targetVector(targetVector);

					posTaskR->position({0.3,-0.25,1.1});
					vecOriTaskR->bodyVector(bodyVector);
					vecOriTaskR->targetVector(targetVector);
				} ),
				mc_rtc::gui::Button( "pos4", [this, &ctl](){
					posTaskL->position({0.5,0.3,0.8});
					vecOriTaskL->bodyVector(bodyVector);
					vecOriTaskL->targetVector(targetVector);

					posTaskR->position({0.5,-0.3,0.8});
					vecOriTaskR->bodyVector(bodyVector);
					vecOriTaskR->targetVector(targetVector);
				} ),
				mc_rtc::gui::Button( "pos5", [this, &ctl](){
					posTaskL->position({0.1,0.4,1.24});
					vecOriTaskL->bodyVector(bodyVector);
					vecOriTaskL->targetVector(targetVector);

					posTaskR->position({0.1,-0.4,1.24});
					vecOriTaskR->bodyVector(bodyVector);
					vecOriTaskR->targetVector(targetVector);
				} ),
				mc_rtc::gui::Button( "pos6", [this, &ctl](){
					posTaskL->position({0.3,0.5,1.0});
					vecOriTaskL->bodyVector(bodyVector);
					vecOriTaskL->targetVector(targetVector);

					posTaskR->position({0.3,-0.5,1.0});
					vecOriTaskR->bodyVector(bodyVector);
					vecOriTaskR->targetVector(targetVector);
				} ),
				mc_rtc::gui::Button( "pos7", [this, &ctl](){
					posTaskL->position({0.1,0.4,1.24});
					vecOriTaskL->bodyVector(bodyVector);
					vecOriTaskL->targetVector(targetVector);

					posTaskR->position({0.5,-0.3,0.8});
					vecOriTaskR->bodyVector(bodyVector);
					vecOriTaskR->targetVector(targetVector);
				} ),
				mc_rtc::gui::Button( "pos8", [this, &ctl](){
					posTaskL->position({0.3,0.5,1.0});
					vecOriTaskL->bodyVector(bodyVector);
					vecOriTaskL->targetVector(targetVector);

					posTaskR->position({0.4,-0.45,1.3});
					vecOriTaskR->bodyVector(bodyVector);
					vecOriTaskR->targetVector(targetVector);
				} )
				);

			/*publish wrench*/
			ctl.gui()->addElement({"Handover", "wrench"},

				mc_rtc::gui::Button("publish_current_wrench", [&ctl]() {
					cout << "left hand Forces " <<
					ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
					// cout << "right hand Forces " <<
					// ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
				}),
				
				mc_rtc::gui::Button("Norm_LeftEf_Force",[this, &ctl](){
					Eigen::Vector3d v = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					cout<<"Norm "<< v.norm() <<endl; }),

				mc_rtc::gui::ArrayInput("set Threshold %",
					{"Left cx", "cy", "cz", "fx", "fy", "fz", "Right cx", "cy", "cz", "fx", "fy", "fz"},
					[this]() { return thresh; },
					[this](const Eigen::VectorXd & t)
					{
						LOG_INFO("Changed threshold to:\nLeft: " << t.head(6).transpose() << "\nRight: " << t.tail(6).transpose() << "\n")
						thresh = t;
					}));

			/*change prediction_ settings*/
			ctl.gui()->addElement({"Handover", "tuner"},
				mc_rtc::gui::ArrayInput("t_predict/t_observe", {"t_predict", "t_observe", "it"},
					[this]() { return approachObj->tuner; }, 
					[this](const Eigen::Vector3d & to){approachObj->tuner = to;cout<< "t_predict = " << approachObj->tuner(0)*1/fps<< "sec, t_observe = "<<approachObj->tuner(1)*1/fps<< "sec"<<endl;}));

			/*move object using cursor or simData*/
			ctl.gui()->addElement({"Handover","move_object"},
				mc_rtc::gui::Transform("Position", 
					[this,&ctl](){ return ctl.robots().robot(2).bodyPosW("base_link"); },
					[this,&ctl](const sva::PTransformd & pos) { 
						ctl.robots().robot(2).posW(pos);
						ctl.removeContact({"handoverobjects", "ground", "handoverPipeBottom", "AllGround"});
						ctl.addContact({"handoverobjects", "ground", "handoverPipeBottom", "AllGround"});
					}),
				// mc_rtc::gui::Button("Replay", [this](){ i = 0;}),
				mc_rtc::gui::Point3D("object position", [this,&ctl](){ ctl.robots().robot(2).posW({approachObj->objectPos}); return approachObj->objectPos;})
				);

			/*trajectory trail*/
			// ctl.gui()->addElement({"Handover", "Trajectories"},
				// mc_rtc::gui::Trajectory("traj_l_wrist", {{1,0,1}, 0.01, mc_rtc::gui::LineStyle::Dotted},
				// 	[this,&ctl](){ return ctl.robot().bodyPosW("LARM_LINK7").translation(); })
				// );

			/*com Task*/
			ctl.gui()->addElement({"Handover", "com"},

				mc_rtc::gui::ArrayInput("Move Com Pos", {"x", "y", "z"},
					[this]() { return move; },
					[this](const Eigen::Vector3d v) { move = v;
						cout << " com pos set to:\n" << initialCom + move << endl;})
				);

			comTask = make_shared<mc_tasks::CoMTask>(ctl.robots(), ctl.robots().robotIndex(), 10., 1e5);
			initialCom = rbd::computeCoM(ctl.robot().mb(),ctl.robot().mbc());
			comTask->com(initialCom);
			ctl.solver().addTask(comTask);



			/*configure MOCAP*/
			if(Flag_CORTEX)
			{
				cout << "\033[1;32m ***MOCAP IS ENABLED*** \033[0m\n";
				Cortex_SetVerbosityLevel(VL_Info);
				Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
	
				if(ctl.Flag_ROBOT)
					{ retval = Cortex_Initialize("10.1.1.180", "10.1.1.190"); }
				else{ retval = Cortex_Initialize("10.1.1.200", "10.1.1.190"); }
	
				if (retval != RC_Okay)
					{ printf("Error: Unable to initialize ethernet communication\n");
				retval = Cortex_Exit(); }
	
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
					{ printf("Failed to get body defs\n"); } 
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
							{ cout << iMarker << " " << pBody->szMarkerNames[iMarker] << endl; }
					}
				}
				printf("\n*** start live mode ***\n");
				Cortex_Request("LiveMode", &pResponse, &nBytes);
			}
			else /*simulation*/
			{
				cout << "\033[1;31m ***MOCAP IS DISABLED*** \033[0m\n";
				// startCapture = true; //true for sim
				
				// name = {"bothHands_1"};
				// string fn = string(DATA_PATH) + "/" + name + ".txt";
				// ifstream file(fn);

				// if(!file.is_open())
				// 	{ LOG_ERROR("Failed to open ") }

				// while(file >> pt)
				// 	{ pts.push_back(pt); }

				// pos.resize(10/*approachObj-totalMarkers*/);
				// for(unsigned int m=0; m<pos.size(); m++)
				// {	
				// 	pos[m] = Eigen::MatrixXd(3, int(pts.size()/30));

				// 	for(size_t i = 0; i < pts.size(); i += 30)
				// 	{
				// 		pos[m](0, i/30) = (pts[i]);
				// 		pos[m](1, i/30) = (pts[i+1]);
				// 		pos[m](2, i/30) = (pts[i+2]);

				// 		cout << pos[m].transpose()<<" " <<i <<endl;
				// 	}
				// 	LOG_SUCCESS(pts.size() )
				// }
				
				// posTaskL->position({0.3, 0.4, 1.1});
				// posTaskR->position({0.3, -0.4, 1.1});
			}



			posTaskL->position({0.3, 0.4, 1.1});
			posTaskR->position({0.3, -0.4, 1.1});

			vecOriTaskL->bodyVector({0., 1., 0.});
			vecOriTaskL->targetVector({0., 0., 1.});

			vecOriTaskR->bodyVector({0., -1., 0.});
			vecOriTaskR->targetVector({0., 0., 1.});


		}// start


		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			// BodyW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("BODY")];
			// cout <<BodyW.rotation()<<endl<<endl;
			// relEfTaskL->set_ef_pose( sva::PTransformd(sva::RotY(-(pi/180)*90)*sva::RotX(-(pi/180)*90), p1l) );


			initPosL = posTaskL->position();
			initPosL << initPosL(0), -initPosL(1), initPosL(2);
			posTaskR->position(initPosL);


			/*hand pose*/
			ltHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")];
			ltRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")].rotation();

			rtHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK7")];
			rtRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")].rotation();


			/*Force sensor*/
			leftForce = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
			rightForce = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();


			/*set com pose*/
			target = initialCom + move;
			comTask->com(target);


			auto open_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({openGrippers});
				approachObj->gOpen = false;
			};

			auto close_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({closeGrippers});
				approachObj->gClose = false;
			};



			/*Get non-stop MOCAP Frame*/
			if(Flag_CORTEX)
			{
				getCurFrame = Cortex_GetCurrentFrame();
				Cortex_CopyFrame(getCurFrame, &FrameofData);
	
				int ithFrame = FrameofData.iFrame;
				del+=FrameofData.fDelay;
	
				if(ithFrame == 0 || ithFrame == 1)
					{ startCapture = true; }
				else if(ithFrame <0)
				{ 
					Cortex_Request("Pause", &pResponse, &nBytes);
					Cortex_Exit();
					output("OK");
					return true;
				}
			}



			/*start only when ithFrame == 1*/
			if(startCapture)
			{
				/*get markers position FrameByFrame*/
				if(approachObj->Flag_withoutRobot)
				{ b_ = 2; c = 8; }
				else
				{ b_ = 0; c = 0; }

				for(int b=b_; b<totalBodies; b++)
				{
					/*make sure mocap template body marker's index are correct*/
					pBody = &pBodyDefs->BodyDefs[b];
					if(pBody->szName == approachObj->strMarkersBodyName[b])
					{
						/*LOG_INFO("body name: "<<pBody->szName<<"\n"<<
							" pBody->nMarkers: " << pBody->nMarkers<<"\n"<<
							" & FrameofData.BodyData[b].nMarkers: " <<FrameofData.BodyData[b].nMarkers<<"\n" )*/

						for(int m=0; m<pBody->nMarkers; m++)
						{
							approachObj->Markers[c] <<
							FrameofData.BodyData[b].Markers[m][0], // X
							FrameofData.BodyData[b].Markers[m][1], // Y
							FrameofData.BodyData[b].Markers[m][2]; // Z
							c+=1;
						}
					}
					else
					{ LOG_ERROR("approachObj->strMarkersBodyName[b] "<<approachObj->strMarkersBodyName[b]<<"\n"<<
						"pBody->szName "<<pBody->szName<<"\n"<< 
						"pBody->nMarkers: " << pBody->nMarkers<<"\n"<< 
						 " & FrameofData.BodyData[b].nMarkers: " <<FrameofData.BodyData[b].nMarkers<<"\n" ) }
				}


				if( approachObj->handoverRun() && (FrameofData.nUnidentifiedMarkers==0) )
				{
					/*Head Pose*/
					headTask->target(approachObj->objectPos);

					/*move EF when subject approaches object 1st time*/
					if( !startHandover && (approachObj->obj_rel_subjRtHand < 0.2) )
					{
						posTaskL->position(p1l);
						vecOriTaskL->bodyVector(bodyVector);
						vecOriTaskL->targetVector(targetVector);
						LOG_ERROR("subject right hand approaching object ")

						if(posTaskL->eval().norm() >0.5 || posTaskL->eval().norm() <0.1) 
						{
							startHandover=true;
						}
					}
					else if( !startHandover && (approachObj->obj_rel_subjLtHand < 0.2) )
					{
						posTaskR->position(p1r);
						vecOriTaskR->bodyVector(bodyVector);
						vecOriTaskR->targetVector(targetVector);
						LOG_SUCCESS("subject left hand approaching object ")
						if(posTaskR->eval().norm() >0.1 || posTaskR->eval().norm() <0.15) 
						{
							startHandover=true;
						}
					}



					/*observe subject motion for t_observe period*/
					if( startHandover && ((approachObj->i)%(approachObj->t_observe)==0) )
					{
						auto obj_rel_subj = [&]() -> std::vector<string>
						{
							if(approachObj->obj_rel_subjRtHand < approachObj->obj_rel_subjLtHand)
							{ subjMarkersName = approachObj->subjRtMarkers; }
							else
							{ subjMarkersName = approachObj->subjLtMarkers; }
							return subjMarkersName;
						};

						auto obj_rel_robot = [&]() -> bool
						{
							obj_rel_subj();
							
							if(approachObj->obj_rel_robotLtHand < approachObj->obj_rel_robotRtHand)
							{
								if(stopRtHand)
								{
									LOG_INFO("robotLeftHand in use\n")
									stopRtHand = false;
									posTaskR->position(initPosR);
									vecOriTaskR->bodyVector(initBodyVector);
									vecOriTaskR->targetVector(initTargetVector);

									stopLtHand = true;
									posTaskL->position(p1l);
									vecOriTaskL->bodyVector(bodyVector);
							 		vecOriTaskL->targetVector(targetVector);
								}

								robotMarkersName = approachObj->robotLtMarkers;
							
								approachObj->lHandPredict = approachObj->predictionController(ltHand, ltRotW, subjMarkersName, robotMarkersName);
								approachObj->useLeftEf = get<0>(approachObj->lHandPredict);
							}
							else
							{
								if(stopLtHand)
								{
									LOG_WARNING("robotRightHand in use\n")
									stopLtHand = false;
									posTaskL->position(initPosL);
									vecOriTaskL->bodyVector(initBodyVector);
									vecOriTaskL->targetVector(initTargetVector);
								
									stopRtHand = true;
									posTaskR->position(p1r);
									vecOriTaskR->bodyVector(bodyVector);
								 	vecOriTaskR->targetVector(targetVector);
								 }

								robotMarkersName = approachObj->robotRtMarkers;

								approachObj->rHandPredict = approachObj->predictionController(rtHand, rtRotW, subjMarkersName, robotMarkersName);
								approachObj->useRightEf = get<0>(approachObj->rHandPredict);
							}
							return false;
						};
						obj_rel_robot();
					}// i%t_observe;



					auto gripperControl = [&](std::string gripperName)
					{
						if(approachObj->gOpen)
						{
							open_gripper(gripperName);
						}

						if(approachObj->gClose)
						{
							close_gripper(gripperName);
						}
					};


					/*feed Ef pose*/
					if( approachObj->useLeftEf )
					{
						approachObj->useRightEf=false;

						taskOK = approachObj->goToHandoverPose(0.2, 0.7, approachObj->enableLHand, ltHand, posTaskL, vecOriTaskL, approachObj->lHandPredict);
						taskOK = approachObj->handoverForceController(approachObj->enableLHand, initPosL, leftForce, leftTh, posTaskL, vecOriTaskL, "l_gripper", robotMarkersName, subjMarkersName);
						gripperControl("l_gripper");
					}
					else if( approachObj->useRightEf )
					{
						approachObj->useLeftEf=false;

						taskOK = approachObj->goToHandoverPose(-0.7, 0.2, approachObj->enableRHand, rtHand, posTaskR, vecOriTaskR, approachObj->rHandPredict);
						taskOK = approachObj->handoverForceController(approachObj->enableRHand, initPosR, rightForce, rightTh, posTaskR, vecOriTaskR, "r_gripper", robotMarkersName, subjMarkersName);
						gripperControl("r_gripper");
					}

				}// handoverRun
			}//startCapture



			if(restartEverything)
			{
				restartEverything = false;
				if(Flag_CORTEX)
				{
					Cortex_Request("Pause", &pResponse, &nBytes);
					Cortex_Exit();
				}
				output("Repeat");
				return true;
			}
			else
			{ return false; }

		}// run


		void StartMocapStep::teardown(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.gui()->removeCategory({"Handover"});
			
			ctl.solver().removeTask(headTask);

			ctl.solver().removeTask(chestPosTask);
			ctl.solver().removeTask(chestOriTask);

			ctl.solver().removeTask(posTaskL);
			ctl.solver().removeTask(posTaskR);

			// ctl.solver().removeTask(relEfTaskL);
			// ctl.solver().removeTask(relEfTaskR);

			ctl.solver().removeTask(vecOriTaskL);
			ctl.solver().removeTask(vecOriTaskR);

			ctl.solver().removeTask(comTask);
		}

	} // namespace states
} // namespace mc_handover




/*try below methods for rotation*/
//http://www.continuummechanics.org/rotationmatrix.html
//https://www.youtube.com/watch?v=lVjFhNv2N8o 7.7min
//http://www.songho.ca/opengl/gl_anglestoaxes.html

/*try below methods*/
//https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
//http://www.euclideanspace.com/maths/geometry/affine/conversions/quaternionToMatrix/index.htm
//http://www.euclideanspace.com/maths/geometry/affine/aroundPoint/



/****** VectorOrientationTask *********/

// void VectorOrientationTask::update(const rbd::MultiBody &mb,
// 	const rbd::MultiBodyConfig &mbc,
// 	const std::vector<sva::MotionVecd> &normalAccB)
// {
// 	//Evaluation of eval
// 	Eigen::Matrix3d E_0_b = mbc.bodyPosW[bodyIndex_].rotation().transpose();
// 	actualVector_ = E_0_b*bodyVector_;
// 	eval_ = targetVector_ - actualVector_;

// 	//Evaluation of speed and jacMat
// 	Eigen::MatrixXd shortMat, fullJac(3, mb.nrDof());
// 	shortMat = jac_.bodyJacobian(mb, mbc);
// 	jac_.fullJacobian(mb, shortMat.block(0, 0, 3, jac_.dof()), fullJac);
// 	jacMat_ = -E_0_b*skewMatrix(bodyVector_)*fullJac;
// 	Eigen::Vector3d w_b_b = jac_.bodyVelocity(mb, mbc).angular();
// 	speed_ = E_0_b*(w_b_b.cross(bodyVector_));

// 	//Evaluation of normalAcc
// 	Eigen::Vector3d bodyNormalAcc = jac_.bodyNormalAcceleration(mb, mbc, normalAccB).angular();
// 	normalAcc_ = w_b_b.cross(w_b_b.cross(bodyVector_));
// 	normalAcc_ += bodyNormalAcc.cross(bodyVector_);
// 	normalAcc_ = E_0_b*normalAcc_;
// }