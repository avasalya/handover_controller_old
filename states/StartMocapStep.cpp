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
			gripperL->setTargetQ({closeGrippers});
			gripperR->setTargetQ({closeGrippers});


			/*initial ef pos*/
			p1l << 0.3,0.3,1.1;
			p1r <<0.3,-0.3,1.1;

			q1l = {0.64, -0.01, -0.76, -0.06};
			q1r = {0.64, -0.01, -0.76, -0.06};


			constRotL<<
			-0.1624, -0.0616,   0.974,
			0.092,  0.9926,  0.0784,
			-0.9716,   0.104, -0.1554;

			constRotR << 
			0.013618,  0.0805374,   0.996659,
			-0.243029,   0.967128, -0.0748305,
			-0.969923,  -0.241198,  0.0327433;


			// X_R_efL_const = sva::PTransformd(q1l.toRotationMatrix(), p1l);
			X_R_efL_const = sva::PTransformd(constRotL, p1l);

			X_R_efR_const = sva::PTransformd(constRotR, p1r);


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


			/*EfL ori Task*/
			oriTaskL = make_shared<mc_tasks::OrientationTask>("LARM_LINK6",ctl.robots(), 0, 4.0, 500);
			ctl.solver().addTask(oriTaskL);
			initRotL = oriTaskL->orientation(); 

			oriTaskR = make_shared<mc_tasks::OrientationTask>("RARM_LINK6",ctl.robots(), 0, 4.0, 500);
			ctl.solver().addTask(oriTaskR);
			initRotR = oriTaskR->orientation();



			/*play-pause MOCAP*/
			ctl.gui()->addElement({"MOCAP"}, 
				mc_rtc::gui::Button("pause", [this]()
				{
					Cortex_Request("Pause", &pResponse, &nBytes);
				}),

				mc_rtc::gui::Button("Play", [this]()
				{
					Cortex_Request("LiveMode", &pResponse, &nBytes);
				})
				);

			/*restart mocapStep*/
			ctl.gui()->addElement({"Handover", "Restart"}, 
				mc_rtc::gui::Button("restartHandover", [this]()
					{restartEverything = true; cout << "restarting Everything"<<endl; }));

			/*Motion FOR CREATING MOCAP TEMPLATE*/
			ctl.gui()->addElement({"Handover", "reset"},
				
				mc_rtc::gui::Button("RIGHT ARM init / set flags", [this, &ctl]()
				{
					posTaskR->position(initPosR);
					oriTaskR->orientation(initRotR);

					approachObj->openGripper = false;
					approachObj->closeGripper = false;
					approachObj->gClose = false;
					approachObj->enableRHand=true;

					LOG_ERROR("manual reset")
				}),
				
				mc_rtc::gui::Button("RIGHT ARM init", [this, &ctl]()
				{
					posTaskR->position(initPosR);
					oriTaskR->orientation(initRotR);
				}),
				
				mc_rtc::gui::Button("LEFT ARM init / set flags", [this, &ctl]()
				{
					posTaskL->position(initPosL);
					oriTaskL->orientation(initRotL);

					approachObj->openGripper = false;
					approachObj->closeGripper = false;
					approachObj->gClose = false;
					approachObj->enableLHand=true;

					LOG_ERROR("manual reset")
				}),
				
				mc_rtc::gui::Button("LEFT ARM init", [this, &ctl]()
				{
					posTaskL->position(initPosL);
					oriTaskL->orientation(initRotL);
				})
				
				);

			/*publish wrench*/
			ctl.gui()->addElement({"Handover", "wrench"},

				mc_rtc::gui::Button("publish_current_wrench", [&ctl]() {
					cout << "left hand Forces " <<
					ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
					cout << "right hand Forces " <<
					ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
				}),
				
				mc_rtc::gui::Button("Norm_LeftEf_Force",[this, &ctl](){
					Eigen::Vector3d v = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					Eigen::Vector3d v2 = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					cout<<"Norm Fl"<< v.norm() <<endl;
					cout<<"Norm Fr"<< v2.norm() <<endl; }),

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
				// mc_rtc::gui::Transform("Position", 
				// 	[this,&ctl](){ return ctl.robots().robot(2).bodyPosW("base_link"); },
				// 	[this,&ctl](const sva::PTransformd & pos) { 
				// 		ctl.robots().robot(2).posW(pos);
				// 		// ctl.removeContact({"handoverobjects", "ground", "handoverPipeBottom", "AllGround"});
				// 		// ctl.addContact({"handoverobjects", "ground", "handoverPipeBottom", "AllGround"});
				// 	}),
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

			
			/*specific logs*/
			ctl.logger().addLogEntry("objectPos",[this]()-> Eigen::Vector3d { return approachObj->objectPos; });
			ctl.logger().addLogEntry("subjFinL",[this]() -> Eigen::Vector3d { return approachObj->fingerPosL; });
			ctl.logger().addLogEntry("subjFinR",[this]() -> Eigen::Vector3d { return approachObj->fingerPosR; });

			ctl.logger().addLogEntry("posTaskL", [this]() -> Eigen::Vector3d { return posTaskL->position(); });
			ctl.logger().addLogEntry("posTaskR", [this]() -> Eigen::Vector3d { return posTaskR->position(); });

		}// start


		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

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

				approachObj->openGripper = true;
				approachObj->gOpen = false;
			};


			auto close_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({closeGrippers});

				approachObj->gClose = false;
			};


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



			// if(R%300==0)
			// {	LOG_ERROR(R)
			// 	open_gripper("l_gripper");
			// 	close_gripper("r_gripper");
			// }


			// if(R%700==0)
			// {	LOG_INFO(R)
			// 	open_gripper("r_gripper");
			// 	close_gripper("l_gripper");
			// }

			// R=R+1;


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
						// LOG_INFO("body name: "<<pBody->szName<<"\n"<<
						// 	" pBody->nMarkers: " << pBody->nMarkers<<"\n"<<
						// 	" & FrameofData.BodyData[b].nMarkers: " <<FrameofData.BodyData[b].nMarkers<<"\n" )

						for(int m=0; m<pBody->nMarkers; m++)
						{
							if(b==0 && m==4)
							{}
						else
						{
							approachObj->Markers[c] <<
								FrameofData.BodyData[b].Markers[m][0], // X
								FrameofData.BodyData[b].Markers[m][1], // Y
								FrameofData.BodyData[b].Markers[m][2]; // Z
								c+=1;
								// cout<<approachObj->Markers[c].transpose()<<"\n";
							}
						}
					}
					else
						{ LOG_ERROR("approachObj->strMarkersBodyName[b] "<<approachObj->strMarkersBodyName[b]<<"\n"<<
							"pBody->szName "<<pBody->szName<<"\n"<< 
							"pBody->nMarkers: " << pBody->nMarkers<<"\n"<< 
							" & FrameofData.BodyData[b].nMarkers: " <<FrameofData.BodyData[b].nMarkers<<"\n" ) }
				}





				if( approachObj->handoverRun() )
				{
					/*Head Pose*/
					headTask->target(approachObj->objectPos);


					if( ((approachObj->i)%(approachObj->t_observe)==0) )
					{
						approachObj->useLeftEf = false;
						
						/*robot left / subj right*/
						approachObj->lHandPredict = approachObj->predictionController(ltHand, ltRotW, X_R_efL_const, approachObj->subjRtMarkers, approachObj->robotLtMarkers);


						/*robot left / subj left*/
						// approachObj->lHandPredict = approachObj->predictionController(ltHand, ltRotW, X_R_efL_const, approachObj->subjLtMarkers, approachObj->robotLtMarkers);

						
						approachObj->useLeftEf = get<0>(approachObj->lHandPredict);
					}



					if( approachObj->useLeftEf )
					{
						/*robot left / subj right*/
						taskOK = approachObj->goToHandoverPose(0.0, 0.7, approachObj->enableLHand, ltHand, posTaskL, oriTaskL, approachObj->lHandPredict, approachObj->fingerPosR);

						taskOK = approachObj->forceController(approachObj->enableLHand, initPosL, initRotL, leftForce, leftTh, posTaskL, oriTaskL, "l_gripper", approachObj->robotLtMarkers, approachObj->subjRtMarkers);
						

						/*robot left / subj left*/
						// taskOK = approachObj->goToHandoverPose(0.0, 0.7, approachObj->enableLHand, ltHand, posTaskL, oriTaskL, approachObj->lHandPredict, approachObj->fingerPosL);

						// taskOK = approachObj->forceController(approachObj->enableLHand, initPosL, initRotL, leftForce, leftTh, posTaskL, oriTaskL, "l_gripper", approachObj->robotLtMarkers, approachObj->subjLtMarkers);


						gripperControl("l_gripper");
					}

				}// handoverRun


			}//startCapture



			// posTaskL->position(p1l);
			// // oriTaskL->orientation(X_R_efL_const.rotation().transpose());
			// oriTaskL->orientation(X_R_efL_const.rotation());

			// LOG_ERROR(oriTaskL->orientation())


			// posTaskR->position(p1r);
			// oriTaskR->orientation(X_R_efR_const.rotation());

			// LOG_INFO(oriTaskR->orientation())








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

			ctl.solver().removeTask(oriTaskL);
			ctl.solver().removeTask(oriTaskR);

			// ctl.solver().removeTask(relEfTaskL);
			// ctl.solver().removeTask(relEfTaskR);

			// ctl.solver().removeTask(vecOriTaskL);
			// ctl.solver().removeTask(vecOriTaskR);

			ctl.solver().removeTask(comTask);
		}

	} // namespace states
} // namespace mc_handover




/*try below methods for rotation*/
//http://www.continuummechanics.org/rotationmatrix.html
//https://reference.wolfram.com/language/ref/RotationMatrix.html
//https://www.youtube.com/watch?v=lVjFhNv2N8o 7.7min
//http://www.songho.ca/opengl/gl_anglestoaxes.html

/*try below methods*/
//https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
//http://www.euclideanspace.com/maths/geometry/affine/conversions/quaternionToMatrix/index.htm
//http://www.euclideanspace.com/maths/geometry/affine/aroundPoint/
//https://stackoverflow.com/questions/10629737/convert-3d-4x4-rotation-matrix-into-2d
