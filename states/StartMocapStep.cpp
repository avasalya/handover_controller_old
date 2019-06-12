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



		void StartMocapStep::ros_spinner()
		{ ros::spin(); }



		void StartMocapStep::cortexCallback(const cortex_ros_bridge_msgs::Markers & msg)
		{
			// LOG_WARNING("cortexCallback")
			c = 0;
			for(unsigned int d=0; d<msg.markers.size(); d++)
			{
				if(msg.markers.at(d).marker_name== "dummy")
				{}
				else if( c < approachObj->totalMarkers)/*used markers count*/
				{					
					approachObj->Markers[c] <<
					msg.markers.at(d).translation.x, msg.markers.at(d).translation.y, msg.markers.at(d).translation.z;
					// LOG_INFO(msg.markers.at(d).marker_name <<" "<<c <<"    "<< approachObj->Markers[c].transpose())
					c+=1;
				}
			}
		}



		void StartMocapStep::start(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);


			/*current time*/
			time_t now = time(0);
			char* dt = ctime(&now);
			cout << "The local date and time is: " << dt << endl;


			/*allocate memory*/
			approachObj = std::make_shared<mc_handover::ApproachObject>();
			approachObj->initials();


			/*close grippers for safety*/
			auto  gripperL = ctl.grippers["l_gripper"].get();
			auto  gripperR = ctl.grippers["r_gripper"].get();
			gripperL->setTargetQ({closeGrippers});
			gripperR->setTargetQ({closeGrippers});


			constRotL<<
			-0.0267968,  -0.999573,  0.0116803,
			-0.141427,  0.0153579,    0.98983,
			-0.989586,  0.0248723,  -0.141778;
			constRotR<<
			-0.181998,  0.982828, -0.0304213,
			-0.0267631, -0.0358778,  -0.998998,
			 -0.982935,  -0.181001,  0.0328332;

			constPosL << 0.3, 0.4, 1;
			constPosR << 0.3, -0.4, 1;


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



			/*Ef ori Task*/
			oriTaskL = make_shared<mc_tasks::OrientationTask>("LARM_LINK6",ctl.robots(), 0, 4.0, 500);
			ctl.solver().addTask(oriTaskL);
			initRotL = oriTaskL->orientation(); 

			oriTaskR = make_shared<mc_tasks::OrientationTask>("RARM_LINK6",ctl.robots(), 0, 4.0, 500);
			ctl.solver().addTask(oriTaskR);
			initRotR = oriTaskR->orientation();



			/*handover endEffectorTask*/
			objEfTask = make_shared<mc_tasks::EndEffectorTask>("base_link", ctl.robots(), 2, 2.0, 1e3);
			ctl.solver().addTask(objEfTask);


			/*add handoverpipe/ground contact*/
			// ctl.addContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround", Eigen::Vector6d::Ones()});


			/*restart mocapStep*/
			ctl.gui()->addElement({"Handover", "Restart"}, 
				mc_rtc::gui::Button("restartHandover", [this]()
					{restartEverything = true; }));

			/*add remove contact*/
			ctl.gui()->addElement({"Handover", "Contacts"},

				mc_rtc::gui::Button("Add ground contact", [this, &ctl]()
				{
					ctl.addContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround", Eigen::Vector6d::Ones()});
				}),

				mc_rtc::gui::Button("Remove ground contact", [this, &ctl]()
				{
					ctl.removeContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"});
				}),

				mc_rtc::gui::Button("Add object/gripper contacts", [this, &ctl]()
				{
					ctl.addContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});
					ctl.addContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});
				}),

				mc_rtc::gui::Button("Remove object/gripper contacts", [this, &ctl]()
				{
					ctl.removeContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});
					ctl.removeContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});
				})

				);

			/*reset robot pose*/
			ctl.gui()->addElement({"Handover", "Reset Pose"},
				
				mc_rtc::gui::Button("LEFT ARM orientation", [this, &ctl]()
				{
					oriTaskL->orientation(constRotL);
				}),

				mc_rtc::gui::Button("RIGHT ARM orientation", [this, &ctl]()
				{
					oriTaskR->orientation(constRotR);
				}),

				mc_rtc::gui::Button("LEFT ARM object-rest pose", [this, &ctl]()
				{
					posTaskL->position(constPosL);
					oriTaskL->orientation(constRotL);
				}),

				mc_rtc::gui::Button("RIGHT ARM object-rest pose", [this, &ctl]()
				{
					posTaskR->position(constPosR);
					oriTaskR->orientation(constRotR);
				}),

				mc_rtc::gui::Button("LEFT ARM half-sit pose", [this, &ctl]()
				{
					posTaskL->position(initPosL);
					oriTaskL->orientation(initRotL);
				}),

				mc_rtc::gui::Button("RIGHT ARM half-sit pose", [this, &ctl]()
				{
					posTaskR->position(initPosR);
					oriTaskR->orientation(initRotR);
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
				
				mc_rtc::gui::Button("Norm_Force",[this, &ctl](){
					Eigen::Vector3d v = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					Eigen::Vector3d v2 = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					cout<<"Norm Fl "<< v.norm() <<endl;
					cout<<"Norm Fr "<< v2.norm() <<endl; }),

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
				cout << "\033[1;32m***MOCAP IS ENABLED*** \033[0m\n";
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
			else
			{
				cout << "\033[1;32m***ROS_MOCAP_BRIDGE IS ENABLED*** \033[0m\n";
				m_nh_ = mc_rtc::ROSBridge::get_node_handle();
				if(!m_nh_)
				{
				LOG_ERROR_AND_THROW(std::runtime_error, "This controller does not work withtout ROS")
				}
				m_ros_spinner_ = std::thread{[this](){ this->ros_spinner(); }};
				l_shape_sub_ = m_nh_->subscribe("novis_markers", 1, & mc_handover::states::StartMocapStep::cortexCallback, this);
			}


			/*specific logs*/
			ctl.logger().addLogEntry("posTaskL", [this]() -> Eigen::Vector3d { return posTaskL->position(); });
			ctl.logger().addLogEntry("posTaskR", [this]() -> Eigen::Vector3d { return posTaskR->position(); });
			
			ctl.logger().addLogEntry("objectPosC",[this]()-> Eigen::Vector3d { return approachObj->objectPosC; });
			ctl.logger().addLogEntry("subjFinL",[this]() -> Eigen::Vector3d { return approachObj->fingerPosL; });
			ctl.logger().addLogEntry("subjFinR",[this]() -> Eigen::Vector3d { return approachObj->fingerPosR; });


			ctl.logger().addLogEntry("Fzero",[this]() -> Eigen::Vector3d { return approachObj->Fzero; });
			ctl.logger().addLogEntry("Fclose",[this]() -> Eigen::Vector3d { return approachObj->Fclose; });
			ctl.logger().addLogEntry("efL Ace",[this]() -> Eigen::Vector3d { return efLAce; });
			ctl.logger().addLogEntry("efR Ace",[this]() -> Eigen::Vector3d { return efRAce; });
			ctl.logger().addLogEntry("Finert",[this]() -> Eigen::Vector3d { return approachObj->Finert; });
			ctl.logger().addLogEntry("Fload",[this]() -> Eigen::Vector3d { return approachObj->Fload; });
			ctl.logger().addLogEntry("new thresh",[this]() -> Eigen::Vector3d { return approachObj->newTh; });
			ctl.logger().addLogEntry("Fpull",[this]() -> Eigen::Vector3d { return approachObj->Fpull; });

			ctl.logger().addLogEntry("obj mass",[this]() -> double { return approachObj->objMass; });
			ctl.logger().addLogEntry("bool enableLHand",[this]() -> double { return approachObj->enableLHand; });
			ctl.logger().addLogEntry("bool enableRHand",[this]() -> double { return approachObj->enableRHand; });


			/*offsets for robot grippers to grasp object*/
			offsetLIn  << 0.0, 0.15, 0.0;
			offsetRIn  << 0.0, -0.15, 0.0;

			offsetLOut << 0.0, -0.10, 0.0;
			offsetROut << 0.0, 0.10, 0.0;

		}// start


		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/*hand pose*/
			ltPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")].translation();
			ltRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")].rotation();

			rtPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK7")].translation();
			rtRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")].rotation();
			

			/*Force sensor*/
			leftForce = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
			rightForce = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();


			/*set com pose*/
			target = initialCom + move;
			comTask->com(target);


			efLPos.resize(3);
			efLVel.resize(2);

			efRPos.resize(3);
			efRVel.resize(2);

			/*get ef(s) acceleration*/
			if( approachObj->i > 3 )
			{
				efLPos[3-g] = ltPosW;
				efRPos[3-g] = rtPosW;
				g++;

				if(g>3)
				{
					g = 1;
					efLVel[0] = (efLPos[1]-efLPos[2])*fps;
					efLVel[1] = (efLPos[0]-efLPos[1])*fps;
					efLAce = (efLVel[1]-efLVel[0])*fps;


					efRVel[0] = (efRPos[1]-efRPos[2])*fps;
					efRVel[1] = (efRPos[0]-efRPos[1])*fps;
					efRAce = (efRVel[1]-efRVel[0])*fps;
				}
			}


			auto open_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({openGrippers});//0.5
			};


			auto close_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({closeGrippers});
			};


			auto gripperControl = [&](std::string gripperName)
			{
				if(approachObj->gOpen)
				{
					open_gripper("l_gripper");
					open_gripper("r_gripper");
					approachObj->openGripper = true;
					approachObj->gOpen = false;
				}

				if(approachObj->gClose)
				{
					close_gripper(gripperName);
					approachObj->gClose = false;
				}
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
			else
			{ startCapture = true; }


			/*start only when ithFrame == 1*/
			if(startCapture)
			{
				if(Flag_CORTEX)
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
									// cout<<*getCurFrame->BodyData[b].Markers[m]<<endl;
								}
							}
						}
						else
							{ LOG_ERROR("approachObj->strMarkersBodyName[b] "<<approachObj->strMarkersBodyName[b]<<"\n"<<
								"pBody->szName "<<pBody->szName<<"\n"<< 
								"pBody->nMarkers: " << pBody->nMarkers<<"\n"<< 
								" & FrameofData.BodyData[b].nMarkers: " <<FrameofData.BodyData[b].nMarkers<<"\n" ) }
					}
				}

				if( approachObj->handoverRun() )
				{
					auto subjLtHandOnObj = [&]() -> sva::PTransformd
					{
						if(caseA)
						{	// Ol < Rr < Oc
							P_M_offset = sva::PTransformd(offsetLIn);
						}
						else if(caseB)
						{	// Rr < Ol
							P_M_offset = sva::PTransformd(offsetLOut);
						}

						X_M_Pipe0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->objectPosL);
						X_M_offsetR = X_M_Pipe0 * P_M_offset;

						return X_M_offsetR;
					};


					auto subjRtHandOnObj = [&]() ->sva::PTransformd
					{
						if(caseC)
						{	// Oc < Rl < Or
							P_M_offset = sva::PTransformd(offsetRIn);
						}
						else if(caseD)
						{	// Rl > Or
							P_M_offset = sva::PTransformd(offsetROut);
						}

						X_M_Pipe0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->objectPosR);
						X_M_offsetL = X_M_Pipe0 * P_M_offset;

						return X_M_offsetL;
					};


					auto obj_rel_subjHands = [&]() -> std::vector<string>
					{
						if(approachObj->obj_rel_subjRtHand < approachObj->obj_rel_subjLtHand)
						{
							subjMarkersName = approachObj->subjRtMarkers;
							fingerPos = approachObj->fingerPosR;
						}
						else
						{
							subjMarkersName = approachObj->subjLtMarkers;
							fingerPos = approachObj->fingerPosL;
						}
						return subjMarkersName;
					};



					auto obj_rel_subj = [&]()
					{
						caseA = (approachObj->objectPosC - approachObj->fingerPosL).norm() > // Hl < Ol < Oc
								(approachObj->objectPosC - approachObj->objectPosL).norm();  // Ol < Rr < Oc

						caseB = (approachObj->objectPosC - approachObj->fingerPosL).norm() < // Ol < Hl < Oc
								(approachObj->objectPosC - approachObj->objectPosL).norm();  // Rr < Ol


						caseC = (approachObj->objectPosC - approachObj->fingerPosR).norm() > // Hr > Oc > Or
								(approachObj->objectPosC - approachObj->objectPosR).norm();  // Oc < Rl < Or

						caseD = (approachObj->objectPosC - approachObj->fingerPosR).norm() < // Oc < Hr < Or
								(approachObj->objectPosC - approachObj->objectPosR).norm();  // Rl > Or

						if(approachObj->subjHasObject)
						{
							headTask->target(approachObj->objectPosC);
							objEfTask->set_ef_pose(sva::PTransformd(approachObj->objRot.transpose(), approachObj->objectPosC));

							subjLtHandOnObj();
							subjRtHandOnObj();
						}
						else if(approachObj->robotHasObject)
						{
							obj_rel_subjHands();

							headTask->target(fingerPos);
						}

					};


					/*track only subj right hand when robot carries the object*/
					auto obj_rel_robot = [&]() -> bool
					{
						obj_rel_subj();

						if(approachObj->subjHasObject)
						{
							approachObj->rHandPredict = approachObj->predictionController(rtPosW, constRotR, approachObj->subjLtMarkers);
							approachObj->useRightEf = get<0>(approachObj->rHandPredict);

							approachObj->lHandPredict = approachObj->predictionController(ltPosW, constRotL, approachObj->subjRtMarkers);
							approachObj->useLeftEf = get<0>(approachObj->lHandPredict);
						}
						else if(approachObj->robotHasObject)
						{
							if( subjMarkersName[0] == "lShapeRtA" )
							{
								approachObj->lHandPredict = approachObj->predictionController(ltPosW, constRotL, subjMarkersName);
								approachObj->useLeftEf = get<0>(approachObj->lHandPredict);
								approachObj->useRightEf = false;
							}
							else
							{
								approachObj->rHandPredict = approachObj->predictionController(rtPosW, constRotR, subjMarkersName);
								approachObj->useRightEf = get<0>(approachObj->rHandPredict);
								approachObj->useLeftEf = false;
							}
						}
						return false;
					};


					/*observe subject motion for t_observe period*/
					if( (approachObj->i)%(approachObj->t_observe) == 0 )
					{
						/* start only if object is within robot constraint space*/
						if( (approachObj->objectPosC(0) > 1.1) && (approachObj->objectPosC(0) < 2.0) && 
							(approachObj->fingerPosL(0) > 1.1) && (approachObj->fingerPosL(0) < 2.0) &&
							(approachObj->fingerPosR(0) > 1.1) && (approachObj->fingerPosR(0) < 2.0) )
						{ approachObj->startNow = true; }

						if( approachObj->startNow )
							{ obj_rel_robot(); }
					}// i%t_observe;



					/*feed Ef pose*/
					if( (approachObj->useLeftEf) || (approachObj->useRightEf) )
					{
						if(approachObj->subjHasObject)
						{
							updateOffsetPosL = X_M_offsetL.translation();
							approachObj->goToHandoverPose(0.0, 0.75, approachObj->enableLHand, ltPosW, posTaskL, oriTaskL, approachObj->lHandPredict, updateOffsetPosL);


							updateOffsetPosR = X_M_offsetR.translation();
							approachObj->goToHandoverPose(-0.75, 0.0, approachObj->enableRHand, rtPosW, posTaskR, oriTaskR, approachObj->rHandPredict, updateOffsetPosR);
						}
						else if(approachObj->robotHasObject)
						{
							updateOffsetPos = fingerPos;

							if(approachObj->useLeftEf)
							{
								approachObj->goToHandoverPose(0.0, 0.75, approachObj->enableLHand, ltPosW, posTaskL, oriTaskL, approachObj->lHandPredict, updateOffsetPos);
							}
							else if(approachObj->useRightEf)
							{
								approachObj->goToHandoverPose(-0.75, 0.0, approachObj->enableRHand, rtPosW, posTaskR, oriTaskR, approachObj->rHandPredict, updateOffsetPos);
							}
						}
						

						/*check both gripper forces together*/
						taskOK = approachObj->forceController( initPosL, initRotL, leftForce, leftTh, efLAce, posTaskL, oriTaskL, "leftGripper", approachObj->enableLHand, approachObj->robotLtMarkers, approachObj->subjRtMarkers, approachObj->obj_rel_robotLtHand);

						gripperControl("l_gripper");

						taskOK = approachObj->forceController( initPosR, initRotR, rightForce, rightTh, efRAce, posTaskR, oriTaskR, "rightGripper", approachObj->enableRHand, approachObj->robotRtMarkers, approachObj->subjLtMarkers, approachObj->obj_rel_robotRtHand);

						gripperControl("r_gripper");
					}

					// add again-one time thing	ctl.solver().addTask(objEfTask);

				}// handoverRun

			}//startCapture



			if(restartEverything)
			{
				dt+=1;

				/*remove contacts*/
				// addContact_object_ground();
				ctl.removeContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});
				ctl.removeContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});


				auto  gripperL = ctl.grippers["l_gripper"].get();
				auto  gripperR = ctl.grippers["r_gripper"].get();

				gripperL->setTargetQ({openGrippers});
				gripperR->setTargetQ({openGrippers});

				posTaskL->stiffness(2.0);
				posTaskL->position(initPosL);

				posTaskR->stiffness(2.0);
				posTaskR->position(initPosR);

				oriTaskL->orientation(initRotL);
				oriTaskR->orientation(initRotR);

				caseA = false;
				caseB = false;
				caseC = false;
				caseD = false;

				approachObj->subjHasObject = true;
				approachObj->robotHasObject = false;

				removeTask = true;

				approachObj->e = 1;

				approachObj->startNow = false;

				approachObj->gClose = false;
				approachObj->gOpen = false;
				approachObj->openGripper = false;
				approachObj->closeGripper = false;

				approachObj->graspObject = true;
				approachObj->goBackInit = true;
				approachObj->takeBackObject = false;
				approachObj->restartHandover = false;

				approachObj->useLeftEf = true;
				approachObj->stopLtEf = true;

				approachObj->useRightEf = true;
				approachObj->stopRtEf = true;


				approachObj->newTh = Eigen::Vector3d::Zero();
				approachObj->Finert = Eigen::Vector3d::Zero();
				approachObj->Fzero = Eigen::Vector3d::Zero();
				approachObj->Fclose = Eigen::Vector3d::Zero();
				approachObj->Fload = Eigen::Vector3d::Zero();
				approachObj->Fpull = Eigen::Vector3d::Zero();

				objEfTask->set_ef_pose( sva::PTransformd(Eigen::Vector3d(0.0,0.0,0.0)) );

				if( (dt%800) == 0 )
				{
					dt = 1;
					posTaskL->stiffness(4.0);
					posTaskR->stiffness(4.0);

					approachObj->enableLHand = true;
					approachObj->enableRHand = true;

					gripperL->setTargetQ({closeGrippers});
					gripperR->setTargetQ({closeGrippers});

					restartEverything = false;

					cout<<"\033[1;33m***handover fresh start***\033[0m\n";
				}
			}
			return false;

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

			ctl.solver().removeTask(comTask);
		}

	} // namespace states
} // namespace mc_handover


// std::string SubjHandOnObj;

// obj_rel_subjHand();

// if( (SubjHandOnObj == "both") || (SubjHandOnObj == "right") )
// {
// 	headTask->target(approachObj->fingerPosR);
// }
// else if(SubjHandOnObj == "left")
// {
// 	headTask->target(approachObj->fingerPosL);
// }


// auto obj_rel_subjHand = [&]() -> std::string
// {
// 	if( (approachObj->obj_rel_subjRtHand < 0.25)  && (approachObj->obj_rel_subjLtHand < 0.25) )
// 	{
// 		SubjHandOnObj = "both";
// 	}
// 	else if(approachObj->obj_rel_subjRtHand < 0.25)
// 	{
// 		SubjHandOnObj = "right";
// 	}
// 	else if(approachObj->obj_rel_subjLtHand < 0.25)
// 	{
// 		SubjHandOnObj = "left";
// 	}
// 	return SubjHandOnObj;
// };
