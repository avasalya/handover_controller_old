#include "StartMocapStep.h"

namespace mc_handover
{

	namespace states
	{

		void StartMocapStep::ros_spinner()
		{ ros::spin(); }



		void MyErrorMsgHandler(int iLevel, const char *szMsg)
		{
			const char *szLevel = NULL;

			if (iLevel == VL_Debug) { szLevel = "Debug"; }
			else if (iLevel == VL_Info) { szLevel = "Info"; }
			else if (iLevel == VL_Warning) { szLevel = "Warning"; }
			else if (iLevel == VL_Error) { szLevel = "Error"; }
			printf("  %s: %s\n", szLevel, szMsg);
		}



		void StartMocapStep::cortexCallback(const cortex_ros_bridge_msgs::Markers & msg)
		{
			// LOG_WARNING("cortexCallback")
			c = 0;
			for(unsigned int d=0; d<msg.markers.size(); d++)
			{
				if(msg.markers.at(d).marker_name== "dummy")
				{}
				else if(msg.markers.at(d).marker_name== "dummyObj")
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


			relaxRotL<<
			-0.0267968,  -0.999573,  0.0116803,
			-0.141427,  0.0153579,    0.98983,
			-0.989586,  0.0248723,  -0.141778;
			relaxRotR<<
			-0.181998,  0.982828, -0.0304213,
			-0.0267631, -0.0358778,  -0.998998,
			 -0.982935,  -0.181001,  0.0328332;

			relaxPosL << 0.15, 0.35, 0.8;
			relaxPosR << 0.15, -0.35, 0.8;


			/*initial force/torque threshold*/
			thresh << 10, 10, 10, 6, 6, 6, 10, 10, 10, 6, 6, 6;

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

			/*body ori task*/
			bodyOriTask.reset(new mc_tasks::OrientationTask("BODY", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(bodyOriTask);
			bodyOriTask->orientation(Eigen::Matrix3d::Identity());


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
			objEfTask->set_ef_pose(Eigen::Vector3d(0.23, 0.0, 0.845));



			/*restart mocapStep*/
			ctl.gui()->addElement({"Handover", "Restart"},
				mc_rtc::gui::Button("restartHandover", [this]()
					{restartEverything = true; }));

			/*add remove contact*/
			ctl.gui()->addElement({"Handover", "Contacts"},

				mc_rtc::gui::Button("Add ground contact", [this, &ctl]()
				{
					ctl.addContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"});
				}),

				mc_rtc::gui::Button("Remove ground contact", [this, &ctl]()
				{
					ctl.removeContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"});
				}),

				mc_rtc::gui::Button("Add object/gripper contacts", [this, &ctl]()
				{
					approachObj->robotHasObject = true;
					approachObj->subjHasObject = false;

					ctl.solver().removeTask(objEfTask);
					ctl.solver().removeTask(posTaskR);
					ctl.solver().removeTask(oriTaskR);

					ctl.addContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});
					ctl.addContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});
				}),

				mc_rtc::gui::Button("Remove object/gripper contacts", [this, &ctl]()
				{
					approachObj->subjHasObject = true;
					approachObj->robotHasObject = false;

					ctl.solver().addTask(objEfTask);
					ctl.solver().addTask(posTaskR);
					ctl.solver().addTask(oriTaskR);

					ctl.removeContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});
					ctl.removeContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});
				})

				);

			/*reset robot pose*/
			ctl.gui()->addElement({"Handover", "Reset Pose"},

				mc_rtc::gui::Button("LEFT ARM orientation", [this, &ctl]()
				{
					oriTaskL->orientation(relaxRotL);
				}),

				mc_rtc::gui::Button("RIGHT ARM orientation", [this, &ctl]()
				{
					oriTaskR->orientation(relaxRotR);
				}),

				mc_rtc::gui::Button("LEFT ARM object-rest pose", [this, &ctl]()
				{
					posTaskL->position(relaxPosL);
					oriTaskL->orientation(relaxRotL);
				}),

				mc_rtc::gui::Button("RIGHT ARM object-rest pose", [this, &ctl]()
				{
					posTaskR->position(relaxPosR);
					oriTaskR->orientation(relaxRotR);
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

			ctl.logger().addLogEntry("obj mass",[this]() -> double { return approachObj->objMass; });
			ctl.logger().addLogEntry("bool enableHand",[this]() -> double { return approachObj->enableHand; });

			ctl.logger().addLogEntry("objectPosC",[this]()-> Eigen::Vector3d { return approachObj->objectPosC; });
			ctl.logger().addLogEntry("subjFinL",[this]() -> Eigen::Vector3d { return approachObj->fingerPosL; });
			ctl.logger().addLogEntry("subjFinR",[this]() -> Eigen::Vector3d { return approachObj->fingerPosR; });


			ctl.logger().addLogEntry("efL Ace",[this]() -> Eigen::Vector3d { return efLAce; });
			ctl.logger().addLogEntry("FzeroL",[this]() -> Eigen::Vector3d { return approachObj->FzeroL; });
			ctl.logger().addLogEntry("FcloseL",[this]() -> Eigen::Vector3d { return approachObj->FcloseL; });
			ctl.logger().addLogEntry("FinertL",[this]() -> Eigen::Vector3d { return approachObj->FinertL; });
			ctl.logger().addLogEntry("FloadL",[this]() -> Eigen::Vector3d { return approachObj->FloadL; });
			ctl.logger().addLogEntry("new threshL",[this]() -> Eigen::Vector3d { return approachObj->newThL; });
			ctl.logger().addLogEntry("FpullL",[this]() -> Eigen::Vector3d { return approachObj->FpullL; });


			ctl.logger().addLogEntry("efR Ace",[this]() -> Eigen::Vector3d { return efRAce; });
			ctl.logger().addLogEntry("FzeroR",[this]() -> Eigen::Vector3d { return approachObj->FzeroR; });
			ctl.logger().addLogEntry("FcloseR",[this]() -> Eigen::Vector3d { return approachObj->FcloseR; });
			ctl.logger().addLogEntry("FinertR",[this]() -> Eigen::Vector3d { return approachObj->FinertR; });
			ctl.logger().addLogEntry("FloadR",[this]() -> Eigen::Vector3d { return approachObj->FloadR; });
			ctl.logger().addLogEntry("new threshR",[this]() -> Eigen::Vector3d { return approachObj->newThR; });
			ctl.logger().addLogEntry("FpullR",[this]() -> Eigen::Vector3d { return approachObj->FpullR; });


			ctl.logger().addLogEntry("updateOffsetPosL",[this]() -> Eigen::Vector3d { return updateOffsetPosL; });
			ctl.logger().addLogEntry("updateOffsetPosR",[this]() -> Eigen::Vector3d { return updateOffsetPosR; });



			/*object geometric properties*/
			objLen = 0.9;
			objLenLt = objLen/2;
			objLenRt = objLenLt - (approachObj->objectPosC - approachObj->objectPosCy).norm();
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

			leftForceLo = ctl.robot().surfaceWrench("InternLeftHand").force();
			rightForceLo = ctl.robot().surfaceWrench("InternRightHand").force();


			/*chest pos and chest/body joint ori*/
			chestPosTask->position({0.032, 0.0, 1.12});
			chestOriTask->orientation(Eigen::Matrix3d::Identity());
			bodyOriTask->orientation(Eigen::Matrix3d::Identity());


			/*robot object contacts*/
			auto addContact_object_Gripper = [&](std::string gripperName)
			{ ctl.addContact({"hrp2_drc", "handoverObjects", gripperName, "handoverPipe"}); };

			auto removeContact_object_Gripper = [&](std::string gripperName)
			{ ctl.removeContact({"hrp2_drc", "handoverObjects", gripperName, "handoverPipe"}); };

			auto objHasContact = [&](std::string gripperName) -> bool
			{
				auto b = ctl.hasContact({"hrp2_drc", "handoverObjects", gripperName, "handoverPipe"});
				return b;
			};


			if(approachObj->addContacts)
			{
				approachObj->addContacts = false;
				addContact_object_Gripper("LeftGripper");
				addContact_object_Gripper("RightGripper");
			}


			if(approachObj->removeContacts)
			{
				approachObj->removeContacts = false;
				removeContact_object_Gripper("LeftGripper");
				removeContact_object_Gripper("RightGripper");

				ctl.solver().addTask(objEfTask);

				ctl.solver().addTask(posTaskL);
				ctl.solver().addTask(oriTaskL);

				ctl.solver().addTask(posTaskR);
				ctl.solver().addTask(oriTaskR);
			}

			approachObj->objHasContacts = objHasContact("LeftGripper") && objHasContact("RightGripper");


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


			auto gripperControl = [&]()
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
					close_gripper("l_gripper");
					close_gripper("r_gripper");
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

					auto robotRtHandOnObj = [&]()-> sva::PTransformd
					{
						al = (approachObj->objectPosC - approachObj->gripperEfR).norm();
						bl = abs(objLenLt - al);

						// al = .44;
						// bl = .25;

						if(al <= objLenLt)
						{
							if(al>=bl)
							{
								offsetLt << 0.0, -al, 0.0;
							}
							else
							{
								offsetLt << 0.0, bl, 0.0;
							}

							X_Obj0_offsetS = sva::PTransformd(offsetLt);
							X_M_Obj0 = sva::PTransformd(approachObj->subjLHandRot.transpose(), approachObj->fingerPosL);
							X_M_offsetR = X_Obj0_offsetS * X_M_Obj0;
							return X_M_offsetR;
						}
					};

					auto robotLtHandOnObj = [&]()-> sva::PTransformd
					{
						ar = (approachObj->objectPosCy - approachObj->gripperEfL).norm();
						br = abs(objLenRt - ar);

						// ar = .44;
						// br= .25;

						if(ar <= objLenRt)
						{
							if(ar>=br)
							{
								offsetRt << 0.0, ar/2, 0.0;
							}
							else
							{
								offsetRt << 0.0, -br/2, 0.0;
							}

							X_Obj0_offsetS = sva::PTransformd(offsetRt);
							X_M_Obj0 = sva::PTransformd(approachObj->subjRHandRot.transpose(), approachObj->fingerPosR);
							X_M_offsetL = X_Obj0_offsetS * X_M_Obj0;
							return X_M_offsetL;
						}
					};



					auto subjLtHandOnObj = [&]()-> sva::PTransformd
					{
						al = (approachObj->objectPosC - approachObj->fingerPosL).norm();
						bl = abs(objLenLt - al);

						if(al <= objLenLt)
						{
							if(al>=bl)
							{
								offsetLt << 0.0, 2*al/3, 0.0;
							}
							else
							{
								offsetLt << 0.0, -2*bl/3, 0.0;
							}
							X_M_Obj0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->fingerPosL);
						}
						else
						{
							//move to center pos of objLenLt/2
							offsetLt << 0.0, -objLenLt/2, 0.0;
							X_M_Obj0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->objectPosC);
						}

						X_Obj0_offsetEf = sva::PTransformd(offsetLt);
						X_M_offsetR = X_Obj0_offsetEf * X_M_Obj0;
						return X_M_offsetR;
					};

					auto subjRtHandOnObj = [&]()-> sva::PTransformd
					{
						ar = (approachObj->objectPosCy - approachObj->fingerPosR).norm();
						br = abs(objLenRt - ar);

						if(ar <= objLenRt)
						{
							if(ar>=br)
							{
								offsetRt << 0.0, -2*ar/3, 0.0;
							}
							else
							{
								offsetRt << 0.0, 2*br/3, 0.0;
							}
							X_M_Obj0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->fingerPosR);
						}
						else
						{
							//move to center pos of objLenRt/2
							offsetRt << 0.0, objLenRt/2, 0.0;
							X_M_Obj0 = sva::PTransformd(approachObj->objRot.transpose(), approachObj->objectPosC);
						}

						X_Obj0_offsetEf = sva::PTransformd(offsetRt);
						X_M_offsetL = X_Obj0_offsetEf * X_M_Obj0;
						return X_M_offsetL;
					};



					auto obj_rel_subjHands = [&]() -> std::vector<string>
					{
						// if(approachObj->obj_rel_subjRtHand < approachObj->obj_rel_subjLtHand)
						if(approachObj->virObj_rel_subjRtHand < approachObj->virObj_rel_subjLtHand)
						{
							subjMarkersName = approachObj->subjRtMarkers;
							fingerPos = approachObj->fingerPosR;

							ctl.solver().addTask(posTaskL);
							ctl.solver().addTask(oriTaskL);

							ctl.solver().removeTask(posTaskR);
							ctl.solver().removeTask(oriTaskR);
						}
						else
						{
							subjMarkersName = approachObj->subjLtMarkers;
							fingerPos = approachObj->fingerPosL;

							ctl.solver().addTask(posTaskR);
							ctl.solver().addTask(oriTaskR);

							ctl.solver().removeTask(posTaskL);
							ctl.solver().removeTask(oriTaskL);
						}
						return subjMarkersName;
					};


					auto obj_rel_subj = [&]()
					{
						if(approachObj->subjHasObject)
						{
							if(addTasks)
							{
								addTasks = false;
								removeTasks  = true;
								ctl.solver().addTask(objEfTask);
							}

							headTask->target(approachObj->objectPosC);
							objEfTask->set_ef_pose(sva::PTransformd(approachObj->objRot.transpose(), approachObj->objectPosC));
							subjLtHandOnObj();
							subjRtHandOnObj();
						}
						else if(approachObj->robotHasObject)
						{
							if(removeTasks)
							{
								removeTasks = false;
								addTasks = true;
								ctl.solver().removeTask(objEfTask);
								ctl.solver().removeTask(posTaskR);
								ctl.solver().removeTask(oriTaskR);
								posTaskL->stiffness(4.0);
								LOG_SUCCESS("begin 2nd cycle, motion enabled")
							}

							headTask->target(fingerPos);
							obj_rel_subjHands();
						}
					};


					/*track only subj right hand when robot carries the object*/
					auto obj_rel_robot = [&]() -> bool
					{
						obj_rel_subj();

						if(approachObj->subjHasObject)
						{
							approachObj->rHandPredict = approachObj->predictionController(rtPosW, relaxRotR, approachObj->subjLtMarkers);
							approachObj->useRightEf = get<0>(approachObj->rHandPredict);

							approachObj->lHandPredict = approachObj->predictionController(ltPosW, relaxRotL, approachObj->subjRtMarkers);
							approachObj->useLeftEf = get<0>(approachObj->lHandPredict);
						}
						else if(approachObj->robotHasObject)
						{
							if( subjMarkersName[0] == "lShapeRtA" )
							{
								approachObj->lHandPredict = approachObj->predictionController(ltPosW, relaxRotL, subjMarkersName);
								approachObj->useLeftEf = get<0>(approachObj->lHandPredict);
								approachObj->useRightEf = false;
							}
							else
							{
								approachObj->rHandPredict = approachObj->predictionController(rtPosW, relaxRotR, subjMarkersName);
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
					}


					/*feed Ef pose*/
					if( (approachObj->useLeftEf) || (approachObj->useRightEf) )
					{
						if(approachObj->subjHasObject)
						{
							updateOffsetPosL = X_M_offsetL.translation();
							approachObj->goToHandoverPose(-0.15, 0.75, approachObj->enableHand, ltPosW, posTaskL, oriTaskL, approachObj->lHandPredict, updateOffsetPosL);


							updateOffsetPosR = X_M_offsetR.translation();
							approachObj->goToHandoverPose(-0.75, 0.15, approachObj->enableHand, rtPosW, posTaskR, oriTaskR, approachObj->rHandPredict, updateOffsetPosR);
						}
						else if(approachObj->robotHasObject)
						{
							if(approachObj->useLeftEf)
							{
								// robotLtHandOnObj();
								// updateOffsetPosL = X_M_offsetL.translation();

								updateOffsetPosL = approachObj->fingerPosR;

								approachObj->goToHandoverPose(-0.15, 0.75, approachObj->enableHand, ltPosW, posTaskL, oriTaskL, approachObj->lHandPredict, updateOffsetPosL);

								// if((approachObj->i) % 400 == 0)
									// {LOG_INFO(" left Ef obj BLUE		" << updateOffsetPosL.transpose())};
								// {LOG_INFO(" left Ef obj BLUE		" << (posTaskL->position() /*- fingerPos*/).transpose())}
							}
							else if(approachObj->useRightEf)
							{
								// robotRtHandOnObj();
								// updateOffsetPosR = X_M_offsetR.translation();

								updateOffsetPosR = approachObj->fingerPosL;

								approachObj->goToHandoverPose(-0.75, 0.15, approachObj->enableHand, rtPosW, posTaskR, oriTaskR, approachObj->rHandPredict, updateOffsetPosR);

								// if((approachObj->i) % 400  == 0)
									// {LOG_ERROR(" right Ef obj RED		" << updateOffsetPosR.transpose())};
								// {LOG_ERROR(" right Ef obj RED		" << (posTaskR->position() /*- fingerPos*/).transpose())}
							}
						}


						/*check both gripper forces together*/
						approachObj->forceController(
							approachObj->enableHand,
							initPosR, initRotR,
							initPosL, initRotL,
							relaxPosR, relaxRotR,
							relaxPosL, relaxRotL,
							thresh,
							leftForce, rightForce,
							leftForceLo, rightForceLo,
							efLAce, efRAce,
							posTaskL, oriTaskL,
							posTaskR, oriTaskR,
							objEfTask);

						gripperControl();
					}

				}// handoverRun

			}//startCapture


			if(restartEverything)
			{
				dt+=1;

				/*reset moap markers*/
				for(unsigned int d=0; d<approachObj->totalMarkers; d++)
				{
					approachObj->Markers[d] << Eigen::Vector3d(0, 0, 0);
				}

				/*remove contacts*/
				ctl.removeContact({"hrp2_drc", "handoverObjects", "RightGripper", "handoverPipe"});
				ctl.removeContact({"hrp2_drc", "handoverObjects", "LeftGripper", "handoverPipe"});

				ctl.solver().addTask(objEfTask);

				ctl.solver().addTask(posTaskL);
				ctl.solver().addTask(oriTaskL);

				ctl.solver().addTask(posTaskR);
				ctl.solver().addTask(oriTaskR);

				auto  gripperL = ctl.grippers["l_gripper"].get();
				auto  gripperR = ctl.grippers["r_gripper"].get();

				gripperL->setTargetQ({openGrippers});
				gripperR->setTargetQ({openGrippers});

				posTaskL->stiffness(2.0);
				posTaskL->position(relaxPosL);
				oriTaskL->orientation(initRotL);

				posTaskR->stiffness(2.0);
				posTaskR->position(relaxPosR);
				oriTaskR->orientation(initRotR);


				addTasks = true;
				removeTasks = true;

				approachObj->addContacts = false;
				approachObj->removeContacts = false;

				approachObj->subjHasObject = true;
				approachObj->robotHasObject = false;

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
				approachObj->useRightEf = true;


				approachObj->local_FzeroL = Eigen::Vector3d::Zero();
				approachObj->newThL = Eigen::Vector3d::Zero();
				approachObj->FinertL = Eigen::Vector3d::Zero();
				approachObj->FzeroL = Eigen::Vector3d::Zero();
				approachObj->FcloseL = Eigen::Vector3d::Zero();
				approachObj->FloadL = Eigen::Vector3d::Zero();
				approachObj->FpullL = Eigen::Vector3d::Zero();


				approachObj->local_FzeroR = Eigen::Vector3d::Zero();
				approachObj->newThR = Eigen::Vector3d::Zero();
				approachObj->FinertR = Eigen::Vector3d::Zero();
				approachObj->FzeroR = Eigen::Vector3d::Zero();
				approachObj->FcloseR = Eigen::Vector3d::Zero();
				approachObj->FloadR = Eigen::Vector3d::Zero();
				approachObj->FpullR = Eigen::Vector3d::Zero();


				if( (dt%800) == 0 ) // 4 seconds
				{
					dt = 1;

					posTaskL->position(initPosL);
					posTaskR->position(initPosR);

					posTaskL->stiffness(4.0);
					posTaskR->stiffness(4.0);

					approachObj->enableHand = true;

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

			ctl.solver().removeTask(objEfTask);

			ctl.solver().removeTask(comTask);
		}

	} // namespace states
} // namespace mc_handover