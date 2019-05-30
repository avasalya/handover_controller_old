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
				else if( c < 18)/*used markers count*/
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

				mc_rtc::gui::Button("LEFT ARM start pose", [this, &ctl]()
				{
					posTaskL->position(constPosL);
					oriTaskL->orientation(constRotL);
				}),

				mc_rtc::gui::Button("RIGHT ARM start pose", [this, &ctl]()
				{
					posTaskR->position(constPosR);
					oriTaskR->orientation(constRotR);
				}),

				mc_rtc::gui::Button("LEFT ARM half-sit", [this, &ctl]()
				{
					posTaskL->position(initPosL);
					oriTaskL->orientation(initRotL);
				}),

				mc_rtc::gui::Button("RIGHT ARM half-sit", [this, &ctl]()
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

			/*move object using cursor or simData*/
			ctl.gui()->addElement({"Handover","cursor_move_object"},

				mc_rtc::gui::Transform("Position base", 
					[this,&ctl](){ return ctl.robots().robot(2).bodyPosW("base_link"); },					
					[this,&ctl] (const sva::PTransformd & pos) { ctl.robots().robot(2).posW(pos); } 
				));

			/*display moving object*/
			// ctl.gui()->addElement({"Handover","Point3D_object"},

				// mc_rtc::gui::Point3D("base link", [this,&ctl](){
				// 	ctl.robots().robot(2).posW({approachObj->object[1]});
				// 	return ctl.robots().robot(2).bodyPosW("base_link").translation(); })

				// mc_rtc::gui::Point3D("object position", [this,&ctl](){ ctl.robots().robot(2).posW({approachObj->objectPos}); return approachObj->objectPos;})
				// );

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


			/*add handoverpipe/ground contact*/
			ctl.addContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround", Eigen::Vector6d::Ones()});


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
			
			ctl.logger().addLogEntry("objectPos",[this]()-> Eigen::Vector3d { return approachObj->objectPos; });
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



			posTaskL->position(constPosL);
			posTaskR->position(constPosR);

			oriTaskL->orientation(constRotL);
			oriTaskR->orientation(constRotR);

		}// start


		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/*hand pose*/
			ltPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")].translation();
			ltRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")].rotation();

			rtPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK7")].translation();
			rtRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")].rotation();
			

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


			/*Force sensor*/
			leftForce = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
			rightForce = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();


			/*set com pose*/
			target = initialCom + move;
			comTask->com(target);


			auto open_gripper = [&](std::string gripperName)
			{
				auto gripper = ctl.grippers[gripperName].get();
				gripper->setTargetQ({0.5});

				approachObj->gOpen = false;
				approachObj->openGripper = true;
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


			auto addContact_object_ground = [&]()
			{ ctl.addContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"}); };


			auto removeContact_object_ground = [&]()
			{ ctl.removeContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"}); };


			auto addContact_object_Gripper = [&](std::string gripperName)
			{ ctl.addContact({"hrp2_drc", "handoverObjects", gripperName, "handoverPipe"}); };


			auto removeContact_object_Gripper = [&](std::string gripperName)
			{ ctl.removeContact({"hrp2_drc", "handoverObjects", gripperName, "handoverPipe"}); };


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

					/*move EF when subject approaches object 1st time*/
					// if((!startHandover) && (approachObj->obj_rel_subjRtHand < 0.2) && (approachObj->obj_rel_subjLtHand < 0.2))
					// {
					// 	posTaskL->position(p1l);
					// 	vecOriTaskL->bodyVector({0., 1., 0.});
					// 	vecOriTaskL->targetVector(targetVector);

					// 	posTaskR->position(p1r);
					// 	vecOriTaskR->bodyVector({0., -1., 0.});
					// 	vecOriTaskR->targetVector(targetVector);
					// 	LOG_WARNING("subject both hands approaching object ")
					// 	if(
					// 		(posTaskR->eval().norm() >0.5 || posTaskR->eval().norm() <0.1) &&
					// 		(posTaskL->eval().norm() >0.5 || posTaskL->eval().norm() <0.1)
					// 		)
					// 		{ startHandover=true; }
					// }




					/*observe subject motion for t_observe period*/
					if( (approachObj->i)%(approachObj->t_observe) == 0 )
					{
						approachObj->lHandPredict = approachObj->predictionController(ltPosW, constRotL, approachObj->subjRtMarkers);
						approachObj->useLeftEf = get<0>(approachObj->lHandPredict);

						approachObj->rHandPredict = approachObj->predictionController(rtPosW, constRotR, approachObj->subjLtMarkers);
						approachObj->useRightEf = get<0>(approachObj->rHandPredict);

					}// i%t_observe;



					/*feed Ef pose*/
					if( (approachObj->useLeftEf) && (approachObj->useRightEf) )
					{
						auto min = approachObj->object[2][1] + 0.05; //half width of ef
						taskOK = approachObj->goToHandoverPose("left", min, 0.7, approachObj->enableLHand, ltPosW, posTaskL, oriTaskL, approachObj->lHandPredict, approachObj->fingerPosL);


						auto max = approachObj->object[0][1] - 0.05; //half width of ef
						taskOK = approachObj->goToHandoverPose("right", -0.7, max, approachObj->enableRHand, rtPosW, posTaskR, oriTaskR, approachObj->rHandPredict, approachObj->fingerPosR);



						taskOK = approachObj->forceController(approachObj->enableLHand, initPosL, initRotL, leftForce, leftTh, efLAce, posTaskL, oriTaskL, "l_gripper", approachObj->robotLtMarkers, approachObj->subjRtMarkers, approachObj->obj_rel_robotLtHand);
							gripperControl("l_gripper");

						taskOK = approachObj->forceController(approachObj->enableRHand, initPosR, initRotR, rightForce, rightTh, efRAce, posTaskR, oriTaskR, "r_gripper", approachObj->robotLtMarkers, approachObj->subjLtMarkers, approachObj->obj_rel_robotRtHand);
							gripperControl("r_gripper");
					}


					/*Head Pose*/
					headTask->target(approachObj->object[1]); //object center marker

				}// handoverRun

			}//startCapture



			if(restartEverything)
			{
				/*remove contacts*/
				addContact_object_ground();
				removeContact_object_Gripper("LeftGripper");
				removeContact_object_Gripper("RightGripper");

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

				approachObj->e = 1;

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

				approachObj->pickaHand = false;

				approachObj->newTh = Eigen::Vector3d::Zero();
				approachObj->Finert = Eigen::Vector3d::Zero();
				approachObj->Fzero = Eigen::Vector3d::Zero();
				approachObj->Fclose = Eigen::Vector3d::Zero();
				approachObj->Fload = Eigen::Vector3d::Zero();
				approachObj->Fpull = Eigen::Vector3d::Zero();


				if( (posTaskL->eval().norm() <0.1)  && (posTaskR->eval().norm() <0.1) )
				{
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