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
			approachObj->initials();
			maxMarkers = approachObj->totalMarkers;


			// approachObj_sRt_rLt = std::make_shared<mc_handover::ApproachObject>();
			
			/*copy pointers*/
			// approachObj_sRt_rLt = approachObj;
			// approachObj_sRt_rRt = approachObj;

			// approachObj_sLt_rRt = approachObj;
			// approachObj_sLt_rLt = approachObj;


			/*close grippers for safety*/
			auto  gripperL = ctl.grippers["l_gripper"].get();
			auto  gripperR = ctl.grippers["r_gripper"].get();
			gripperL->setTargetQ({0.13});
			gripperR->setTargetQ({0.13});

			// ltRotW << -0.138901, -0.0699416, 0.987833,
			// 0.084922, 0.992987, 0.0822475,
			// -0.986658, 0.095313, -0.131987;

			// initial orientation
			ql = {0.95, 0.086, -0.25, -0.15};
			qr = {0.95, 0.086, -0.25, -0.15};
			
			//for mocap_temp
			q1l = {0.64, -0.01, -0.76, -0.06};
			q1r = {0.64, -0.01, -0.76, -0.06};

			p1l << 0.3,0.3,1.1;
			p1r <<0.3, -0.3,1.1;

			/*initial F/T thresholds*/
			thresh << 10, 10, 10, 6, 6, 6, 10, 10, 10, 6, 6, 6;
			/*initial force threshold*/
			leftTh = thresh.segment(3,3);
			rightTh = thresh.segment(9,3);

			/*chest pos task*/
			chestPosTask.reset(new mc_tasks::PositionTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(chestPosTask);
			chestPosTask->position({0.032, 0.0, 1.12});

			/*chest ori task*/
			chestOriTask.reset(new mc_tasks::OrientationTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(chestOriTask);
			chestOriTask->orientation(Eigen::Matrix3d::Identity());



			/*EfL pos Task*/
			ctl.posTaskL = make_shared<mc_tasks::PositionTask>("LARM_LINK7", ctl.robots(), 0, 3.0, 1e3);
			ctl.solver().addTask(ctl.posTaskL);

			/*EfL ori Task*/
			ctl.oriTaskL = make_shared<mc_tasks::OrientationTask>("LARM_LINK6",ctl.robots(), 0, 2.0, 1e2);
			ctl.solver().addTask(ctl.oriTaskL);

			LOG_INFO("leftEf pos " << ctl.posTaskL->position().transpose() << "\n"
				"leftEf ori " << ctl.oriTaskL->orientation() << "\n")
			initPosL = ctl.posTaskL->position();
			initOriL = ctl.oriTaskL->orientation();


			/*EfR pos Task*/
			ctl.posTaskR = make_shared<mc_tasks::PositionTask>("RARM_LINK7", ctl.robots(), 0, 3.0, 1e3);
			ctl.solver().addTask(ctl.posTaskR);

			/*EfL ori Task*/
			ctl.oriTaskR = make_shared<mc_tasks::OrientationTask>("RARM_LINK6",ctl.robots(), 0, 2.0, 1e2);
			ctl.solver().addTask(ctl.oriTaskR);

			LOG_INFO("rightEf pos " << ctl.posTaskR->position().transpose() << "\n"
				"rightEf ori " << ctl.oriTaskR->orientation() << "\n")
			initPosR = ctl.posTaskR->position();
			initOriR = ctl.oriTaskR->orientation();


			/*HeadTask*/
			Eigen::Vector3d headVector(1., 0, 0);
			Eigen::Vector3d headTarget(0.,0.3,1.);
			std::vector<std::string> activeJointsName = {"HEAD_JOINT0", "HEAD_JOINT1"};
			headTask.reset(new mc_tasks::LookAtTask("HEAD_LINK1", headVector, headTarget, ctl.robots(), ctl.robots().robotIndex(), 2., 500.));
			ctl.solver().addTask(headTask);
			headTask->selectActiveJoints(ctl.solver(), activeJointsName);



			/*Motion FOR CREATING MOCAP TEMPLATE*/
			ctl.gui()->addElement({"Handover", "randomPos"},
				mc_rtc::gui::Button("open_gripperL & set flags", [this, &ctl]()
				{
					auto gripper = ctl.grippers["l_gripper"].get();
					gripper->setTargetQ({0.5});//openGripper
					/*closeGripper = false; motion=true;*/
				}),
				mc_rtc::gui::Button("init*", [this, &ctl]()
				{
					ctl.posTaskL->position({0.0,0.37,0.72});
					ctl.oriTaskL->orientation(ql.toRotationMatrix().transpose());

					ctl.posTaskR->position({0.0,-0.37,0.72});
					ctl.oriTaskR->orientation(qr.toRotationMatrix().transpose());
				}),
				mc_rtc::gui::Button( "pos0", [this, &ctl](){
						ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());
						ctl.posTaskL->position(p1l);

						ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
						ctl.posTaskR->position(p1r);
				} ),
				mc_rtc::gui::Button( "pos1", [this, &ctl](){
					ctl.posTaskL->position({0.24,0.3,0.8});
					ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());

					ctl.posTaskR->position({0.24,-0.3,0.8});
					ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
				} ),
				mc_rtc::gui::Button( "pos2", [this, &ctl](){
					ctl.posTaskL->position({0.4,0.45,1.3});
					ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());

					ctl.posTaskR->position({0.4,-0.45,1.3});
					ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
				} ),
				mc_rtc::gui::Button( "pos3", [this, &ctl](){
					ctl.posTaskL->position({0.3,0.25,1.1});
					ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());

					ctl.posTaskR->position({0.3,-0.25,1.1});
					ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
				} ),
				mc_rtc::gui::Button( "pos4", [this, &ctl](){
					ctl.posTaskL->position({0.5,0.3,0.8});
					ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());

					ctl.posTaskR->position({0.5,-0.3,0.8});
					ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
				} ),
				mc_rtc::gui::Button( "pos5", [this, &ctl](){
					ctl.posTaskL->position({0.1,0.4,1.24});
					ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());

					ctl.posTaskR->position({0.1,-0.4,1.24});
					ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
				} ),
				mc_rtc::gui::Button( "pos6", [this, &ctl](){
					ctl.posTaskL->position({0.3,0.5,1.0});
					ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());

					ctl.posTaskR->position({0.3,-0.5,1.0});
					ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
				} ),
				mc_rtc::gui::Button( "pos7", [this, &ctl](){
					ctl.posTaskL->position({0.1,0.4,1.24});
					ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());

					ctl.posTaskR->position({0.5,-0.3,0.8});
					ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
				} ),
				mc_rtc::gui::Button( "pos8", [this, &ctl](){
					ctl.posTaskL->position({0.3,0.5,1.0});
					ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());

					ctl.posTaskR->position({0.4,-0.45,1.3});
					ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
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
			ctl.gui()->addElement({"Handover", "Trajectories"},

				// mc_rtc::gui::Trajectory("obseve subj pos", {{0, 1, 0}, 0.01, mc_rtc::gui::LineStyle::Solid},
				// 	[this, &ctl]() -> const vector<sva::PTransformd>& { return X_efL_S; }),

				// mc_rtc::gui::Trajectory("subj knuckle marker pos", {{1,1,0}, 0.01, mc_rtc::gui::LineStyle::Dotted},
				// 	[this]() -> const vector<Eigen::Vector3d>& { return approachObj->predictedPositions; } ),
				
				mc_rtc::gui::Trajectory("traj_l_wrist", {{1,0,1}, 0.01, mc_rtc::gui::LineStyle::Dotted},
					[this,&ctl](){ return ctl.robot().bodyPosW("LARM_LINK7").translation(); })
				);

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
			LOG_SUCCESS("***MOCAP IS ENABLED***")
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
			rightForce=ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
			// LOG_ERROR("leftWrenchWithoutGravity " <<leftForce.transpose() << " norm "<< leftForce.norm() )


			/*set com pose*/
			target = initialCom + move;
			comTask->com(target);


			/*Get non-stop MOCAP Frame*/
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


				// LOG_SUCCESS("nUnidentifiedMarkers "<<FrameofData.nUnidentifiedMarkers)
				if( approachObj->handoverRun() && (FrameofData.nUnidentifiedMarkers==0) )
				{
					/*move EF when subject approaches object 1st time*/
					if( !subjRtHandReady && (approachObj->obj_rel_subjRtHand < 0.2) )
					{
						ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());
						ctl.posTaskL->position(p1l);
						LOG_ERROR("subject right hand approaching object ")

						if(ctl.posTaskL->eval().norm() >0.5 || ctl.posTaskL->eval().norm() <0.1) 
						{
							subjRtHandReady=true;
						}
					}
					// else if( !subjLtHandReady && (approachObj->obj_rel_subjLtHand < 0.2) )
					// {
					// 	ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
					// 	ctl.posTaskR->position(p1r);
					// 	LOG_SUCCESS("subject left hand approaching object ")
					// 	if(ctl.posTaskR->eval().norm() >0.1 || ctl.posTaskR->eval().norm() <0.15) 
					// 	{
					// 		subjLtHandReady=true;
					// 	}
					// }


					/*Head Pose*/
					headTask->target(approachObj->objectPos);



					/*observe subject motion for t_observe period*/
					if( ((approachObj->i)%(approachObj->t_observe)==0) )
					{
						approachObj->collected = approachObj->predictionController(p1l, q1l, subjRtHandReady, ltHand, ltRotW, approachObj->lShapeRtMarkers, approachObj->robotLtMarkers);
					}



					/*feed Ef pose*/
					if( approachObj->collected )
					{
						if(	subjRtHandReady && (approachObj->objectPos(1)> 0.15) && (approachObj->objectPos(1)<= 0.7) )
						{
							taskOK = approachObj->goToHandoverPose(ltHand, ctl.oriTaskL, ctl.posTaskL);
							ctl.posTaskR->position(initPosR);
							ctl.oriTaskR->orientation(initOriR);
						}
						else if( subjLtHandReady && (approachObj->objectPos(1)>= -0.7) && (approachObj->objectPos(1)<= 0.15) )
						{
							taskOK = approachObj->goToHandoverPose(rtHand, ctl.oriTaskR, ctl.posTaskR);
							ctl.posTaskL->position(initPosL);
							ctl.oriTaskL->orientation(initOriL);
						}
					}


					/*force based handover control*/
					taskOK = approachObj->handoverForceController(leftForce, leftTh, "l_gripper", approachObj->lShapeRtMarkers, approachObj->robotLtMarkers);


				}// handoverRun
			}
			return false;// startCapture
		}// run


		void StartMocapStep::teardown(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.solver().removeTask(comTask);
			
			ctl.solver().removeTask(chestPosTask);
			ctl.solver().removeTask(chestOriTask);

			ctl.solver().removeTask(ctl.posTaskL);
			ctl.solver().removeTask(ctl.oriTaskL);
			
			ctl.solver().removeTask(ctl.posTaskR);
			ctl.solver().removeTask(ctl.oriTaskR);

			ctl.gui()->removeCategory({"Handover"});
		}

	} // namespace states
} // namespace mc_handover




// 	Eigen::Matrix3d from_rot = X_efL_Subj.rotation(); //ctl.oriTaskL->orientation(); //q1l.toRotationMatrix().transpose();
// 	Eigen::Matrix3d to_rot = subjLtHandRot.transpose();
// 	Eigen::Vector3d er_the = (RadToDeg)*sva::rotationError(from_rot,to_rot);
// LOG_ERROR(er_the.transpose())

// Eigen::Matrix3d handoverRot = sva::RotX(er_the(0)) * sva::RotY(er_the(1)) * sva::RotZ(er_the(2));
// sva::PTransformd new_pose(handoverRot,handoverPos);
// ctl.oriTaskL->orientation(new_pose.rotation());
// ctl.posTaskL->position(new_pose.translation());
// // ctl.oriTaskL->orientation(togo_);
// // ctl.posTaskL->position(handoverPos);



// Eigen::Matrix3d my_angles = ctl.oriTaskL->orientation();
// LOG_WARNING("my XYZ angles  " <<
// (RadToDeg)*atan2(my_angles(2,1), my_angles(2,2)) << " "<<
// (RadToDeg)*atan2(-my_angles(2,0), sqrt( pow( my_angles(2,1),2) + pow(my_angles(2,2),2) ) ) << " "<<
// (RadToDeg)*atan2(my_angles(1,0), my_angles(0,0)) )



/*try below methods for rotation*/
// /*reverse Z*/
// subjLtHandRot.col(0) = lshpLt_X;
// subjLtHandRot.col(1) = lshpLt_Y;
// subjLtHandRot.col(2) = lshpLt_Z;

//http://www.continuummechanics.org/rotationmatrix.html
//https://www.youtube.com/watch?v=lVjFhNv2N8o 7.7min
//http://www.songho.ca/opengl/gl_anglestoaxes.html

/*try below methods*/
//https://math.stackexchange.com/questions/180418/calculate-rotation-matrix-to-align-vector-a-to-vector-b-in-3d
//http://www.euclideanspace.com/maths/geometry/affine/conversions/quaternionToMatrix/index.htm
//http://www.euclideanspace.com/maths/geometry/affine/aroundPoint/




	// LOG_INFO(approachObj->obj_rel_subjRtHand);
	// LOG_ERROR(approachObj->obj_rel_subjRtHand);

	// approachObj->obj_rel_subjRtHand = approachObj->obj_rel_subjRtHand;

	// LOG_WARNING(approachObj->obj_rel_subjRtHand);


	// approachObj->dum1 = false;
	// LOG_ERROR(approachObj->dum1);
	// LOG_INFO(approachObj->dum1);

	// approachObj->dum1 = true;
	// LOG_SUCCESS(approachObj_sLt_rRt->dum1);
	// LOG_INFO(approachObj->dum1);