#include "StartMocapStep.h"

namespace mc_handover
{

	namespace states
	{

		void StartMocapStep::configure(const mc_rtc::Configuration & config)
		{
			// thresh 	= config("handsWrenchTh%");
			thresh 	= config("handsWrenchTh");
			baseTh 	= config("handsWrenchBaseTh");
			handsWrenchDir = config("handsWrenchDir");
		}



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

			/*close grippers for safety*/
			auto  gripperL = ctl.grippers["l_gripper"].get();
			auto  gripperR = ctl.grippers["r_gripper"].get();
			gripperL->setTargetQ({closeGrippers});
			gripperR->setTargetQ({closeGrippers});

			q  = {0.64, -0.01, -0.76, -0.06}; // initial orientation
			q1 = {0.95, 0.086, -0.25, -0.15}; //for mocap_temp
			q2 = {-0.42, -0.5, 0.56, 0.49}; // for extra orientation (-90)
			q3 = {-0.44, 0.47, 0.49, 0.58}; //to get closer to body

			/*chest pos task*/
			chestPosTask.reset(new mc_tasks::PositionTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(chestPosTask);
			// cout<< chestPosTask->position().transpose()<<endl;
			chestPosTask->position({0.032, 0.0, 1.12});

			/*chest ori task*/
			chestOriTask.reset(new mc_tasks::OrientationTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(chestOriTask);
			// cout<<chestOriTask->orientation()<<endl;
			chestOriTask->orientation(Eigen::Matrix3d::Identity());


			/*EfL pos Task*/
			ctl.posTaskL = std::make_shared<mc_tasks::PositionTask>("LARM_LINK7", ctl.robots(), 0, 3.0, 1e3);
			ctl.solver().addTask(ctl.posTaskL);
			// ctl.posTaskL->position({0.3,0.3,1.1}); 										//COMMENT LATER

			/*EfL ori Task*/
			ctl.oriTaskL = std::make_shared<mc_tasks::OrientationTask>("LARM_LINK6",ctl.robots(), 0, 2.0, 1e2);
			ctl.solver().addTask(ctl.oriTaskL);
			// ctl.oriTaskL->orientation(q.toRotationMatrix().transpose()); 				//COMMENT LATER


			/*EfR pos Task*/
			ctl.posTaskR = std::make_shared<mc_tasks::PositionTask>("RARM_LINK7", ctl.robots(), 0, 3.0, 1e3);
			ctl.solver().addTask(ctl.posTaskR);
			// cout << ctl.posTaskR->position().transpose() <<endl;
			ctl.posTaskR->position({0.060, -0.373, 0.724});

			/*Handover buttons*/
			ctl.gui()->addElement({"Handover", "Buttons"},
				mc_rtc::gui::Button("B1", [this](){option1=true; option2=false; option3=false;}),
				mc_rtc::gui::Button("B2", [this](){option1=false; option2=true; option3=false;}),
				mc_rtc::gui::Button("B3", [this](){option1=false; option2=false; option3=true;})
				);

			/*Motion FOR CREATING MOCAP TEMPLATE*/
			ctl.gui()->addElement({"Handover", "randomPos"},
				mc_rtc::gui::Button("init*", [this, &ctl]()
				{
					ctl.posTaskL->position({0.0,0.37,0.72});
					auto gripper = ctl.grippers["l_gripper"].get();
					gripper->setTargetQ({closeGrippers});
					ctl.oriTaskL->orientation(q1.toRotationMatrix().transpose());
				}),

				mc_rtc::gui::Button( "pos1", [this, &ctl](){ ctl.posTaskL->position({0.2,0.3,1.4});
					ctl.oriTaskL->orientation(q.toRotationMatrix().transpose()); } ),
				mc_rtc::gui::Button( "pos2", [this, &ctl](){ ctl.posTaskL->position({0.4,0.6,1.3});
					ctl.oriTaskL->orientation(q.toRotationMatrix().transpose()); } ),
				mc_rtc::gui::Button( "pos3", [this, &ctl](){ ctl.posTaskL->position({0.3,0.4,1.1}); 
					ctl.oriTaskL->orientation(q.toRotationMatrix().transpose()); } ),
				mc_rtc::gui::Button( "pos4", [this, &ctl](){ ctl.posTaskL->position({0.5,0.6,0.9}); 
					ctl.oriTaskL->orientation(q.toRotationMatrix().transpose()); } ),
				mc_rtc::gui::Button( "pos5", [this, &ctl](){ ctl.posTaskL->position({0.1,0.3,1.35}); 
					ctl.oriTaskL->orientation(q.toRotationMatrix().transpose()); } ),
				mc_rtc::gui::Button( "pos6", [this, &ctl](){ ctl.posTaskL->position({0.3,0.5,1.0}); 
					ctl.oriTaskL->orientation(q.toRotationMatrix().transpose()); } )

				// , mc_rtc::gui::Transform("random_pos", 
				// 	[this,&ctl](){ return ctl.robots().robot(0).bodyPosW("LARM_LINK7"); },
				// 	[this,&ctl](const sva::PTransformd & pos){ctl.posTaskL->position(pos.translation());} ),

				// , mc_rtc::gui::ArrayInput("random_pos",
				// 	{"x", "y", "z"},
				// 	[this]() { return randPos; },
				// 	[this, &ctl](const Eigen::Vector3d & t){randPos = t; ctl.posTaskL->position(randPos);} )
				);


			/*publish wrench*/
			ctl.gui()->addElement({"Handover", "wrench"},

				mc_rtc::gui::Button("publish_current_wrench", [&ctl]() {
					std::cout << "left hand Forces " <<
					ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					std::cout << "right hand Forces " <<
					ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
				}),
				
				mc_rtc::gui::Button("Norm_LeftEf_Force",[this, &ctl](){
					Eigen::Vector3d v = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					cout<<"Norm "<< v.norm() <<endl; }),

				mc_rtc::gui::ArrayInput("set Threshold %",
					{"Left cx", "cy", "cz", "fx", "fy", "fz", "Right cx", "cy", "cz", "fx", "fy", "fz"},
					[this]() { return thresh; },
					[this](const Eigen::VectorXd & t)
					{
						LOG_INFO("Changed threshold percnt to:\nLeft: " << t.head(6).transpose() << "\nRight: " << t.tail(6).transpose() << "\n")
						thresh = t;
					}),

				mc_rtc::gui::ArrayInput("Base Threshold",
					{"Left cx", "cy", "cz", "fx", "fy", "fz", "Right cx", "cy", "cz", "fx", "fy", "fz"},
					[this]() { return baseTh; },
					[this](const Eigen::VectorXd & bt)
					{
						LOG_INFO("Changed Base threshold to:\nLeft: " << bt.head(6).transpose() << "\nRight: " << bt.tail(6).transpose() << "\n")
						baseTh = bt;
					}));


			/*move object using cursor or simData*/
			ctl.gui()->addElement({"Handover","move_object"},
				mc_rtc::gui::Transform("Position", 
					[this,&ctl](){ return ctl.robots().robot(2).bodyPosW("base_link"); },
					[this,&ctl](const sva::PTransformd & pos) { 
						ctl.robots().robot(2).posW(pos);
						ctl.removeContact({"handoverobjects", "ground", "handoverPipeBottom", "AllGround"});
						ctl.addContact({"handoverobjects", "ground", "handoverPipeBottom", "AllGround"});
					})
				, mc_rtc::gui::Button("Replay", [this](){ i = 0;}),
				mc_rtc::gui::Point3D("subj fing pos", [this,&ctl](){ ctl.robots().robot(2).posW({Subj_X_efL.translation()}); return Subj_X_efL.translation();})
				);


			/*change prediction_ settings*/
			ctl.gui()->addElement({"Handover", "tuner"},
				mc_rtc::gui::ArrayInput("t_predict/t_observe", {"t_predict", "t_observe", "it"},
					[this]() { return tuner; }, 
					[this](const Eigen::Vector3d & to){tuner = to;cout<< "t_predict = " << tuner(0)*1/fps<< "sec, t_observe = "<<tuner(1)*1/fps<< "sec"<<endl;}));

			tuner << 400., 20., 60.; 
			t_predict = (int)tuner(0);
			t_observe = (int)tuner(1);
			it = (int)tuner(2);
			newPosSubj = Eigen::MatrixXd::Zero(3, t_observe);


			/*com Task*/
			ctl.gui()->addElement({"Handover", "com"},

				mc_rtc::gui::ArrayInput("Move Com Pos", {"x", "y", "z"},
					[this]() { return move; },
					[this](const Eigen::Vector3d v) { move = v;
						cout << " com pos set to:\n" << initialCom + move << endl;})
				);

			comTask = std::make_shared<mc_tasks::CoMTask>
			(ctl.robots(), ctl.robots().robotIndex(), 10., 1000.);
			// comTask->dimWeight(Eigen::Vector3d(1., 1., 1.));

			initialCom = rbd::computeCoM(ctl.robot().mb(),ctl.robot().mbc());
			comTask->com(initialCom);
			ctl.solver().addTask(comTask);


			/*trajectory trail*/
			ctl.gui()->addElement({"Handover", "Trajectories"},

				// mc_rtc::gui::Trajectory("obseve subj pos", {{0, 1, 0}, 0.01, mc_rtc::gui::LineStyle::Solid},
				// 	[this, &ctl]() -> const std::vector<sva::PTransformd>& { return S_X_efL; }),

				mc_rtc::gui::Trajectory("subj knuckle marker pos", {{1,1,0}, 0.01, mc_rtc::gui::LineStyle::Dotted},
					[this, &ctl]() -> const std::vector<Eigen::Vector3d>& { return predictedPositions; } ),
				
				mc_rtc::gui::Trajectory("traj_l_wrist", {{1,0,1}, 0.01, mc_rtc::gui::LineStyle::Dotted},
					[this,&ctl](){ return ctl.robot().bodyPosW("LARM_LINK7").translation(); })

				);


			/*configure MOCAP*/
			if(Flag_CORTEX)
			{
				LOG_INFO("***MOCAP IS ENABLED***")
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
				LOG_WARNING("***MOCAP IS DISABLED***")
				startCapture = false; //true for sim
				
				name = {"simData3"};
				std::string fn = std::string(DATA_PATH) + "/" + name + ".txt";
				std::ifstream file(fn);

				if(!file.is_open())
					{ LOG_ERROR("Failed to open ") }

				while(file >> pt)
					{ pts.push_back(pt); }
				
				pos.resize(maxMarkers);
				for(int m=0; m<maxMarkers; m++)
				{	
					pos[m] = Eigen::MatrixXd(3, pts.size()/36);

					for(size_t i = 0; i < pts.size(); i += 36)
					{
						pos[m](0, i/36) = (pts[i]);
						pos[m](1, i/36) = (pts[i+1]);
						pos[m](2, i/36) = (pts[i+2]);
					}
				}
			}

			/*allocate memory for mocap markers*/
			Markers.resize(maxMarkers);
			markersPos.resize(maxMarkers);

			predictedPositions.resize(1);
			S_X_efL.resize(t_observe);

			for(int m=0; m<maxMarkers; m++)
				{ markersPos[m] = Eigen::MatrixXd::Zero(3,60000); }


			/*initil force threshold*/
			leftThAtGrasp = thresh.segment(3,3);
		}// start



		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/*hand pose*/
			ltHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")];
			rtHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK7")];


			/*set com pose*/
			target = initialCom + move;
			comTask->com(target);


			/*Get non-stop MOCAP Frame*/
			if(Flag_CORTEX)
			{
				getCurFrame =  Cortex_GetCurrentFrame();
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
				if(Flag_CORTEX)
				{
					for(int m=0; m<maxMarkers; m++)
					{
						Markers[m] <<
						FrameofData.BodyData[body].Markers[m][0], // X
						FrameofData.BodyData[body].Markers[m][1], // Y
						FrameofData.BodyData[body].Markers[m][2]; // Z
					}
				}
				else /*simulation*/
				{
					for(int m=0; m<maxMarkers; m++)
					{ 
							// Markers[m] = pos[m].col(s);
						Markers[m] = ctl.robots().robot(2).bodyPosW("base_link").translation();
					}
				}


				/*check for non zero frame only and store them*/
				if(	Markers[wristLtEfA](0)!=0 && Markers[wristLtEfA](0)< 20
					&&  Markers[object](0)!=0 && Markers[object](0)< 20
					&&  Markers[fingerSubjLt](0)!=0 && Markers[fingerSubjLt](0)< 20
					)
				{
					for(int m=0; m<maxMarkers; m++)
						{ markersPos[m].col(i) << Markers[m]; }
					// cout << "Markers[4] " << Markers[4].transpose()<<endl;

					/*direction vectors, projections and area*/
					auto lEf_wA_wB = markersPos[wristLtEfA].col(i)-markersPos[wristLtEfB].col(i);
					auto lEf_wA_gA = markersPos[wristLtEfA].col(i)-markersPos[gripperLtEfA].col(i);
					auto lEf_wA_gB = markersPos[wristLtEfA].col(i)-markersPos[gripperLtEfB].col(i);
					auto lEf_wA_O  = markersPos[wristLtEfA].col(i)-markersPos[object].col(i);
					auto lEf_wA_lf  = markersPos[wristLtEfA].col(i)-markersPos[fingerSubjLt].col(i);

					// auto lEf_wAgB_theta_wAgA = acos( lEf_wA_gB.dot(lEf_wA_gA)/( lEf_wA_gB.norm()*lEf_wA_gA.norm() ) );
					auto lEf_wAB_theta_wAgA = acos( lEf_wA_wB.dot(lEf_wA_gA)/( lEf_wA_wB.norm()*lEf_wA_gA.norm() ) );
					auto lEf_wAB_theta_wAgB = acos( lEf_wA_wB.dot(lEf_wA_gB)/( lEf_wA_wB.norm()*lEf_wA_gB.norm() ) );
					auto lEf_wAB_theta_wAO = acos( lEf_wA_wB.dot(lEf_wA_O)/( lEf_wA_wB.norm()*lEf_wA_O.norm() ) );
					auto lEf_wAB_theta_wAf = acos( lEf_wA_wB.dot(lEf_wA_lf)/( lEf_wA_wB.norm()*lEf_wA_lf.norm() ) );
					
					// auto lEf_area_gAB_wA = 0.5*lEf_wA_gB.norm()*lEf_wA_gA.norm()*sin(lEf_wAgB_theta_wAgA);
					auto lEf_area_wAB_gA = 0.5*lEf_wA_wB.norm()*lEf_wA_gA.norm()*sin(lEf_wAB_theta_wAgA);
					auto lEf_area_wAB_gB = 0.5*lEf_wA_wB.norm()*lEf_wA_gB.norm()*sin(lEf_wAB_theta_wAgB);
					auto lEf_area_wAB_O  = 0.5*lEf_wA_wB.norm()*lEf_wA_O.norm()*sin(lEf_wAB_theta_wAO);
					auto lEf_area_wAB_f  = 0.5*lEf_wA_wB.norm()*lEf_wA_lf.norm()*sin(lEf_wAB_theta_wAf);


					/*get rotation matrix XYZ of subject LEFT hand*/							  //  B  ________\C
					Vector3d lshpLt_X = markersPos[lShapeLtB].col(i)-markersPos[lShapeLtA].col(i);//    |        /
					Vector3d lshpLt_Y = markersPos[lShapeLtB].col(i)-markersPos[lShapeLtC].col(i);//    |
					Vector3d lshpLt_Z = (lshpLt_X/lshpLt_X.norm()).cross(lshpLt_Y/lshpLt_Y.norm());// A\|/

					subjLtHandRot.row(0) = (lshpLt_X/lshpLt_X.norm()).transpose();
					subjLtHandRot.row(1) = (lshpLt_Y/lshpLt_Y.norm()).transpose();
					subjLtHandRot.row(2) = (lshpLt_Z/lshpLt_Z.norm()).transpose();


					// /*get rotation matrix XYZ of subject RIGHT hand*/  							//C /________ B
					// Vector3d lshpRt_X = markersPos[lShapeRtB].col(i)-markersPos[lShapeRtA].col(i);// \        |
					// Vector3d lshpRt_Y = markersPos[lShapeRtB].col(i)-markersPos[lShapeRtC].col(i);//          |
					// Vector3d lshpRt_Z = (lshpRt_X/lshpRt_X.norm()).cross(lshpRt_Y/lshpRt_Y.norm());//        \|/A

					// subjRtHandRot.row(0) = (lshpRt_X/lshpRt_X.norm()).transpose();
					// subjRtHandRot.row(1) = -(lshpRt_Y/lshpRt_Y.norm()).transpose();
					// subjRtHandRot.row(2) = (lshpRt_Z/lshpRt_Z.norm()).transpose();

					
					/*move EF when subject is approaches object*/
					if( oneTime && (markersPos[fingerSubjLt].col(i)-markersPos[object].col(i)).norm()<0.5 )
					{
						oneTime=false;
						ctl.oriTaskL->orientation(q.toRotationMatrix().transpose());
						ctl.posTaskL->position({0.3,0.25,1.1});
					}


					if( (i%t_observe==0) )
					{
						/*prediction_ tuner*/
						t_predict = (int)tuner(0);
						t_observe = (int)tuner(1);
						it = (int)tuner(2);

						/*get robot ef current pose*/
						curRotLeftEf = ltHand.rotation();
						curPosLeftEf = ltHand.translation();
						// sva::PTransformd R_X_efL(curPosLeftEf);
						sva::PTransformd R_X_efL(curPosLeftEf);

						/*get robot ef marker(s) current pose*/
						auto efLGripperPos = 0.25*( markersPos[wristLtEfA].col((i-t_observe)+1) + markersPos[wristLtEfB].col((i-t_observe)+1) +
							markersPos[gripperLtEfA].col((i-t_observe)+1) + markersPos[gripperLtEfB].col((i-t_observe)+1) );
						curPosLeftEfMarker << efLGripperPos;
						sva::PTransformd M_X_efLMarker(curPosLeftEfMarker);

						/*subj marker(s) pose w.r.t to robot EF frame*/
						for(int j=1;j<=t_observe; j++)
						{
							rotSubj = Eigen::Matrix3d::Identity();
							sva::PTransformd M_X_Subj(rotSubj, markersPos[fingerSubjLt].middleCols((i-t_observe)+j,i));

							Subj_X_efL = R_X_efL.inv()*M_X_Subj*M_X_efLMarker.inv()*R_X_efL;

							S_X_efL[j-1] = Subj_X_efL;
							// cout << "S_X_efL "<<S_X_efL[j-1].translation().transpose()<<endl;

							newPosSubj.col(j-1) = Subj_X_efL.translation();
							
							if(j==t_observe)
								{ ithPosSubj = newPosSubj.col(t_observe-1); }
						}

						/*get average velocity of previous *t_observe* sec Subj motion*/
						curVelSubj  = ctl.handoverTraj->diff(newPosSubj)*fps;//ignore diff > XXXX
						avgVelSubj  << ctl.handoverTraj->takeAverage(curVelSubj);

						/*predict position in straight line after t_predict time*/
						predictPos = ctl.handoverTraj->constVelocityPredictPos(avgVelSubj, ithPosSubj, t_predict);
						predictedPositions[0] << predictPos;
						// cout << "predicted pos " << predictPos.transpose()<<endl;

						/*get predicted way points between left ef and Subj*/	/*** GET NEW VELOCITY PROFILE ****/
						wp_efL_Subj=ctl.handoverTraj->constVelocity(ithPosSubj, predictPos, t_predict);
						wp = get<0>(wp_efL_Subj);

						initRefPos << wp(0,it), wp(1,it), wp(2,it);

						collected = true;
					}//t_observe


					/*feed Ef pose*/
					if( collected )
					{
						it+= (int)tuner(2);//40+(int)t_predict/t_observe;

						auto curLEfPos = ltHand.translation();

						if(it<=wp.cols())
						{
							refPos << wp(0,it), wp(1,it), wp(2,it);

							// handoverPos = curLEfPos + refPos - initRefPos;


							if(option1)
							{
								handoverPos = curLEfPos + refPos - initRefPos;
							}
							else if(option2)
							{
								handoverPos = curLEfPos + refPos;
							}
							else if(option3)
							{
								handoverPos = refPos;
							}


							// /*robot constraint*/
							// if(	(handoverPos(0)>= 0.20) && (handoverPos(0)<= 0.7) && 
							// 	(handoverPos(1)>= 0.05) && (handoverPos(1)<= 0.7) &&
							// 	(handoverPos(2)>= 0.90) && (handoverPos(2)<= 1.5) )
							// {
							// 	/*control head*/
							// 	if(handoverPos(1) >.45){ctl.set_joint_pos("HEAD_JOINT0",  0.8);} //y //+ve to move head left
							// 	else{ctl.set_joint_pos("HEAD_JOINT0",  0.); }//-ve to move head right

							// 	if(handoverPos(2) < 1.1){ctl.set_joint_pos("HEAD_JOINT1",  0.6);} //z //+ve to move head down
							// 	else{ctl.set_joint_pos("HEAD_JOINT1",  -0.4);} //-ve to move head up

							// 	/*handover pose*/
							// 	if(motion)
							// 	{
							// 		ctl.posTaskL->position(handoverPos);
							// 		// ctl.oriTaskL->orientation(q.toRotationMatrix().transpose());

							// 		if( (handoverPos(0)<= 0.4) && (handoverPos(1)<= 0.25) )
							// 		{
							// 			ctl.oriTaskL->orientation(q3.toRotationMatrix().transpose());
							// 		}
							// 		else
							// 		{ ctl.oriTaskL->orientation(q.toRotationMatrix().transpose()); }
							// 	}
							// }

							if(it==wp.cols())
								{ collected  = false; }
						}
					}//collected


					/*gripper control*/
					auto close_gripperL = [&]()
					{
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({closeGrippers});
					};
					auto open_gripperL = [&]()
					{
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({openGrippers});
					};


					/*Force sensor*/
					leftForce = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					leftTh = thresh.segment(3,3);

					/*force control*/
					auto checkForce = [&](const char *axis_name, int idx)
					{
						/*check each force individually*/
						leftForcesAtGrasp = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
						// leftForceNormAtGrasp = leftForcesAtGrasp.norm();

						/*check if forces are already greater than default thresholds*/
						if( abs(leftForcesAtGrasp(0))>=leftTh[0] )
						{
							leftThAtGrasp[0] = leftTh[0] + abs(leftForcesAtGrasp(0))-leftTh[0] + 1.0;
						}
						else if( abs(leftForcesAtGrasp(1))>=leftTh[1] )
						{
							leftThAtGrasp[1] = leftTh[1] + abs(leftForcesAtGrasp(1))-leftTh[1] + 1.0;
						}
						else if( abs(leftForcesAtGrasp(2))>=leftTh[2] )
						{
							leftThAtGrasp[2] = leftTh[2] + abs(leftForcesAtGrasp(2))-leftTh[2] + 1.0;
						}
						else
						{
							leftThAtGrasp = thresh.segment(3,3);
						}

						if( (abs(leftForcesAtGrasp[idx]) > leftThAtGrasp[idx]) && ( (lEf_area_wAB_gA > lEf_area_wAB_f) || (lEf_area_wAB_gB > lEf_area_wAB_f) ) )
						{
							open_gripperL();
							restartHandover=true;
							readyToGrasp=false;
							dum1=false;
							if(dum3)
							{
								dum3=false;
								LOG_SUCCESS("object returned, threshold on " << axis_name << " with forces " << leftForcesAtGrasp.transpose()<< " reached on left hand with th1 " << leftThAtGrasp.transpose())
							}
						}
						return false;
					};

					/*handover control*/
					auto  gripperLtEf = (markersPos[gripperLtEfA].col(i)+markersPos[gripperLtEfB].col(i))/2;

					if( ( (gripperLtEf-markersPos[fingerSubjLt].col(i) ).norm() <0.2 ) )
					{
						/*open empty gripper when subject come near to robot*/
						if( (!openGripper) && (leftForce.norm()<1.0) )
						{
							open_gripperL();
							LOG_WARNING("opening gripper with left Force Norm "<< leftForce.norm())
							openGripper = true;
						}

						/*close gripper*/
						if( (openGripper) && (!closeGripper) && (!restartHandover) && ( (lEf_area_wAB_gA > lEf_area_wAB_O) || (lEf_area_wAB_gB > lEf_area_wAB_O) ) )
						{
							close_gripperL();
							closeGripper = true;
						}
						
						/*when closed WITH object*/
						if( closeGripper && (leftForce.norm()>=2.0) )
						{
							if(dum2)
							{
								LOG_INFO(" object is inside gripper ")
								dum2 = false;
							}
						}

						/*check if object is being pulled*/
						if(readyToGrasp)
							{
								return checkForce("x-axis", 0) || checkForce("y-axis", 1) || checkForce("z-axis", 2);
							}
						//motion=false;
					}

					/*when closed WITHOUT object*/
					else if( dum1 && closeGripper && (leftForce.norm()<1.0) )
					{
						open_gripperL();
						closeGripper = false;
						LOG_ERROR("object is not inside gripper, try again")
						dum1=false;
					}

					/*restart handover*/
					if( (gripperLtEf-markersPos[fingerSubjLt].col(i)).norm() > 0.50 )
					{
						// motion=true;
						if( (!restartHandover) && (leftForce.norm()>=2.0) )
						{
							readyToGrasp=true;
						} 
						if(restartHandover)
						{
							restartHandover=false;
							openGripper=false;
							closeGripper=false;

							dum1=true;
							dum2=true;
							dum3=true;
							cout<<"/*******restarting handover*******/"<<endl;
						}
					}

					/*iterator*/
					i+= 1;
				}// check for non zero frame

				/*iterator for sim data*/
				s+= 1;
			}// startCapture

			// output("OK");
			return false;
		}// run


		void StartMocapStep::teardown(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			ctl.solver().removeTask(comTask);
			ctl.solver().removeTask(ctl.posTaskL);
			ctl.solver().removeTask(ctl.oriTaskL);

			ctl.gui()->removeCategory({"Handover"});
		}

	} // namespace states

} // namespace mc_handover
