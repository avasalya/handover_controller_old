#include "StartMocapStep.h"

namespace mc_handover
{

	namespace states
	{

		void StartMocapStep::configure(const mc_rtc::Configuration & config)
		{
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

			ltRotW << -0.138901, -0.0699416, 0.987833,
			0.084922, 0.992987, 0.0822475,
			-0.986658, 0.095313, -0.131987;

			// initial orientation
			ql = {0.95, 0.086, -0.25, -0.15};
			qr = {0.95, 0.086, -0.25, -0.15};
			
			//for mocap_temp
			q1l = {0.64, -0.01, -0.76, -0.06};
			q1r = {0.64, -0.01, -0.76, -0.06};
			

			p_l<< 0.3,0.3,1.1;
			p_r<<0.3, -0.3,1.1;


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



			/*EfR pos Task*/
			ctl.posTaskR = make_shared<mc_tasks::PositionTask>("RARM_LINK7", ctl.robots(), 0, 3.0, 1e3);
			ctl.solver().addTask(ctl.posTaskR);

			/*EfL ori Task*/
			ctl.oriTaskR = make_shared<mc_tasks::OrientationTask>("RARM_LINK6",ctl.robots(), 0, 2.0, 1e2);
			ctl.solver().addTask(ctl.oriTaskR);



			/*Handover buttons*/
			// ctl.gui()->addElement({"Handover", "Weights"},
			// 	mc_rtc::gui::NumberInput("lHand Weight", [this]() { return lHandMass; }, [this](double w1){ lHandMass = w1; }),
			// 	mc_rtc::gui::NumberInput("object Weight",[this]() { return objMass; }, [this](double w2){ objMass = w2; }) );

			/*Motion FOR CREATING MOCAP TEMPLATE*/
			ctl.gui()->addElement({"Handover", "randomPos"},
				mc_rtc::gui::Button("open_gripperL & set flags", [this, &ctl]()
				{
					auto gripper = ctl.grippers["l_gripper"].get();
					gripper->setTargetQ({openGrippers});
					closeGripperLt = false; motionLt=true;
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
					ctl.posTaskL->position(p_l);

					ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
					ctl.posTaskR->position(p_r);
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
					cout << "right hand Forces " <<
					ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force().transpose()<<endl;
				}),
				
				mc_rtc::gui::Button("Norm_Ef_Forces",[this, &ctl](){
					Eigen::Vector3d vl = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force(); cout<<"Norm Lt "<< vl.norm() <<endl;
					Eigen::Vector3d vr = ctl.robot().forceSensor("RightHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force(); cout<<"Norm Rt "<< vr.norm() <<endl;
				}),

				mc_rtc::gui::ArrayInput("set Threshold %",
					{"Left cx", "cy", "cz", "fx", "fy", "fz", "Right cx", "cy", "cz", "fx", "fy", "fz"},
					[this]() { return thresh; },
					[this](const Eigen::VectorXd & t)
					{
						LOG_INFO("Changed threshold to:\nLeft: " << t.head(6).transpose() << "\nRight: " << t.tail(6).transpose() << "\n")
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


			/*change prediction_ settings*/
			ctl.gui()->addElement({"Handover", "tuner"},
				mc_rtc::gui::ArrayInput("t_predict/t_observe", {"t_predict", "t_observe", "it"},
					[this]() { return tuner; }, 
					[this](const Eigen::Vector3d & to){tuner = to;cout<< "t_predict = " << tuner(0)*1/fps<< "sec, t_observe = "<<tuner(1)*1/fps<< "sec"<<endl;}));

			tuner << 400., 20., 60.;
			t_predict = (int)tuner(0);
			t_observe = (int)tuner(1);
			it = (int)tuner(2);


			// tunerRt << 400., 20., 60.;
			// t_predictRt = (int)tunerRt(0);
			// t_observeRt = (int)tunerRt(1);
			// it_rt = (int)tunerRt(2);


			/*com Task*/
			ctl.gui()->addElement({"Handover", "com"},

				mc_rtc::gui::ArrayInput("Move Com Pos", {"x", "y", "z"},
					[this]() { return move; },
					[this](const Eigen::Vector3d v) { move = v;
						cout << " com pos set to:\n" << initialCom + move << endl;})
				);

			comTask = make_shared<mc_tasks::CoMTask>
			(ctl.robots(), ctl.robots().robotIndex(), 10., 1e5);
			// comTask->dimWeight(Eigen::Vector3d(1., 1., 1.));

			initialCom = rbd::computeCoM(ctl.robot().mb(),ctl.robot().mbc());
			comTask->com(initialCom);
			ctl.solver().addTask(comTask);

			
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
				mc_rtc::gui::Point3D("subj fing pos", [this,&ctl](){ ctl.robots().robot(2).posW({X_efL_Subj.translation()}); return X_efL_Subj.translation();})
				);

			/*trajectory trail*/
			ctl.gui()->addElement({"Handover", "Trajectories"},

				// mc_rtc::gui::Trajectory("obseve subj pos", {{0, 1, 0}, 0.01, mc_rtc::gui::LineStyle::Solid},
				// 	[this, &ctl]() -> const vector<sva::PTransformd>& { return S_X_efL; }),

				mc_rtc::gui::Trajectory("subj knuckle marker pos", {{1,1,0}, 0.01, mc_rtc::gui::LineStyle::Dotted},
					[this, &ctl]() -> const vector<Eigen::Vector3d>& { return predictedPositionsLt; } ),

				mc_rtc::gui::Trajectory("traj_l_wrist", {{1,0,1}, 0.01, mc_rtc::gui::LineStyle::Dotted},
					[this,&ctl](){ return ctl.robot().bodyPosW("LARM_LINK7").translation(); }),

				mc_rtc::gui::Trajectory("traj_r_wrist", {{0,0,1}, 0.01, mc_rtc::gui::LineStyle::Dotted},
					[this,&ctl](){ return ctl.robot().bodyPosW("RARM_LINK7").translation(); })
				);


			/*configure MOCAP*/
			if(Flag_CORTEX)
			{
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
			}
			else /*simulation*/
			{
				LOG_ERROR("***MOCAP IS DISABLED***")
				startCapture = false; //true for sim
				
				name = {"simData3"};
				string fn = string(DATA_PATH) + "/" + name + ".txt";
				ifstream file(fn);

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
				ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());
				ctl.posTaskL->position({0.3,0.25,1.1});
			}

			/*allocate memory for mocap markers*/
			newPosSubjLt = Eigen::MatrixXd::Zero(3, t_observe);
			newPosSubjRt = Eigen::MatrixXd::Zero(3, t_observe);
			
			Markers.resize(maxMarkers);
			markersPos.resize(maxMarkers);

			efLPos.resize(3);
			efLVel.resize(2);

			predictedPositionsLt.resize(1);
			predictedPositionsRt.resize(1);

			X_S_efL.resize(t_observe);

			for(int m=0; m<maxMarkers; m++)
				{ markersPos[m] = Eigen::MatrixXd::Zero(3,60000); }


			/*initial force threshold*/
			leftTh = thresh.segment(3,3);
		}// start


		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
			
			/*hand pose*/
			ltHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")];
			ltRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")].rotation();

			rtHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK7")];
			rtRotW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")].rotation();


			/*set com pose*/
			target = initialCom + move;
			comTask->com(target);


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
			/*on simulation*/
			else
			{
				// cout<<" rotX matrix \n" << sva::RotX(90*pi/180)<<endl;
				// ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose()*sva::RotX(150*pi/180));

				// subjLtHandRot = ctl.oriTaskL->orientation();
				// idt <<	1.0, 0.0, 0.0,
				// 0.0, 1.0, 0.0,
				// 0.0, 0.0, 1.0;
				
				// ctl.oriTaskL->orientation(idt); 
				
				// Vector3d err= sva::rotationError(subjLtHandRot, idt);
				
				// LOG_SUCCESS("XYZ degrees "<< (RadToDeg)*err.transpose())
				
				// LOG_ERROR(" x-angle " << (RadToDeg)*atan2(subjLtHandRot(2,1), subjLtHandRot(2,2)) <<
				// 	" y-angle " << (RadToDeg)*atan2(-subjLtHandRot(2,0), sqrt( pow( subjLtHandRot(2,1),2) + pow(subjLtHandRot(2,2),2) ) ) <<
				// 	" z-angle " << (RadToDeg)*atan2(subjLtHandRot(1,0), subjLtHandRot(0,0)) )
			}


			// if(i%200)
			// { LOG_ERROR( roundf( (ctl.posTaskL->speed().norm()) * 100 )/100 ); }


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
				checkNonZeroLt =
				Markers[wristLtEfA](0)!=0 && Markers[wristLtEfA](0)< 20 &&
				Markers[wristLtEfB](0)!=0 && Markers[wristLtEfB](0)< 20 &&

				Markers[gripperLtEfA](0)!=0 && Markers[gripperLtEfA](0)< 20 &&
				Markers[gripperLtEfB](0)!=0 && Markers[gripperLtEfB](0)< 20 &&

				Markers[fingerSubjLt](0)!=0 && Markers[fingerSubjLt](0)< 20;


				checkNonZeroRt =
				Markers[wristRtEfA](0)!=0 && Markers[wristRtEfA](0)< 20 &&
				Markers[wristRtEfB](0)!=0 && Markers[wristRtEfB](0)< 20 &&

				Markers[gripperRtEfA](0)!=0 && Markers[gripperRtEfA](0)< 20 &&
				Markers[gripperRtEfB](0)!=0 && Markers[gripperRtEfB](0)< 20 &&

				Markers[fingerSubjRt](0)!=0 && Markers[fingerSubjRt](0)< 20;


				if( checkNonZeroLt && checkNonZeroRt && (Markers[object](0)!=0) && (Markers[object](0)<20) )
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

					auto lEf_wAB_theta_wAgA = acos( lEf_wA_wB.dot(lEf_wA_gA)/( lEf_wA_wB.norm()*lEf_wA_gA.norm() ) );
					auto lEf_wAB_theta_wAgB = acos( lEf_wA_wB.dot(lEf_wA_gB)/( lEf_wA_wB.norm()*lEf_wA_gB.norm() ) );
					auto lEf_wAB_theta_wAO = acos( lEf_wA_wB.dot(lEf_wA_O)/( lEf_wA_wB.norm()*lEf_wA_O.norm() ) );
					auto lEf_wAB_theta_wAf = acos( lEf_wA_wB.dot(lEf_wA_lf)/( lEf_wA_wB.norm()*lEf_wA_lf.norm() ) );
					
					auto lEf_area_wAB_gA = 0.5*lEf_wA_wB.norm()*lEf_wA_gA.norm()*sin(lEf_wAB_theta_wAgA);
					auto lEf_area_wAB_gB = 0.5*lEf_wA_wB.norm()*lEf_wA_gB.norm()*sin(lEf_wAB_theta_wAgB);
					auto lEf_area_wAB_O  = 0.5*lEf_wA_wB.norm()*lEf_wA_O.norm()*sin(lEf_wAB_theta_wAO);
					auto lEf_area_wAB_f  = 0.5*lEf_wA_wB.norm()*lEf_wA_lf.norm()*sin(lEf_wAB_theta_wAf);


					auto rEf_wA_wB = markersPos[wristRtEfA].col(i)-markersPos[wristRtEfB].col(i);
					auto rEf_wA_gA = markersPos[wristRtEfA].col(i)-markersPos[gripperRtEfA].col(i);
					auto rEf_wA_gB = markersPos[wristRtEfA].col(i)-markersPos[gripperRtEfB].col(i);
					auto rEf_wA_O  = markersPos[wristRtEfA].col(i)-markersPos[object].col(i);
					auto rEf_wA_lf  = markersPos[wristRtEfA].col(i)-markersPos[fingerSubjRt].col(i);

					auto rEf_wAB_theta_wAgA = acos( rEf_wA_wB.dot(rEf_wA_gA)/( rEf_wA_wB.norm()*rEf_wA_gA.norm() ) );
					auto rEf_wAB_theta_wAgB = acos( rEf_wA_wB.dot(rEf_wA_gB)/( rEf_wA_wB.norm()*rEf_wA_gB.norm() ) );
					auto rEf_wAB_theta_wAO = acos( rEf_wA_wB.dot(rEf_wA_O)/( rEf_wA_wB.norm()*rEf_wA_O.norm() ) );
					auto rEf_wAB_theta_wAf = acos( rEf_wA_wB.dot(rEf_wA_lf)/( rEf_wA_wB.norm()*rEf_wA_lf.norm() ) );
					
					auto rEf_area_wAB_gA = 0.5*rEf_wA_wB.norm()*rEf_wA_gA.norm()*sin(rEf_wAB_theta_wAgA);
					auto rEf_area_wAB_gB = 0.5*rEf_wA_wB.norm()*rEf_wA_gB.norm()*sin(rEf_wAB_theta_wAgB);
					auto rEf_area_wAB_O  = 0.5*rEf_wA_wB.norm()*rEf_wA_O.norm()*sin(rEf_wAB_theta_wAO);
					auto rEf_area_wAB_f  = 0.5*rEf_wA_wB.norm()*rEf_wA_lf.norm()*sin(rEf_wAB_theta_wAf);


					/*move EF when subject approaches object 1st time*/
					if( leftHandReady && (markersPos[fingerSubjLt].col(i)-markersPos[object].col(i)).norm()<0.5 )
					{
						ctl.oriTaskL->orientation(q1l.toRotationMatrix().transpose());
						ctl.posTaskL->position(p_l);
					}
					else if( rightHandReady && (markersPos[fingerSubjRt].col(i)-markersPos[object].col(i)).norm()<0.5 )
					{
						ctl.oriTaskR->orientation(q1r.toRotationMatrix().transpose());
						ctl.posTaskR->position(p_r);
					}


					if( leftHandReady
						&& ( roundf( (ctl.posTaskL->speed().norm()) * 100 )/100 <= 0.01 ) 
						&& ( roundf( (ctl.oriTaskL->speed().norm()) * 100 )/100 <= 0.01 ) )
						{
							leftHandReady=false;
						}

					if( rightHandReady
						&& ( roundf( (ctl.posTaskR->speed().norm()) * 100 )/100 <= 0.01 ) 
						&& ( roundf( (ctl.oriTaskR->speed().norm()) * 100 )/100 <= 0.01 ) )
						{
							rightHandReady=false;
						}


					/*observe subject motion for t_observe period*/
					if( (i%t_observe==0) )
					{
						/*prediction_ tuner*/
						t_predict = (int)tuner(0);
						t_observe = (int)tuner(1);
						it = (int)tuner(2);


						/*get robot efl current pose*/
						curRotLeftEf = ltHand.rotation();
						curPosLeftEf = ltHand.translation();

						/*get robot efl marker(s) current pose*/
						auto efLGripperPos = 0.25*( 
							markersPos[wristLtEfA].col((i-t_observe)+1) + markersPos[wristLtEfB].col((i-t_observe)+1) +
							markersPos[gripperLtEfA].col((i-t_observe)+1) + markersPos[gripperLtEfB].col((i-t_observe)+1) );
						curPosLeftEfMarker << efLGripperPos;

						X_R_efL = sva::PTransformd(ltRotW, curPosLeftEf);
						X_M_efLMarker = sva::PTransformd(ltRotW, curPosLeftEfMarker);
						X_R_M_lt = X_M_efLMarker.inv() * X_R_efL;
						X_R_efL_const = sva::PTransformd(q1l.toRotationMatrix(), p_l);


						/*get robot efr current pose*/
						curRotRightEf = rtHand.rotation();
						curPosRightEf = rtHand.translation();

						/*get robot efr marker(s) current pose*/
						auto efRGripperPos = 0.25*( 
							markersPos[wristRtEfA].col((i-t_observe)+1) + markersPos[wristRtEfB].col((i-t_observe)+1) +
							markersPos[gripperRtEfA].col((i-t_observe)+1) + markersPos[gripperRtEfB].col((i-t_observe)+1) );
						curPosRightEfMarker << efRGripperPos;

						X_R_efR = sva::PTransformd(rtRotW, curPosRightEf);
						X_M_efRMarker = sva::PTransformd(rtRotW, curPosRightEfMarker);
						X_R_M_rt = X_M_efRMarker.inv() * X_R_efR;
						X_R_efR_const = sva::PTransformd(q1r.toRotationMatrix(), p_r);


						/*check left subj hand's relative orientation*/
						if(!leftHandReady
							&&	Markers[lShapeLtA](0)!=0 && Markers[lShapeLtA](0)< 20
							&& 	Markers[lShapeLtB](0)!=0 && Markers[lShapeLtB](0)< 20
							&& 	Markers[lShapeLtC](0)!=0 && Markers[lShapeLtC](0)< 20
							&& 	Markers[lShapeLtD](0)!=0 && Markers[lShapeLtD](0)< 20
							)
						{
							// sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];
							// BodyW.rotation();

							curPosLtLshp = markersPos[lShapeLtC].col(i);

							/*get unit vectors XYZ of subject LEFT hand*/
							x = markersPos[lShapeLtA].col(i) - markersPos[lShapeLtC].col(i);//vCA=X
							y = markersPos[lShapeLtD].col(i) - markersPos[lShapeLtC].col(i);//vCD=Y

							lshpLt_X = x/x.norm();
							lshpLt_Y = y/y.norm();
							lshpLt_Z = lshpLt_X.cross(lshpLt_Y);//X.cross(Y)=Z

							subjLtHandRot.col(0) = lshpLt_X;
							subjLtHandRot.col(1) = lshpLt_Y;
							
							/*convert to 2D rotation method*/
							subjLtHandRot.row(2) = lshpLt_Z/lshpLt_Z.norm();
							// LOG_ERROR(subjLtHandRot<<"\n\n")

							X_M_Ltlshp = sva::PTransformd(subjLtHandRot, curPosLtLshp);
							
							X_e_l_lt =  X_R_efL_const.inv() * X_M_Ltlshp;// * X_R_M_lt;
							// LOG_SUCCESS(X_e_l_lt.rotation()<<"\n")

							handoverLtRot = X_e_l_lt.rotation();// * idt;
						}


						/*check right subj hand's relative orientation*/
						if(!rightHandReady
							&&	Markers[lShapeRtA](0)!=0 && Markers[lShapeRtA](0)< 20
							&& 	Markers[lShapeRtB](0)!=0 && Markers[lShapeRtB](0)< 20
							&& 	Markers[lShapeRtC](0)!=0 && Markers[lShapeRtC](0)< 20
							&& 	Markers[lShapeRtD](0)!=0 && Markers[lShapeRtD](0)< 20
							)
						{
							// sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];
							// BodyW.rotation();

							curPosRtLshp = markersPos[lShapeRtC].col(i);

							/*get unit vectors XYZ of subject right hand*/
							x_ = markersPos[lShapeRtA].col(i) - markersPos[lShapeRtC].col(i);//vCA=X
							y_ = markersPos[lShapeRtD].col(i) - markersPos[lShapeRtC].col(i);//vCD=Y

							lshpRt_X = x_/x_.norm();
							lshpRt_Y = y_/y_.norm();
							lshpRt_Z = lshpRt_X.cross(lshpRt_Y);//X.cross(Y)=Z

							subjRtHandRot.col(0) = lshpRt_X;
							subjRtHandRot.col(1) = lshpRt_Y;
							
							/*convert to 2D rotation method*/
							subjRtHandRot.row(2) = lshpRt_Z/lshpRt_Z.norm();
							// LOG_ERROR(subjRtHandRot<<"\n\n")

							X_M_Rtlshp = sva::PTransformd(subjRtHandRot, curPosRtLshp);
							
							X_e_l_rt =  X_R_efL_const.inv() * X_M_Rtlshp;// * X_R_M_rt;
							// LOG_SUCCESS(X_e_l_rt.rotation()<<"\n")

							handoverRtRot = X_e_l_rt.rotation();// * idt;
						}



						/*subj marker(s) pose w.r.t to robot EF(s) frame*/
						for(int j=1;j<=t_observe; j++)
						{
							// X_M_SubjL = sva::PTransformd(idtMat, markersPos[fingerSubjLt].middleCols((i-t_observe)+j,i));
							X_M_SubjL = sva::PTransformd(handoverLtRot, markersPos[fingerSubjLt].middleCols((i-t_observe)+j,i));

							X_efL_Subj = X_R_efL.inv() * X_M_SubjL * X_M_efLMarker.inv() * X_R_efL;

							X_S_efL[j-1] = X_efL_Subj;

							newPosSubjLt.col(j-1) = X_efL_Subj.translation();


							// X_M_SubjR = sva::PTransformd(idtMat, markersPos[fingerSubjRt].middleCols((i-t_observe)+j,i));
							X_M_SubjR = sva::PTransformd(handoverRtRot, markersPos[fingerSubjRt].middleCols((i-t_observe)+j,i));

							X_efR_Subj = X_R_efR.inv() * X_M_SubjR * X_M_efRMarker.inv() * X_R_efR;

							X_S_efR[j-1] = X_efR_Subj;

							newPosSubjRt.col(j-1) = X_efR_Subj.translation();
							

							if(j==t_observe)
							{
								ithPosSubjLt = newPosSubjLt.col(t_observe-1);
								ithPosSubjRt = newPosSubjRt.col(t_observe-1);
							}
						}



						/*get average velocity of previous *t_observe* sec Subj motionLt*/
						curVelSubjLt = ctl.handoverTraj->diff(newPosSubjLt)*fps;//ignore diff > XXXX
						avgVelSubjLt << ctl.handoverTraj->takeAverage(curVelSubjLt);

						/*predict position in straight line after t_predict time*/
						predictPosLt = ctl.handoverTraj->constVelocityPredictPos(avgVelSubjLt, ithPosSubjLt, t_predict);
						predictedPositionsLt[0] << predictPosLt;
						// cout << "predicted pos " << predictPos.transpose()<<endl;

						/*get predicted way points between left ef and Subj*/	/*** GET NEW VELOCITY PROFILE ****/
						wp_efL_Subj = ctl.handoverTraj->constVelocity(ithPosSubjLt, predictPosLt, t_predict);
						wpLt = get<0>(wp_efL_Subj);

						initRefPosLt << wpLt(0,it), wpLt(1,it), wpLt(2,it);
						// initRefPos << ltHand.translation();



						/*get average velocity of previous *t_observe* sec Subj motionRt*/
						curVelSubjRt = ctl.handoverTraj->diff(newPosSubjRt)*fps;//ignore diff > XXXX
						avgVelSubjRt << ctl.handoverTraj->takeAverage(curVelSubjRt);

						/*predict position in straight line after t_predict time*/
						predictPosRt = ctl.handoverTraj->constVelocityPredictPos(avgVelSubjRt, ithPosSubjRt, t_predict);
						predictedPositionsRt[0] << predictPosRt;
						// cout << "predicted pos " << predictPos.transpose()<<endl;

						/*get predicted way points between left ef and Subj*/	/*** GET NEW VELOCITY PROFILE ****/
						wp_efL_Subj = ctl.handoverTraj->constVelocity(ithPosSubjRt, predictPosRt, t_predict);
						wpRt = get<0>(wp_efL_Subj);

						initRefPosRt << wpRt(0,it), wpRt(1,it), wpRt(2,it);
						// initRefPos << ltHand.translation();



						collected = true;
					}//t_observe



					/*feed Ef pose*/
					if( collected )
					{
						it+= (int)tuner(2);//40+(int)t_predict/t_observe;

						auto curLEfPos = ltHand.translation();
						auto curREfPos = rtHand.translation();

						if(it<=wpLt.cols())
						{
							refPosLt << wpLt(0,it), wpLt(1,it), wpLt(2,it);
							handoverPosLt = curLEfPos + refPosLt - initRefPosLt;
							
							// idt << RotX(90*DegToRad) * RotY(90*DegToRad) * RotZ(90*DegToRad);
							// handoverLtRot = idt.transpose()*subjLtHandRot.transpose();

							 /*robot constraint*/
							if((handoverPosLt(0)>= 0.20) && (handoverPosLt(0)<= 0.7) && 
								(handoverPosLt(1)>= 0.05) && (handoverPosLt(1)<= 0.7) &&
								(handoverPosLt(2)>= 0.90) && (handoverPosLt(2)<= 1.5))
							{
								if(motionLt)
								{
									/*control head*/
									if(handoverPosLt(1) >.45){ctl.set_joint_pos("HEAD_JOINT0",  0.8);} //y //+ve to move head left
									else{ctl.set_joint_pos("HEAD_JOINT0",  0.); }//-ve to move head right

									if(handoverPosLt(2) <1.1){ctl.set_joint_pos("HEAD_JOINT1",  0.6);} //z //+ve to move head down
									else{ctl.set_joint_pos("HEAD_JOINT1",  -0.4);} //-ve to move head up

									/*handover pose*/
									sva::PTransformd new_poseLt(handoverLtRot, handoverPosLt);
									ctl.oriTaskL->orientation(new_poseLt.rotation());
									ctl.posTaskL->position(new_poseLt.translation());

									// ctl.oriTaskR->orientation(new_poseLt.rotation());
									// ctl.posTaskR->position(new_poseLt.translation());
								}
							}

							if(it==wpLt.cols())
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

					auto close_gripperR = [&]()
					{
						auto gripper = ctl.grippers["r_gripper"].get();
						gripper->setTargetQ({closeGrippers});
					};
					auto open_gripperR = [&]()
					{
						auto gripper = ctl.grippers["r_gripper"].get();
						gripper->setTargetQ({openGrippers});
					};


					/*Force sensor*/
					leftForce = ctl.robot().forceSensor("LeftHandForceSensor").worldWrenchWithoutGravity(ctl.robot()).force();
					// LOG_ERROR("leftWrenchWithoutGravity " <<leftForce.transpose() << " norm "<< leftForce.norm() )
					
					/*force control*/
					auto checkForce = [&](const char *axis_name, int idx)
					{
						/*get acceleration of lHand*/
						if(i>3)
						{
							for(int g=1; g<=3; g++)
							{
								efLPos[3-g] = 0.25*(
									markersPos[wristLtEfA].col(i-g) + markersPos[wristLtEfB].col(i-g) +
									markersPos[gripperLtEfA].col(i-g) + markersPos[gripperLtEfB].col(i-g)
									);
							}
							efLVel[0] = (efLPos[1]-efLPos[0])*fps;
							efLVel[1] = (efLPos[2]-efLPos[1])*fps;
							efLAce = (efLVel[1]-efLVel[0])*fps;

							efLMass = lFload.norm()/9.8; //lHandMass + objMass;
							lFinert = efLMass*efLAce; //inertial Force at efL

							lFpull[0] = abs(leftForce[0])-abs(lFinert[0]); //-abs(lFzero[0]);
							lFpull[1] = abs(leftForce[1])-abs(lFinert[1]); //-abs(lFzero[1]);
							lFpull[2] = abs(leftForce[2])-abs(lFinert[2]); //-abs(lFzero[2]);
						}

						/*new threshold*/
						auto newLeftTh = lFload + leftTh;



						/* MAY BE check torque too ???? */
						if( (abs(lFpull[idx]) > newLeftTh[idx]) && ( (lEf_area_wAB_gA > lEf_area_wAB_f) || (lEf_area_wAB_gB > lEf_area_wAB_f) ) )
						{
							open_gripperL();
							restartHandover=true;
							readyToGrasp=false;
							dum1=false;
							if(dum3)
							{
								dum3=false;
								cout << "leftForces at Grasp "<< leftForce.transpose() <<endl;
								cout << "lFinert              "<< lFinert.transpose() << " object mass " << efLMass <<endl;
								LOG_SUCCESS("object returned, threshold on " << axis_name << " with pull forces " << lFpull.transpose()<< " reached on left hand with th1 " << newLeftTh.transpose())
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
							lFzero = leftForce; //this has Fintertia too
							open_gripperL();
							LOG_WARNING("opening gripper with left Force Norm "<< leftForce.norm())
							openGripper = true;
						}

						/*close gripper*/
						if( (openGripper) && (!closeGripperLt) && (!restartHandover) && ( (lEf_area_wAB_gA > lEf_area_wAB_O) || (lEf_area_wAB_gB > lEf_area_wAB_O) ) )
						{
							close_gripperL();
							motionLt=false; //when subject hand is very close to efL
							closeGripperLt = true;
						}
						
						/*when closed WITH object*/
						if( dum2 && closeGripperLt && (leftForce.norm()>=2.0) )
						{
							lFNormAtClose = leftForce.norm();
							LOG_INFO(" object is inside gripper "<< leftForce.norm() )
							dum2 = false;
						}

						/*check if object is being pulled*/
						if(readyToGrasp)
						{
							return checkForce("x-axis", 0) || checkForce("y-axis", 1) || checkForce("z-axis", 2);
						}
					}

					/*restart handover*/
					if( (gripperLtEf-markersPos[fingerSubjLt].col(i)).norm() > 0.50 )
					{
						/*here comes only after object is grasped*/
						if( (closeGripperLt) && (!restartHandover) && (lFNormAtClose>=2.0) )
						{
							if(e%200==0)//wait xx sec
							{
								motionLt=true;
								readyToGrasp=true;

								lFload <<
								accumulate( lFloadx.begin(), lFloadx.end(), 0.0)/double(lFloadx.size()),
								accumulate( lFloady.begin(), lFloady.end(), 0.0)/double(lFloady.size()),
								accumulate( lFloadz.begin(), lFloadz.end(), 0.0)/double(lFloadz.size());

								// LOG_INFO("ready to grasp again, avg itr size "<< lFloadx.size())

								/*clear vector memories*/
								lFloadx.clear(); lFloady.clear(); lFloadz.clear();
							}
							/*divide by 9.8 and you will get object mass*/
							else
							{
								lFloadx.push_back( abs( abs(leftForce[0])-abs(lFzero[0]) ) );
								lFloady.push_back( abs( abs(leftForce[1])-abs(lFzero[1]) ) );
								lFloadz.push_back( abs( abs(leftForce[2])-abs(lFzero[2]) ) );
							}
							e+=1;//cout << "e " << e << endl;
						}

						if(restartHandover)
						{
							restartHandover=false;
							openGripper=false;
							closeGripperLt=false;

							dum1=true;
							dum2=true;
							dum3=true;
							cout<<"/*******restarting handover*******/"<<endl;
						}
					}
					/*iterator*/
					i+= 1;
				}// checkNonZeroLt && checkNonZeroRt


			}// startCapture

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


// 	Eigen::Matrix3d from_rot = X_efL_Subj.rotation(); //ctl.oriTaskL->orientation(); //q1l.toRotationMatrix().transpose();
// 	Eigen::Matrix3d to_rot = subjLtHandRot.transpose();
// 	Eigen::Vector3d er_the = (RadToDeg)*sva::rotationError(from_rot,to_rot);
// LOG_ERROR(er_the.transpose())

// Eigen::Matrix3d handoverLtRot = sva::RotX(er_the(0)) * sva::RotY(er_the(1)) * sva::RotZ(er_the(2));
// sva::PTransformd new_pose(handoverLtRot,handoverPos);
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