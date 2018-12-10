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


			/*chest task*/
			chestPosTask.reset(new mc_tasks::PositionTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			chestOriTask.reset(new mc_tasks::OrientationTask("CHEST_LINK1", ctl.robots(), 0, 3.0, 1e2));
			ctl.solver().addTask(chestPosTask);
			ctl.solver().addTask(chestOriTask);


			/*EfL pos Task*/
			ctl.posTaskL = std::make_shared<mc_tasks::PositionTask>("LARM_LINK7", ctl.robots(), 0, 3.0, 1e3);
			ctl.solver().addTask(ctl.posTaskL);
			ctl.posTaskL->position({0.3,0.3,1.1});


			/*EfL ori Task*/
			ctl.oriTaskL = std::make_shared<mc_tasks::OrientationTask>("LARM_LINK6",ctl.robots(), 0, 2.0, 1e2);
			ctl.solver().addTask(ctl.oriTaskL);
			q = {0.64, -0.01, -0.76, -0.06};
			// q = {-0.42, -0.5, 0.56, 0.49};
			ctl.oriTaskL->orientation(q.toRotationMatrix().transpose());


			/*change prediction_ settings*/
			ctl.gui()->addElement({"Handover", "tuner"},
				mc_rtc::gui::ArrayInput("t_predict/t_observe", {"t_predict", "t_observe", "zero"},
				[this]() { return tuner; }, 
				[this](const Eigen::Vector3d & to){tuner = to;cout<< "t_predict = " << tuner(0)*1/fps<< "sec, t_observe = "<<tuner(1)*1/fps<< "sec"<<endl;}));

			tuner << 400., 20., 0.; 
			t_predict = (int)tuner(0);
			t_observe = (int)tuner(1);
			newPosSubj = Eigen::MatrixXd::Zero(3, t_observe);


			/*Motion FOR CREATING MOCAP TEMPLATE*/
			ctl.gui()->addElement({"Handover", "MOCAP_template"},
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
					gripper->setTargetQ({0.5}); } )
				
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
						ctl.removeContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"});
						ctl.addContact({"handoverObjects", "ground", "handoverPipeBottom", "AllGround"});
					})
				, mc_rtc::gui::Button("Replay", [this](){ i = 0;}),
				mc_rtc::gui::Point3D("log data", [this,&ctl](){ ctl.robots().robot(2).posW({Subj_X_efL.translation()}); return Subj_X_efL.translation();})
				);


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


			/*configure MOCAP*/
			if(Flag_CORTEX)
			{
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
			for(int m=0; m<maxMarkers; m++)
				{ markersPos[m] = Eigen::MatrixXd::Zero(3,60000);	}
		}// start



		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			/*hand pose*/
			ltHand = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK7")];


			/*set com pose*/
			target = initialCom + move;
			comTask->com(target);


			/*Force sensor*/
			auto leftTh = thresh.head(6);
			// auto baseLeftTh = baseTh.head(6);
			auto leftForce = ctl.wrenches.at("LeftHandForceSensor").force();


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


			/* start only when ithFrame == 1 */
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
						{ Markers[m] = pos[m].col(s); }
				}


				/* check for non zero frame only and store them */ 
				if(	Markers[wristR](0)!= 0 && Markers[wristR](0)< 100
					&&  Markers[object](0)!= 0 && Markers[object](0)< 100
					&&  Markers[knuckleS](0)!= 0 && Markers[knuckleS](0)< 100
					)
				{
					for(int m=0; m<maxMarkers; m++)
						{ markersPos[m].col(i) << Markers[m]; }


					/*direction vectors, projections and area*/
					CD = markersPos[gripperLC].col(i)-markersPos[gripperLD].col(i);
					AB = markersPos[gripperLB].col(i)-markersPos[gripperLA].col(i);
					AC = markersPos[gripperLC].col(i)-markersPos[gripperLA].col(i);
					AD = markersPos[gripperLD].col(i)-markersPos[gripperLA].col(i);
					AK = markersPos[knuckleS].col(i) -markersPos[gripperLA].col(i);
					AO = markersPos[object].col(i)	 -markersPos[gripperLA].col(i);

					auto AB_theta_AC = acos( AB.dot(AC)/( AB.norm()*AC.norm() ) );
					// auto AB_theta_AD = acos( AB.dot(AD)/( AB.norm()*AD.norm() ) );
					auto AC_theta_AO = acos( AC.dot(AO)/( AC.norm()*AO.norm() ) );
					auto AC_theta_AD = acos( AC.dot(AD)/( AC.norm()*AD.norm() ) );
					auto AC_theta_AK = acos( AC.dot(AK)/( AC.norm()*AK.norm() ) );

					auto area_ABC = 0.5*AB.norm()*AC.norm()*sin(AB_theta_AC);
					// auto area_ABD = 0.5*AB.norm()*AD.norm()*sin(AB_theta_AD);
					auto area_ACD = 0.5*AB.norm()*AC.norm()*sin(AC_theta_AD);
					auto area_ACO = 0.5*AC.norm()*AO.norm()*sin(AC_theta_AO);
					auto area_ACK = 0.5*AC.norm()*AK.norm()*sin(AC_theta_AK);

					// PQ = markersPos[wristR].col(i)-markersPos[elbowR].col(i);
					// auto CD_proj_PQ = (CD.dot(PQ)*PQ)/PQ.squaredNorm();
					// auto AB_proj_PQ = (AB.dot(PQ)*PQ)/PQ.squaredNorm();


					/*get robot ef marker(s) current pose*/
					auto efGripperPos =
					( markersPos[gripperLA].col(i) + markersPos[gripperLB].col(i) +
						markersPos[gripperLC].col(i) + markersPos[gripperLD].col(i) )/4;
					curPosLeftEfMarker << efGripperPos;
					sva::PTransformd M_X_efLMarker(curPosLeftEfMarker);
					
					/*get robot ef current pose*/
					curRotLeftEf = ltHand.rotation();
					curPosLeftEf = ltHand.translation();
					sva::PTransformd R_X_efL(curPosLeftEf);

					rotSubj = Eigen::Matrix3d::Identity();
					sva::PTransformd M_X_Subj(rotSubj, markersPos[knuckleS].col(i)); //.middleCols((i-t_observe)+j,i));

					/*subj marker(s) pose w.r.t to robot EF frame*/
					Subj_X_efL = R_X_efL.inv()*M_X_Subj*M_X_efLMarker.inv()*R_X_efL;
					
					if(j<=t_observe)
					{
						newPosSubj.col(j-1) = Subj_X_efL.translation();
						if(j==t_observe)
							{ ithPosSubj = newPosSubj.col(t_observe-1); j=1; }
						j++;
					}

					/*predicted pos*/
					if( (i%t_observe == 0) && prediction )
					{
						/*prediction tuner*/
						t_predict = (int)tuner(0);
						t_observe = (int)tuner(1);

						/*get average velocity of previous *t_observe* sec Subj motion*/
						curVelSubj  = ctl.handoverTraj->diff(newPosSubj)*fps;//ignore diff > XXXX
						avgVelSubj  << ctl.handoverTraj->takeAverage(curVelSubj);

						/*predict position in straight line after t_predict time*/
						predictPos = ctl.handoverTraj->constVelocityPredictPos(avgVelSubj, ithPosSubj, t_predict);

						/*get predicted way points between left ef and Subj*/
						wp_efL_Subj=ctl.handoverTraj->constVelocity(ithPosSubj, predictPos, t_predict);
						wp = get<0>(wp_efL_Subj);

						it = 40+(int)t_predict/t_observe;
						initRefPos << wp(0,it), wp(1,it), wp(2,it);

						collected = true;
					}//t_observe


					/*feed Ef pose*/
					if( collected )
					{
						it+= 40+(int)t_predict/t_observe;
						
						auto curLEfPos = ltHand.translation();

						if(it<=wp.cols())
						{
							refPos << wp(0,it), wp(1,it), wp(2,it);

							handoverPos = curLEfPos + refPos - initRefPos;

							/*robot constraint*/
							if(	(handoverPos(0))<= 0.7 && (handoverPos(1))<= 0.6 && (handoverPos(2))<=1.5 &&
								(handoverPos(0))>= 0.2 && (handoverPos(1))>= 0.25 && (handoverPos(2))>=0.9 ) 
							{
								/*control head*/
								if(handoverPos(1) >.45){ctl.set_joint_pos("HEAD_JOINT0",  0.8);} //y //+ve to move head left
								else{ctl.set_joint_pos("HEAD_JOINT0",  0.); }//+ve to move head left

								if(handoverPos(2) < 1.1){ctl.set_joint_pos("HEAD_JOINT1",  0.4);} //z //+ve to move head down
								else{ctl.set_joint_pos("HEAD_JOINT1",  -0.4);} //+ve to move head down

								/*handover position*/
								ctl.posTaskL->position(handoverPos);
							}
							if(it==wp.cols())
								{ collected  = false; }
						}
					}// collected


					/*gripper control*/
					auto close_gripperL = [&]()
					{
						closeGripper = true;
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({0.0});
					};

					auto open_gripperL = [&]()
					{
						openGripper = false;
						auto gripper = ctl.grippers["l_gripper"].get();
						gripper->setTargetQ({0.5});
					};


					/*force control*/
					auto checkForce = [&](const char * axis_name, int idx)
					{
						/**** dont use fabs & check also force direction ****/
						if( (fabs(leftForce[idx]) > leftTh[idx+3]) && ( (area_ABC > area_ACK) || (area_ACD > area_ACK) ) )
						{
							openGripper = true;
							open_gripperL();
							closeGripper = false;
							ctl.publishWrench();
							LOG_INFO("Opening grippers, threshold on " << axis_name << " reached on left hand")
							return true;
						}
					};


					/*grasp object (close gripper)*/
					auto compObjRelPos = [&]()
					{
						if( (closeGripper==false) && ( (area_ABC > area_ACO) || (area_ACD > area_ACO) ) )
						{
							close_gripperL();
							cout << "object is inside gripper, closing gripper" <<endl;
						}
						return checkForce("x-axis", 0) || checkForce("y-axis", 1) || checkForce("z-axis", 2);
					};

					
					/*handover control*/
					if( ( markersPos[wristR].col(i)-markersPos[knuckleS].col(i) ).norm() <0.2 )
					{
						if(openGripper)
							{ open_gripperL(); }
						prediction = false;
						compObjRelPos();
					}
					else
						{prediction = true;}

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

/*------------------------------TO DOs----------------------------------------------
--- fix threshold
--- stop prediction during handover

--- add different object & orientation

--- record/display pos trail in rviz***
----------------------------------------------------------------------------------*/