
#include "StartMocapStep.h"

namespace mc_handover
{

	namespace states 
	{

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

			/*com Task*/
			auto initialCom = rbd::computeCoM(ctl.robot().mb(),ctl.robot().mbc());
			auto comTask = std::make_shared<mc_tasks::CoMTask>
			(ctl.robots(), ctl.robots().robotIndex(), 10., 1000.);//10, 1e3
			comTask->com(initialCom);
			ctl.solver().addTask(comTask);


			/*position Task*/
			ctl.posTask = std::make_shared<mc_tasks::PositionTask>("LARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), 5.0, 1000);
			


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
						sBodyDef* pBody = &pBodyDefs->BodyDefs[iBody];
						cout << "number of markers defined in body " << iBody+1 << " (\"" << pBody->szName << "\") : " << bodyMarkers.at(iBody) << endl;    

						for (int iMarker=0 ; iMarker<pBody->nMarkers; iMarker++)
						{
							cout << iMarker+1 << " " << pBody->szMarkerNames[iMarker] << endl;
						}
					}
				}
				printf("\n*** start live mode ***\n");
				Cortex_Request("LiveMode", &pResponse, &nBytes);
			}
			else
			{
				startCapture = true;

				/*simData*/
				name = {"simData_1"};

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
					pos(0, i/3) = pts[i];
					pos(1, i/3) = pts[i+1];
					pos(2, i/3) = pts[i+2];
				}
				ctl.gui()->addElement({"Fake mocap"},
					// mc_rtc::gui::Button("Replay", [this](){ i = 0;}),
					mc_rtc::gui::Point3D("log data", [this,&ctl](){ ctl.robots().robot(2).posW({objectBodyMarker}); return objectBodyMarker; }));
			}
		}// start




		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
			auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

			
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
				/* get object marker pose */
				if(Flag_CORTEX)
				{
					// robotBodyMarker <<	
					// FrameofData.BodyData[robotBody].Markers[robotMarkerNo][0], // X left marker
					// FrameofData.BodyData[robotBody].Markers[robotMarkerNo][1], // Y
					// FrameofData.BodyData[robotBody].Markers[robotMarkerNo][2]; // Z

					// objectBodyMarker <<	
					// FrameofData.BodyData[objBody].Markers[objMarkerNo][0], // X farmost
					// FrameofData.BodyData[objBody].Markers[objMarkerNo][1], // Y
					// FrameofData.BodyData[objBody].Markers[objMarkerNo][2]; // Z


					objectBodyMarker <<	
					FrameofData.BodyData[body].Markers[markerO][0], // X
					FrameofData.BodyData[body].Markers[markerO][1], // Y
					FrameofData.BodyData[body].Markers[markerO][2]; // Z

					robotBodyMarker <<	
					FrameofData.BodyData[body].Markers[markerR][0], // X
					FrameofData.BodyData[body].Markers[markerR][1], // Y
					FrameofData.BodyData[body].Markers[markerR][2]; // Z
				}
				else
				{	//0.6115   -0.3264    0.4615; // only 1st itr
					robotBodyMarker << ctl.efTaskL->get_ef_pose().translation(); //replace with LARM_LINK6 pos
					objectBodyMarker << pos.col(i);
					
					// cout << "i & pos.size() " << i << " " << pos.size() << endl;
					if(i==pos.size()/3)
					{
						LOG_WARNING("iter over, restarting again")
						// i =0;	
					}
				}



				/* check for non zero frame only and store them */ 
				if( (robotBodyMarker(0) != 0 && objectBodyMarker(0) != 0 ) 
					&& (robotBodyMarker(0) < 100 && objectBodyMarker(0) < 100) )
				{
					posLeftEfMarker(0, i) = robotBodyMarker(0); // X
					posLeftEfMarker(1, i) = robotBodyMarker(1); // Y
					posLeftEfMarker(2, i) = robotBodyMarker(2); // Z

					posObjMarkerA(0, i) = objectBodyMarker(0); // X
					posObjMarkerA(1, i) = objectBodyMarker(1); // Y
					posObjMarkerA(2, i) = objectBodyMarker(2); // Z
					


					// if(!Flag_CORTEX)
					// {
					// 	if( (posObjMarkerA.col(i)- posLeftEfMarker.col(i)).norm() <0.5 )
					// 	{
					// 		cout <<"norm " << (posObjMarkerA.col(i)- posLeftEfMarker.col(i)).norm() << endl;
					// 		startPrediction = true;
					// 	}
					// 	// else
					// 	// {
					// 	// 	startPrediction = false;
					// 	// }
					// }
					// else
					// {
					// 	startPrediction = true;
					// }

					
					if( (i%tune1 == 0) && (startPrediction) )
					{
						/*get robot ef marker current pose*/
						curPosLeftEfMarker << posLeftEfMarker.col((i-tune1)+1);//1.162, -0.268, 1.074;
						sva::PTransformd M_X_efLMarker(curRotLeftEfMarker, curPosLeftEfMarker);
						// cout << "curPosLeftEfMarker " << curPosLeftEfMarker.transpose() << endl;


						/*get robot ef current pose*/
						// curRotLeftEf = ctl.relEfTaskL->get_ef_pose().rotation();
						// curPosLeftEf = ctl.relEfTaskL->get_ef_pose().translation();

						curRotLeftEf = ctl.efTaskL->get_ef_pose().rotation();
						curPosLeftEf = ctl.efTaskL->get_ef_pose().translation();
						// cout << "curPosLeftEf\n" << curPosLeftEf.transpose() << endl;

						
						//sva::PTransformd R_X_efL(curRotLeftEf, curPosLeftEf); 
						sva::PTransformd R_X_efL(curPosLeftEf);


						/*object marker pose w.r.t to robot frame */
						for(int j=1;j<=tune1; j++)
						{
							sva::PTransformd M_X_ObjMarkerA(rotObjMarkerA, posObjMarkerA.middleCols((i-tune1)+j,i));
							// cout << "M_X_ObjMarkerA.trans() \n" << M_X_ObjMarkerA.translation().transpose() << endl;

							sva::PTransformd efL_X_ObjMarkerA;
							efL_X_ObjMarkerA = R_X_efL.inv()*M_X_ObjMarkerA*M_X_efLMarker.inv()*R_X_efL;
							

							newPosObjMarkerA(0,j-1) = efL_X_ObjMarkerA.translation()(0);
							newPosObjMarkerA(1,j-1) = efL_X_ObjMarkerA.translation()(1);
							newPosObjMarkerA(2,j-1) = efL_X_ObjMarkerA.translation()(2);
							

							/*get obj marker initials*/
							if(j==1)
							{
								initPosObjMarkerA = newPosObjMarkerA.col(j-1);
								// cout << " initPosObjMarkerA " << initPosObjMarkerA.transpose() << endl;
							}
							if(j==tune1)
							{
								ithPosObjMarkerA  = newPosObjMarkerA.col(tune1-1);
								// cout << " ithPosObjMarkerA " << ithPosObjMarkerA.transpose() << endl; 
							}
						}
						// cout << " newPosObjMarkerA " << newPosObjMarkerA.transpose() <<endl<< endl;
						// helpFun->plotPos(newPosObjMarkerA, tune1);


						/*get average velocity of previous 1sec obj motion*/
						curVelObjMarkerA  = ctl.handoverTraj->diff(newPosObjMarkerA)*fps;//ignore diff > XXXX
						// helpFun->plotVel(curVelObjMarkerA, tune1);
						// cout << "curVelObjMarkerA " << curVelObjMarkerA.transpose() <<endl<<endl;


						avgVelObjMarkerA  << ctl.handoverTraj->takeAverage(curVelObjMarkerA);
						// cout << "avgVelObjMarkerA " << avgVelObjMarkerA.transpose() << endl<<endl;


						/*get way points between obj inital motion*/
						auto actualPosObjMarkerA = ctl.handoverTraj->constVelocity(initPosObjMarkerA, ithPosObjMarkerA, tune1);
						// cout<< "slope " << get<1>(actualPosObjMarkerA).transpose()<< endl<< endl;
						// cout<< "const " << get<2>(actualPosObjMarkerA).transpose()<< endl<< endl;



						/*predict position in straight line after tune2 time*/
						//avgVelObjMarkerA //get<1>(actualPosObjMarkerA)
						predictPos = ctl.handoverTraj->constVelocityPredictPos(avgVelObjMarkerA, get<2>(actualPosObjMarkerA), tune2);

						/*get predicted way points between left ef and obj*/
						wp_efL_objMarkerA = 
						ctl.handoverTraj->constVelocity(ithPosObjMarkerA, predictPos, tune2);
						wp = get<0>(wp_efL_objMarkerA);
						collected = true;
						

						// cout << "predictPos " <<"\nFROM " << ithPosObjMarkerA.transpose() << "\nTO "<< predictPos.transpose() << endl;
						
						// cout << "wp " << get<0>(wp_efL_objMarkerA).transpose() << endl<< endl;
						// cout << "slope " << get<1>(wp_efL_objMarkerA).transpose() << endl<< endl;

						// cout << "wp " << wp.col(0).transpose() << endl;
						// cout << "wp.cols() " << wp.cols() << endl;
						// cout << "wp.rows() " << wp.rows() << endl;
					} //tune1
					

					if(collected)
					{
						// LOG_WARNING("new object pos updated ")
						// cout << i << endl;

						Eigen::Vector3d initPos = 
						ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")].translation();
						
						// ctl.posTask->position(initPos);

						if(removePrevTask)
						{	
							removePrevTask = false;
							ctl.solver().removeTask(ctl.efTaskL);
							ctl.solver().removeTask(ctl.efTaskR);
							ctl.solver().addTask(ctl.posTask);
						}

						initRefPos << -wp(0,0), -wp(1,0), wp(2,0);

						for(int it=0; it<wp.cols(); it++)
						{

							// refPos << wp.col(it);
							refPos << -wp(0,it), -wp(1,it), wp(2,it);// -ve X,Y
							// cout << "wp " << wp.col(it).transpose()<<endl;

							refVel << Eigen::MatrixXd::Zero(3,1); //avgVelObjMarkerA;
							refAcc << Eigen::MatrixXd::Zero(3,1);

							auto gothere = refPos + initPos -initRefPos;
							// cout << "gothere " << gothere.transpose()<<endl;

							if(
								(gothere(0))<= 0.7 && (gothere(1))<= 0.6 && (gothere(2))<=1.5 &&
								(gothere(0))>= 0.2 && (gothere(1))>= 0.25 && (gothere(2))>=0.9
								) 
							{
								ctl.posTask->position(gothere);
								ctl.posTask->refVel(refVel);
								ctl.posTask->refAccel(refAcc);
								// cout << "PosTask pos " << ctl.posTask->position().transpose()<<endl;

								if(gothere(1) >.45) //y
								{
									ctl.set_joint_pos("HEAD_JOINT0",  0.8); //+ve to move head left	
								}
								else
								{
									ctl.set_joint_pos("HEAD_JOINT0",  0.); //+ve to move head left
								}

								if(gothere(2) < 1.1) //z
								{
									ctl.set_joint_pos("HEAD_JOINT1",  0.4); //+ve to move head down	
								}
								else
								{
									ctl.set_joint_pos("HEAD_JOINT1",  -0.4); //+ve to move head down		
								}
								
							}
						}

						collected = false;
					}

					i = i + 1;
				}// check for non zero frame

			} // startCapture
			
			// output("OK");
			return false;
		}// run

	} // namespace states

} // namespace mc_handover





	/*------------------------------TO DOs----------------------------------------------
	1) trajectory task here and don't wait for it to finish -- overwrite in every loop
	2) if(wp_efL_objMarkerA-predictPos).eval().norm()> xxx -- pick another closet point on line
	----------------------------------------------------------------------------------*/

	// efL_X_ObjMarkerA = R_X_efL.inv()*M_X_R.inv()*M_X_ObjMarkerA;

	// cout << "efL_X_ObjMarkerA "<< efL_X_ObjMarkerA.translation().transpose() << endl;
	// cout <<"error " << M_X_ObjMarkerA.translation()- efL_X_ObjMarkerA.translation()<<endl;
	// cout << endl << endl;



	// // or
	// efL_X_ObjMarkerA = R_X_efL.inv()*M_X_efLMarker.inv()*R_X_efL*M_X_ObjMarkerA;

	// cout << "efL_X_ObjMarkerA2 "<< efL_X_ObjMarkerA.translation().transpose() << endl;
	// cout <<"error " << M_X_ObjMarkerA.translation()- efL_X_ObjMarkerA.translation()<<endl;
	// cout << endl << endl;

