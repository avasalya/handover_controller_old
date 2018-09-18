
#include "StartMocapStep.h"

namespace mc_handover
{

	namespace states 
	{

		void StartMocapStep::configure(const mc_rtc::Configuration & config)
		{
			cout << "config" << endl;
		}


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
			else
			{
				bot = Eigen::MatrixXd::Random(3,1)*0.01;
				obj = Eigen::MatrixXd::Random(3,1)*0.01;

				// cout << "bot " << bot.transpose() << endl;
				// cout << "obj " << obj.transpose() << endl;

				// eflrot = Eigen::MatrixXd::Random(3,3);
				eflpos = bot;// Eigen::MatrixXd::Random(3,1);
			}



			/* start only when ithFrame == 1 */
			if(startCapture)
			{

				/* get object marker pose */
				if(Flag_CORTEX)
				{
					robotBodyMarker <<	
					FrameofData.BodyData[robotBody].Markers[robotMarkerNo][0], // X left marker
					FrameofData.BodyData[robotBody].Markers[robotMarkerNo][1], // Y
					FrameofData.BodyData[robotBody].Markers[robotMarkerNo][2]; // Z

					objectBodyMarker <<	
					FrameofData.BodyData[objBody].Markers[objMarkerNo][0], // X farmost
					FrameofData.BodyData[objBody].Markers[objMarkerNo][1], // Y
					FrameofData.BodyData[objBody].Markers[objMarkerNo][2]; // Z
				}
				else
				{
					robotBodyMarker << bot;
					objectBodyMarker << obj;
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
					

					if(i%tune1 == 0)
					{
						/*get robot ef marker current pose*/
						curPosLeftEfMarker << posLeftEfMarker.col((i-tune1)+1);//1.162, -0.268, 1.074;
						sva::PTransformd M_X_efLMarker(curRotLeftEfMarker, curPosLeftEfMarker);
						// cout << "curPosLeftEfMarker " << curPosLeftEfMarker.transpose() << endl;


						/*get robot ef current pose*/
						if(Flag_CORTEX)
						{
							// curRotLeftEf = ctl.relEfTaskL->get_ef_pose().rotation();
							// curPosLeftEf = ctl.relEfTaskL->get_ef_pose().translation();

							curRotLeftEf = ctl.efTaskL->get_ef_pose().rotation();
							curPosLeftEf = ctl.efTaskL->get_ef_pose().translation();
						}
						else
						{
							// curRotLeftEf << eflrot;
							curRotLeftEf = ctl.efTaskL->get_ef_pose().rotation();
							curPosLeftEf << eflpos;
						}
						// cout << "curPosLeftEf\n" << curPosLeftEf.transpose() << endl;
						sva::PTransformd R_X_efL(curRotLeftEf, curPosLeftEf); 


						/*get transformation martix from mocap frame to robot frame*/
						Eigen::Vector3d pos(0.862, 0.032, 0.524);
						Eigen::Matrix3d ori; ori <<  0, 0, 1,    0, 1, 0,    0, 0, 1;
						sva::PTransformd M_X_R(ori,pos);
						
						/*may not need this with real-time robot demo*/
						// sva::PTransformd M_X_R = R_X_efL.inv()*M_X_efLMarker;
						// cout << "M_X_R " << M_X_R.translation().transpose() << endl;


						/*object marker pose w.r.t to robot frame */
						for(int j=1;j<=tune1; j++)
						{
							sva::PTransformd M_X_ObjMarkerA(rotObjMarkerA, posObjMarkerA.middleCols((i-tune1)+j,i));
							// cout << "M_X_ObjMarkerA.trans() \n" << M_X_ObjMarkerA.translation().transpose() << endl;

							sva::PTransformd efL_X_ObjMarkerA;

							// efL_X_ObjMarkerA = R_X_efL.inv()*M_X_R.inv()*M_X_ObjMarkerA;

							// efL_X_ObjMarkerA = R_X_efL.inv()*M_X_efLMarker.inv()*R_X_efL*M_X_ObjMarkerA;
							
							efL_X_ObjMarkerA = M_X_efLMarker.inv()*M_X_ObjMarkerA;

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
						// cout << "predictPos " <<"\nFROM " << ithPosObjMarkerA.transpose() << "\nTO "<< predictPos.transpose() << endl;


						/*get predicted way points between left ef and obj*/
						wp_efL_objMarkerA = ctl.handoverTraj->constVelocity(ithPosObjMarkerA, predictPos, tune2);
						// cout << "wp " << get<0>(wp_efL_objMarkerA).transpose() << endl<< endl;
						// cout << "slope " << get<1>(wp_efL_objMarkerA).transpose() << endl<< endl;

						wp = get<0>(wp_efL_objMarkerA);
						// cout << "wp " << wp.col(0).transpose() << endl;
						// cout << "wp.cols() " << wp.cols() << endl;
						// cout << "wp.rows() " << wp.rows() << endl;

						collected  = true;
					} //tune1


					if(collected)
					{
						LOG_WARNING("new object pos updated ")
						cout << i << endl;

						Eigen::Vector3d initPos = 
						ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")].translation();
						
						// ctl.posTask->position(initPos);


						ctl.solver().removeTask(ctl.efTaskL);
						ctl.solver().removeTask(ctl.efTaskR);
						ctl.solver().addTask(ctl.posTask);

						// if(	predictPos(0)<= 1.5 && predictPos(1)<= 1 && predictPos(2)<=1.5 &&
						// 	predictPos(0)>= 0.0 && predictPos(1)>= 0.0 && predictPos(2)>=0.8
						// 	)
						{

							for(int it=0; it<wp.cols(); it++)
							{

								refPos << wp.col(it);
								refVel << Eigen::MatrixXd::Zero(3,1);
								refAcc << Eigen::MatrixXd::Zero(3,1);

								// cout << "wp " << wp.col(it).transpose()<<endl;

								ctl.posTask->position(refPos+initPos - wp.col(0));
								ctl.posTask->refVel(refVel);
								ctl.posTask->refAccel(refAcc);
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

