
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
			(ctl.robots(), ctl.robots().robotIndex(), 10., 1000.);
			comTask->com(initialCom);
			ctl.solver().addTask(comTask);




			if(Flag_PosTask)
			{
				ctl.posTask = std::make_shared<mc_tasks::PositionTask>("LARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), 5.0, 1000);
				ctl.solver().addTask(ctl.posTask);
			}



			if(Flag_CirTraj)
			{
				ctl.posTask = std::make_shared<mc_tasks::PositionTask>("LARM_LINK6", ctl.robots(), ctl.robots().robotIndex(), 5.0, 1000);
				ctl.solver().addTask(ctl.posTask);

				Eigen::Vector3d zeroPos = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")].translation();
				ctl.posTask->position(zeroPos);
				ctl.cirTraj = std::make_shared<mc_handover::CircularTrajectory>(CircularTrajectory(0.1, 2000, zeroPos+Eigen::Vector3d(0, -0.1, 0)));
			}



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
		}




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
				bot = Eigen::MatrixXd::Random(3,1);
				obj = Eigen::MatrixXd::Random(3,1);
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
					// if(i%100==0)
					// {
					//   // cout <<"i "<< i << " posLeftEfMarker X " << posLeftEfMarker(0, i) << " ithFrame " << ithFrame <<endl;
					//   // cout << " posObjMarkerA X " << posObjMarkerA(0, i) << " ith  Frame " << ithFrame << endl;
					// }

					if(i%tunParam1 == 0)
					{
						/*get robot ef marker current pose*/
						curPosLeftEfMarker << posLeftEfMarker.col((i-tunParam1)+1);//1.162, -0.268, 1.074;
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
						for(int j=1;j<=tunParam1; j++)
						{
							sva::PTransformd M_X_ObjMarkerA(rotObjMarkerA, posObjMarkerA.middleCols((i-tunParam1)+j,i));
							// cout << "M_X_ObjMarkerA.trans() \n" << M_X_ObjMarkerA.translation().transpose() << endl;

							sva::PTransformd efL_X_ObjMarkerA;

							efL_X_ObjMarkerA = R_X_efL.inv()*M_X_R.inv()*M_X_ObjMarkerA;

							// cout << "efL_X_ObjMarkerA1 "<< efL_X_ObjMarkerA.translation().transpose() << endl;
							// cout <<"error " << M_X_ObjMarkerA.translation()- efL_X_ObjMarkerA.translation()<<endl;
							// cout << endl << endl;



							// // or
							// efL_X_ObjMarkerA = R_X_efL.inv()*M_X_efLMarker.inv()*R_X_efL*M_X_ObjMarkerA;
							
							// cout << "efL_X_ObjMarkerA2 "<< efL_X_ObjMarkerA.translation().transpose() << endl;
							// cout <<"error " << M_X_ObjMarkerA.translation()- efL_X_ObjMarkerA.translation()<<endl;
							// cout << endl << endl;



							// // or
							// efL_X_ObjMarkerA = M_X_efLMarker.inv()*M_X_ObjMarkerA;

							// cout << "efL_X_ObjMarkerA3 "<< efL_X_ObjMarkerA.translation().transpose() << endl;
							// cout << endl << endl;
							// cout <<"error " << M_X_ObjMarkerA.translation()- efL_X_ObjMarkerA.translation()<<endl;
							// cout << endl << endl;
							
							newPosObjMarkerA(0,j-1) = efL_X_ObjMarkerA.translation()(0);
							newPosObjMarkerA(1,j-1) = efL_X_ObjMarkerA.translation()(1);
							newPosObjMarkerA(2,j-1) = efL_X_ObjMarkerA.translation()(2);

							/*get obj marker initials*/
							if(j==1)
							{
								initPosObjMarkerA = newPosObjMarkerA.col(j-1);
								// cout << " initPosObjMarkerA " << initPosObjMarkerA.transpose() << endl;
							}
							if(j==tunParam1)
							{
								ithPosObjMarkerA  = newPosObjMarkerA.col(tunParam1-1);
								// cout << " ithPosObjMarkerA " << ithPosObjMarkerA.transpose() << endl; 
							}
						}
						// cout << " newPosObjMarkerA " << newPosObjMarkerA.transpose() <<endl<< endl;
						// helpFun->plotPos(newPosObjMarkerA, tunParam1);


						/*get average velocity of previous 1sec obj motion*/
						curVelObjMarkerA  = ctl.handoverTraj->diff(newPosObjMarkerA)*fps;//ignore diff > XXXX
						// helpFun->plotVel(curVelObjMarkerA, tunParam1);
						// cout << "curVelObjMarkerA " << curVelObjMarkerA.transpose() <<endl<<endl;


						avgVelObjMarkerA  << ctl.handoverTraj->takeAverage(curVelObjMarkerA);
						// cout << "avgVelObjMarkerA " << avgVelObjMarkerA.transpose() << endl<<endl;


						/*get way points between obj inital motion*/
						auto actualPosObjMarkerA = ctl.handoverTraj->constVelocity(initPosObjMarkerA, ithPosObjMarkerA, tunParam1);
						// cout<< "slope " << get<1>(actualPosObjMarkerA).transpose()<< endl<< endl;
						// cout<< "const " << get<2>(actualPosObjMarkerA).transpose()<< endl<< endl;


						/*predict position in straight line after tunParam2 time*/  
						//avgVelObjMarkerA //get<1>(actualPosObjMarkerA)
						predictPos = ctl.handoverTraj->constVelocityPredictPos(avgVelObjMarkerA, get<2>(actualPosObjMarkerA), tunParam2);
						// cout << "predictPos " <<"\nFROM " << ithPosObjMarkerA.transpose() << "\nTO "<< predictPos.transpose() << endl<< endl;


						/*get predicted way points between left ef and obj*/
						wp_efL_objMarkerA = ctl.handoverTraj->constVelocity(ithPosObjMarkerA, predictPos, tunParam2);
						// cout << "wp " << get<0>(wp_efL_objMarkerA).transpose() << endl<< endl;
						// cout << "slope " << get<1>(wp_efL_objMarkerA).transpose() << endl<< endl;

						// ctl.handoverTrajTask->config.pos = get<0>(wp_efL_objMarkerA);


						/* set ef pose based on prediction */
						

						if(Flag_PosTask)
						{
							auto gothere = Eigen::MatrixXd::Random(3,1);
							if(onceTrue)
							{
								// cout << " from curPosLeftEf " << curPosLeftEf.transpose() << " To predictPose "<< predictPos.transpose() << endl;

								if( gothere(0)<= 0.5 && gothere(1)<= 0.5 && gothere(2)<=1.5 &&
									gothere(0)>= 0.0 && gothere(1)>= 0.0 && gothere(2)>=0.5)
								{
									sva::PTransformd dtrL(curRotLeftEf, {fabs(gothere(0)), fabs(gothere(1)), fabs(gothere(2))+0.3 });
									cout << "gothere " << dtrL.translation().transpose() << endl;
									// ctl.efTaskL->set_ef_pose(dtrL);

									ctl.posTask->position({fabs(gothere(0)), fabs(gothere(1)), fabs(gothere(2))+0.3});
									ctl.posTask->refVel(Eigen::MatrixXd::Zero(3,1));
									cout << "avg velocity " << avgVelObjMarkerA.transpose() << endl;

									/*real-time*/
									// ctl.posTask->position(predictPos);
									// ctl.posTask->refVel(avgVelObjMarkerA);

									// ctl.posTask->position(get<0>(wp_efL_objMarkerA).col(1));
									// ctl.posTask->refVel(get<1>(wp_efL_objMarkerA));
								}
								onceTrue = false;
							}
						}


					} //tunParam1
					i = i + 1;

					/*------------------------------TO DOs----------------------------------------------
					1) trajectory task here and don't wait for it to finish -- overwrite in every loop
					2) if(wp_efL_objMarkerA-predictPos).eval().norm()> xxx -- pick another closet point on line
					3) replace predictPos with wp using trajTask
					4) if(ctl.efTaskL->eval().norm()<0.02) // ctl.efTaskL->eval().speed()<0.02
					----------------------------------------------------------------------------------*/
					

					/*move ef in circle*/
					if(Flag_CirTraj)
					{
						if(onceTrue)
						{   
							auto pair = ctl.cirTraj->pop();
							ctl.posTask->position(pair.first);
							ctl.posTask->refVel(pair.second);
						}
						onceTrue = false;
					}



					onceTrue = true;
				}// check for non zero frame

			} // startCapture
			// output("OK");
			return false;
		}






	} // namespace states

} // namespace mc_handover







/*************************************************************************************/

/*teardown*/
// controller.solver().removeTask(comt_);

// for(int i=0; i<100; i++)
// {
//   cout << " posRobotMarker X " << posLeftEfMarker(0, i) << " i "<< i << endl;
//   cout << " posObjMarker X " << posObjMarkerA(0, i) << " i "<< i << endl;
// }

// auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
// Cortex_FreeFrame(getCurFrame);
// Cortex_FreeFrame(&FrameofData);



// cout << "size newPosObjMarkerA " << newPosObjMarkerA.cols() << endl;
// cout << "size curVelObjMarkerA " << curVelObjMarkerA.cols() << endl;
// cout << "tunParam1 " << tunParam1 << "tunParam2  " << tunParam2 <<  endl;


// if(j%(tunParam1/5)==0)
// {
//   cout <<"curposObjMarkerA " <<M_X_ObjMarkerA.translation().transpose()<<endl;
//   cout <<"newPosObjMarkerA  "<< newPosObjMarkerA.col(j-1).transpose() << endl;
//   cout << "diff    " << M_X_ObjMarkerA.translation().transpose() - newPosObjMarkerA.col(j-1).transpose() << endl;
// }

// if(j%(tunParam1/5)==0)
// {
// cout << "efL_X_ObjMarkerA\n"<< efL_X_ObjMarkerA.translation().transpose() << endl;
// cout << "newPosObjMarkerA\n"<< newPosObjMarkerA.col(j-1) << endl;
// cout << "checking transfrm \n"<< " " <<
// efL_X_ObjMarkerA.translation()(0)- newPosObjMarkerA(0,j-1)<< " "<<
// efL_X_ObjMarkerA.translation()(1)- newPosObjMarkerA(1,j-1)<< " "<<
// efL_X_ObjMarkerA.translation()(2)- newPosObjMarkerA(2,j-1)<< endl;
// }

// /* using matplotlib */
// for(int it = 0; it<get<0>(out).rows(); it++)
// { 
//   if(it%10==0)
//   {
//     x.push_back(get<0>(out)(it,0));
//     y.push_back(get<0>(out)(it,1));
//     z.push_back(get<0>(out)(it,2));
//     tp.push_back(it);
//     plt::clf();
//     plt::plot(tp,x);
//     plt::plot(tp,y);
//     plt::plot(tp,z);
//     plt::xlim(0, tunParam2);
//     plt::pause(0.0000000001);                    
//   }
// }

/* get ef cur pose */
// leftHandPosW  = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")];
// rightHandPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")];

/* debug */


// cout << "xxx\n " << xxx.translation() << endl;
// cout << "efL_X_ObjMarkerA\n"<< efL_X_ObjMarkerA << endl;

// cout << "initPosObjMarkerA " << initPosObjMarkerA << endl;
// cout << "ithPosObjMarkerA " << ithPosObjMarkerA << endl;
// cout << "curVelObjMarkerA " << curVelObjMarkerA.topLeftCorner(3,10) << endl;
// cout << "avgVelObjMarkerA " << avgVelObjMarkerA << endl;



// cout << "curPosLeftEf\n" << curPosLeftEf.transpose() << endl;
// cout << "curPosLeftEfMarker\n" << curPosLeftEfMarker.transpose() << endl;
// cout << "posObjMarkerA\n" << posObjMarkerA.col(i).transpose() << endl;
// cout << "posObjMarkerA_wrt_robotLeftEf translation\n" << efL_X_ObjMarkerA.translation().transpose() << endl;
// cout << "posObjMarkerA_wrt_robotLeftEf rot\n" << efL_X_ObjMarkerA.rotation() << endl;




/*std::vector<int> nums{3, 4, 2, 8, 15, 267};
auto print = [](const int& n) { std::cout << " " << n; };
std::for_each(nums.begin(), nums.end(), print);
std::for_each(nums.begin(), nums.end(), [](int &n){ std::cout <<'\n'<< " " <<n++; });*/


/*Eigen::Vector3d tL( 0.7, 0.35, .3 );
Eigen::Matrix3d getCurRotL =  relEfTaskL->get_ef_pose().rotation();
sva::PTransformd dtrL(getCurRotL, tL);
relEfTaskL->set_ef_pose(dtrL);

sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];
initPosR <<  0.30, -0.35, 0.3;
relEfTaskR->set_ef_pose(sva::PTransformd(i-tunParam1+1sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.i-tunParam1+1rotation(), initPosR));
solver().addTask(relEfTaskR);*/


/*usleep(1000000);
if(FrameofData->iFrame == 1000000)
{
cout << "\ntotal delay from camera " << del << endl;

printf("\n****** Paused live mode ... exiting Cortex ******\n");
Cortex_FreeFrame(FrameofData);
Cortex_Request("Pause", &pResponse, &nBytes);
Cortex_Exit();
}*/

/*
for(int b = 0; b<totalBodies; b++)
{
for(int m = 0; m<bodyMarkers.at(b); m++)
{
cout << "body " << b+1 << "\n marker == " << (m+1) << endl <<
" X: " << FrameofData->BodyData[b].Markers[m][0] << endl <<
" Y: " << FrameofData->BodyData[b].Markers[m][1] << endl <<
" Z: " << FrameofData->BodyData[b].Markers[m][2] << endl;
}
}*/
