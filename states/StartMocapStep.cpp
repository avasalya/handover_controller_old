
#include "StartMocapStep.h"

namespace mc_handover
{

	namespace states {

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
			cout << "start -- StartMocapStep " << endl;
  		auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
      // comt_ = std::make_shared<mc_tasks::CoMTask>(ctl.robots(), 0);
      // ctl.solver().addTask(comt_);

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
       printf("\n*** Starting live mode ***\n");
       retval = Cortex_Request("LiveMode", &pResponse, &nBytes);      
		}


		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
          auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

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
          
          /* start only when ithFrame == 1 */  
          if(startCapture)
          {
            /* get object marker pose */
            robotBodyMarker <<  FrameofData.BodyData[robotBody].Markers[0][0], // X left marker
                                FrameofData.BodyData[robotBody].Markers[0][1], // Y
                                FrameofData.BodyData[robotBody].Markers[0][2]; // Z

            objectBodyMarker << FrameofData.BodyData[objBody].Markers[0][0], // X farmost
                                FrameofData.BodyData[objBody].Markers[0][1], // Y
                                FrameofData.BodyData[objBody].Markers[0][2]; // Z

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

                if(i%tunParam1 == 0) // 1sec at 200fps
                {

                  /*get robot ef current pose*/
                  curRotLeftEf = ctl.relEfTaskL->get_ef_pose().rotation();
                  curPosLeftEf = ctl.relEfTaskL->get_ef_pose().translation();
                  sva::PTransformd X_robotLeftEf(curRotLeftEf, curPosLeftEf);


                  /*get robot ef marker current pose*/
                  curPosLeftEfMarker << posLeftEfMarker.col(i-tunParam1+1);
                  cout << "curPosLeftEfMarker " << curPosLeftEfMarker.transpose() << endl;
                  sva::PTransformd X_robotLeftEfMarker(curRotLeftEfMarker, curPosLeftEfMarker);



                  /*get transformation martix from mocap frame to robot EF frame*/
                  sva::PTransformd X_mocap_robotLeftEf = X_robotLeftEf.inv()*X_robotLeftEfMarker;
                  cout << "X_mocap_robotLeftEf " << X_mocap_robotLeftEf.translation().transpose() << endl;

                  /***************************************************/
                  
                  /*object marker pose w.r.t to robot ef frame */
                  for(int j=1;j<=tunParam1; j++)
                  {
                    sva::PTransformd X_poseObjMarkerA(rotObjMarkerA, posObjMarkerA.middleCols(i-tunParam1+j,i));
                    // cout << "X_poseObjMarkerA.trans() \n" << X_poseObjMarkerA.translation().transpose() << endl;

                    sva::PTransformd newPoseObjMarkerA = 
                    X_robotLeftEf.inv()*X_mocap_robotLeftEf.inv()*X_poseObjMarkerA;
                    
                    poseObjMarkerA_wrt_robotLeftEf(0,j-1) = newPoseObjMarkerA.translation()(0);
                    poseObjMarkerA_wrt_robotLeftEf(1,j-1) = newPoseObjMarkerA.translation()(1);
                    poseObjMarkerA_wrt_robotLeftEf(2,j-1) = newPoseObjMarkerA.translation()(2);
                    
                    // if(j%(tunParam1/5)==0)
                    // {cout << "poseObjMarkerA_wrt_robotLeftEf\n"<< 
                    // poseObjMarkerA_wrt_robotLeftEf.transpose() << endl;}
                    
                  }cout << "/***************************************************/" << endl;

                  // /* using matplotlib */
                  // for(int j=0;j<tunParam1; j++)
                  // { 
                  //     x.push_back(poseObjMarkerA_wrt_robotLeftEf(0,j));
                  //     y.push_back(poseObjMarkerA_wrt_robotLeftEf(1,j));
                  //     z.push_back(poseObjMarkerA_wrt_robotLeftEf(2,j));
                  //     tp.push_back(j);
                  //   if(j%(tunParam1/5)==0)
                  //   {
                  //     cout <<"x y z   " << x.at(j) << "  " << y.at(j) << "  " << z.at(j) << endl;
                  //     plt::clf();
                  //     plt::plot(tp,x);
                  //     plt::plot(tp,y);
                  //     plt::plot(tp,z);
                  //     plt::ylim(-1.5,1.5);
                  //     plt::xlim(0, tunParam1);
                  //     plt::pause(1e-10);//(0.0000000001);
                  //   }
                  // }

                  /***************************************************/

                  // /*get obj marker initials*/
                  // initPosObjMarkerA = poseObjMarkerA_wrt_robotLeftEf.col(i-tunParam1+1);
                  // ithPosObjMarkerA  = poseObjMarkerA_wrt_robotLeftEf.col(i);
                  
                  // /*get average velocity of previous 1sec obj motion*/
                  // curVelObjMarkerA  = diff(poseObjMarkerA_wrt_robotLeftEf)*fps;
                  // avgVelObjMarkerA  = takeAverage(curVelObjMarkerA);

                  // // cout << "initPosObjMarkerA\n " << initPosObjMarkerA.transpose() << endl;
                  // // cout << "ithPosObjMarkerA\n " << ithPosObjMarkerA.transpose() << endl;
                  // // cout << "avgVelObjMarkerA\n " << avgVelObjMarkerA.transpose() << endl;
                  // // cout << "curVelObjMarkerA\n " << curVelObjMarkerA << endl;

                  // /*predict position in straight line after tunParam2 time*/
                  // auto predictPos = ctl.handoverTraj->constVelocityPredictPos(ithPosObjMarkerA, avgVelObjMarkerA, tunParam2);

                  // /*get way points between obj and left ef*/
                  // auto wp_objMarkerA_robotLeftEf = ctl.handoverTraj->constVelocity(ithPosObjMarkerA, predictPos, tunParam2);


                  /***************************************************/
                  // robotPredictPos on points between ithPosObjMarkerA to predictPos from curPosLeftEf
                  // (curPosLeftEf-predictPos).norm()> xxx -- pick another closet point on line

                    // cout << "curPosLeftEf\n" << curPosLeftEf.transpose() << endl;
                    // cout << "predictPos\n " << predictPos.transpose() << endl;
                  // /* set ef pose based on prediction */
                  // if(onceTrue)
                  // { 
                  //   sva::PTransformd dtrL(curRotLeftEf, predictPos);
                  //   // ctl.relEfTaskL->set_ef_pose(dtrL);
                  //   // onceTrue = false;
                  // }

                  /**/
                  // if(ctl.relEfTaskL->eval().norm()<0.02) // ctl.relEfTaskL->eval().speed()<0.02
                  // { 
                  //   onceTrue = true;
                  //   cout << "True " << onceTrue << endl;
                  // }
                

                } //tunParam1

                // poseObjMarkerA_wrt_robotLeftEf = Eigen::MatrixXd::Zero(3,tunParam1); //clear out
                i = i + 1;              
            }// check for non zero frame
          } // startCapture
           
          // output("OK");
          return false;
    }

    void StartMocapStep::teardown(mc_control::fsm::Controller & controller)
    { 
      // controller.solver().removeTask(comt_);
        
        // for(int i=0; i<100; i++)
        // {
        //   cout << " posRobotMarker X " << posLeftEfMarker(0, i) << " i "<< i << endl;
        //   cout << " posObjMarker X " << posObjMarkerA(0, i) << " i "<< i << endl;
        // }

        // auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
        // Cortex_FreeFrame(getCurFrame);
        // Cortex_FreeFrame(&FrameofData);
    }


    
    Eigen::MatrixXd StartMocapStep::diff(Eigen::MatrixXd data)
    { 
      Eigen::MatrixXd matrix_diff;
      matrix_diff.resize(data.rows(),data.cols());

      for(int j=0; j< data.rows(); j++)
      {
        for (unsigned int i=1; i<data.cols(); i++)
        {
          matrix_diff(j,i) = (data(j,i)-data(j,i-1));
        } 
      }
      return matrix_diff;
    }


    Eigen::Vector3d StartMocapStep::takeAverage(Eigen::MatrixXd m)
    {
      std::vector<double> v[m.rows()];
      double avg[m.rows()];

      Eigen::Vector3d mean;

      for(int j=0; j<m.rows(); j++)
      {
        for(int i=0;i<m.cols();i++)
        {
          v[j].push_back(m(j,i));
          avg[j]+=v[j].at(i);
        }
        mean(j)= avg[j]/v[j].size();
      }
      return mean;
    }



  } // namespace states

} // namespace mc_handover




/*************************************************************************************/

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
// cout << "poseObjMarkerA_wrt_robotLeftEf\n"<< poseObjMarkerA_wrt_robotLeftEf << endl;

  // cout << "initPosObjMarkerA " << initPosObjMarkerA << endl;
  // cout << "ithPosObjMarkerA " << ithPosObjMarkerA << endl;
  // cout << "curVelObjMarkerA " << curVelObjMarkerA.topLeftCorner(3,10) << endl;
  // cout << "avgVelObjMarkerA " << avgVelObjMarkerA << endl;



// cout << "curPosLeftEf\n" << curPosLeftEf.transpose() << endl;
// cout << "curPosLeftEfMarker\n" << curPosLeftEfMarker.transpose() << endl;
// cout << "posObjMarkerA\n" << posObjMarkerA.col(i).transpose() << endl;
// cout << "posObjMarkerA_wrt_robotLeftEf translation\n" << poseObjMarkerA_wrt_robotLeftEf.translation().transpose() << endl;
// cout << "posObjMarkerA_wrt_robotLeftEf rot\n" << poseObjMarkerA_wrt_robotLeftEf.rotation() << endl;




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
