
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
    		// auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

         Cortex_SetVerbosityLevel(VL_Info);
         Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);

         retval = Cortex_Initialize("10.1.1.180", "10.1.1.190");
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
        

        posRobotMarker.resize(3,60000);
        posObjMarker.resize(3,60000);

 // used to run ./clienttest from here //
 // system("/home/avasalya/mc_handover_controller/build/cortex/clienttest");
		}


		bool StartMocapStep::run(mc_control::fsm::Controller & controller)
		{
          // auto & ctl = static_cast<mc_handover::HandoverController&>(controller);

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
          else if(ithFrame == 100)
          { 
            int nthFrame = ithFrame;
            cout << "nthFrame "<< nthFrame << endl;
            // Cortex_FreeFrame(getCurFrame);
            // Cortex_FreeFrame(&FrameofData);
            Cortex_Request("Pause", &pResponse, &nBytes);
            Cortex_Exit();
            output("OK");
            return true;
          }
          
          /* start only when ithFrame == 1 */  
          if(startCapture)
          {
            RMarker << FrameofData.BodyData[0].Markers[0][0], // X
                       FrameofData.BodyData[0].Markers[0][1], // Y
                       FrameofData.BodyData[0].Markers[0][2]; // Z
            // for(int i=0; i<3; i++)
            // { cout << RMarker(i) <<" ";} cout << endl;
            OMarker << FrameofData.BodyData[1].Markers[0][0], // X
                       FrameofData.BodyData[1].Markers[0][1], // Y
                       FrameofData.BodyData[1].Markers[0][2]; // Z
            // for(int i=0; i<3; i++)
            // { cout << OMarker(i) <<" ";} cout << endl;

            /* check for non zero frame only */ 
            if(RMarker(0) != 0 || OMarker(0) != 0)
            {
              posRobotMarker(0, i) = RMarker(0); // X
              posRobotMarker(1, i) = RMarker(1); // X
              posRobotMarker(2, i) = RMarker(2); // X
              cout << " posRobotMarker X " << posRobotMarker(0, i) << " ithFrame " << ithFrame <<endl;

              posObjMarker(0, i) = OMarker(0); // X
              posObjMarker(1, i) = OMarker(1); // X
              posObjMarker(2, i) = OMarker(2); // X
              cout << " posObjMarker X " << posObjMarker(0, i) << " ithFrame " << ithFrame <<endl;
              i = i + 1;              
            }


            // if(RMarker(0) != 0 || RMarker(1) != 0 || RMarker(2) != 0)
            // {
            //   posRobotMarker(0, ir) = RMarker(0); // X
            //   posRobotMarker(1, ir) = RMarker(1); // X
            //   posRobotMarker(2, ir) = RMarker(2); // X
            //   cout << " posRobotMarker X " << posRobotMarker(0, ir) << " ithFrame " << ithFrame <<endl;
            //   ir = ir + 1;
            // }

            // if(OMarker(0) != 0 || OMarker(1) != 0 || OMarker(2) != 0)
            // {            
            //   posObjMarker(0, io) = OMarker(0); // X
            //   posObjMarker(1, io) = OMarker(1); // X
            //   posObjMarker(2, io) = OMarker(2); // X
            //   cout << " posObjMarker X " << posObjMarker(0, io) << " ithFrame " << ithFrame <<endl;
            //   io = io + 1;
            // }


          }

                      
           
          // MinJerk::minJerkZeroBoundary(const MatrixXd & xi, const MatrixXd & xf, double tf);
          // mjObj.produceWp(posRobotMarker(1), posRobotMarker(100),  i, 100);

          // rotRobotMarker = relEfTaskL->get_ef_pose().rotation();          
          // rotObjMarker = Eigen::Matrix3d::Identity();

          // sva::PTransformd robotMarker(rotRobotMarker, posRobotMarker);
          // sva::PTransformd objMarker(rotObjMarker, posObjMarker);

          // leftHandPosW  = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("LARM_LINK6")];
          // rightHandPosW = ctl.robot().mbc().bodyPosW[ctl.robot().bodyIndexByName("RARM_LINK6")];

          // output("OK");
          return false;
    }


    void StartMocapStep::teardown(mc_control::fsm::Controller & controller)
    { 
        for(int i=0; i<100; i++)
        {
          cout << " posRobotMarker X " << posRobotMarker(0, i) << " i "<< i << endl;
          cout << " posObjMarker X " << posObjMarker(0, i) << " i "<< i << endl;
        }

      // auto & ctl = static_cast<mc_handover::HandoverController&>(controller);
       // Cortex_FreeFrame(getCurFrame);
       // Cortex_FreeFrame(&FrameofData);
    }


  } // namespace states

} // namespace mc_handover




/*************************************************************************************/

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
relEfTaskR->set_ef_pose(sva::PTransformd(sva::RotY(-(M_PI/180)*90)*sva::RotX(-(M_PI/180)*90)*BodyW.rotation(), initPosR));
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
