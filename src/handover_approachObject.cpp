#include "handover_approachObject.h"

namespace mc_handover
{
	ApproachObject::ApproachObject()
	{ cout<<"\033[1;50m handover object created\033[0m\n"; }



	ApproachObject::~ApproachObject()
	{ cout<<"\033[1;50m handover object destroyed\033[0m\n"; }



	/*allocate memory*/
	void ApproachObject::initials()
	{
		/*markers Name strings*/
		strMarkersBodyName = {"4mars_robot_left_hand", "4mars_robot_right_hand", "5mars_subj_right_hand", "3mars_obj", "4mars_subj_left_hand"};

		robotLtMarkers = {"wristLtEfA", "wristLtEfB", "gripperLtEfA", "gripperLtEfB"};//0-3
		robotRtMarkers = {"wristRtEfA", "wristRtEfB", "gripperRtEfA", "gripperRtEfB"};//4-7

		objMarkers = {"left", "center", "right"};//8-10

		subjRtMarkers = {"fingerSubjRt", "lShapeRtA", "lShapeRtB", "lShapeRtC", "lShapeRtD"};//11-15
		subjLtMarkers = {"fingerSubjLt", "lShapeLtA", "lShapeLtC", "lShapeLtD"};//16-19

		strMarkersName.insert(strMarkersName.begin(), robotLtMarkers.begin(), robotLtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), robotRtMarkers.begin(), robotRtMarkers.end());

		strMarkersName.insert(strMarkersName.end(), objMarkers.begin(), objMarkers.end());

		strMarkersName.insert(strMarkersName.end(), subjRtMarkers.begin(), subjRtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), subjLtMarkers.begin(), subjLtMarkers.end());

		for(unsigned int k=0; k<strMarkersName.size(); k++)
			markers_name_index[strMarkersName[k]] = k;

		Markers.resize(totalMarkers);
		object.resize(objMarkers.size());

		markersPos.resize(totalMarkers);
		for(int m=0; m<totalMarkers; m++)
			{ markersPos[m] = Eigen::MatrixXd::Zero(3,60000); }

		/*prediction controller parameter*/
		tuner << 600., 30., 20.;
		// tuner(2) = tuner(0)/tuner(1);
	
		t_predict = (int)tuner(0);
		t_observe = (int)tuner(1);
		it = (int)tuner(2);//t_predict/t_observe;
	}


	
	bool ApproachObject::checkFrameOfData(std::vector<Eigen::Vector3d>)
	{
		bool checkNonZero{false};

		if(Flag_withoutRobot)
		{
			/*robot left markers*/
			Markers[0] << 0.208955, 0.350982, 0.552377;
			Markers[1] << 0.15638, 0.352496, 0.547814;
			Markers[2] << 0.217815, 0.334505, 0.432962;
			Markers[3] << 0.162401,  0.33451, 0.42898;

			/*robot right markers*/
			Markers[4] << 0.14048, -0.309184, 0.550067;
			Markers[5] << 0.198771, -0.308889,  0.555116;
			Markers[6] << 0.148997, -0.29622, 0.418868;
			Markers[7] << 0.202506, -0.293441, 0.425241;
		}

		for(unsigned int k=0; k<strMarkersName.size(); k++)
		{
			// LOG_WARNING(k<<" "<<strMarkersName[k])//<<" "<< Markers[ markers_name_index[ strMarkersName[k] ] ].transpose())
			if( Markers[k](0)>-10 && Markers[k](0)!=0 && Markers[k](0)<10 )
				{ checkNonZero = true; }
			else
			{ return false; }
		}
		return checkNonZero;
	}


	
	bool ApproachObject::handoverRun()
	{
		/*check for non zero frame only and store them*/
		if( checkFrameOfData(Markers) )
		{
			i+=1;
			for(int m=0; m<totalMarkers; m++)
				{ markersPos[m].col(i) << Markers[m]; }

			/*for GUI*/
			object[0] = markersPos[8].col(i);//left
			object[1] = markersPos[9].col(i);//center
			object[2] = markersPos[10].col(i);//right
			
			objectPos = markersPos[9].col(i); //object[1]//center

			/*move EF when subject approaches object 1st time*/
			obj_rel_robotLtHand = ( markersPos[2].col(i) - objectPos ).norm();//gripperLtEfA
			obj_rel_robotRtHand = ( markersPos[6].col(i) - objectPos ).norm();//gripperRtEfA

			obj_rel_subjRtHand = ( markersPos[14].col(i) - objectPos ).norm();//lshpRtC
			obj_rel_subjLtHand = ( markersPos[18].col(i) - objectPos ).norm();//lshpLtC

			return true;
		}
		else
		{ return false; }
	}


	
	std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> ApproachObject::predictionController(const sva::PTransformd& robotEf, const Eigen::Matrix3d & curRotLink6, std::vector<std::string> subjMarkersName, std::vector<std::string> robotMarkersName)
	{
		bool ready{false};

		Eigen::Vector3d x, y, z, lshp_X, lshp_Y, lshp_Z;
		Eigen::Vector3d initRefPos, curPosEf, curPosEfMarker,  curPosLshp, ithPosSubj, avgVelSubj, predictPos;

		Eigen::MatrixXd curVelSubj, newPosSubj, wp;

		sva::PTransformd X_R_ef;
		sva::PTransformd X_M_efMarker;
		sva::PTransformd X_R_M;
		sva::PTransformd X_M_Subj;
		sva::PTransformd X_ef_Subj;

		std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_efL_Subj;


		/*prediction_ tuner*/
		t_predict = (int)tuner(0);
		t_observe = (int)tuner(1);
		it = (int)tuner(2); //t_predict/t_observe;

		newPosSubj = Eigen::MatrixXd::Zero(3, t_observe);

		/*get robot ef current pose*/
		curPosEf = robotEf.translation();


		/*get robot ef marker(s) current pose*/
		curPosEfMarker << 0.25*( 
			markersPos[markers_name_index[robotMarkersName[0]]].col((i-t_observe)+1) +
			markersPos[markers_name_index[robotMarkersName[1]]].col((i-t_observe)+1) +
			markersPos[markers_name_index[robotMarkersName[2]]].col((i-t_observe)+1) + 
			markersPos[markers_name_index[robotMarkersName[3]]].col((i-t_observe)+1) );


		X_R_ef = sva::PTransformd(curRotLink6, curPosEf);
		X_M_efMarker = sva::PTransformd(curRotLink6, curPosEfMarker);
		X_R_M = X_M_efMarker.inv() * X_R_ef;


		/*subj marker(s) pose w.r.t to robot EF frame*/
		for(int j=1;j<=t_observe; j++)
		{
			X_M_Subj =
			sva::PTransformd(idtMat, markersPos[markers_name_index[subjMarkersName[0]]].col((i-t_observe)+j));

			X_ef_Subj = X_R_ef.inv()*X_M_Subj*X_M_efMarker.inv()*X_R_ef;

			newPosSubj.col(j-1) = X_ef_Subj.translation();
			// cout << "newPosSubj.col(j-1) " << newPosSubj.col(j-1).transpose() <<endl;
			
			if(j==t_observe)
				{ ithPosSubj = newPosSubj.col(t_observe-1); }
		}

		/*get average velocity of previous *t_observe* sec Subj movement*/
		curVelSubj  = handoverTraj->diff(newPosSubj)*fps;//ignore diff > XXXX
		avgVelSubj  << handoverTraj->takeAverage(curVelSubj);
		
		/*predict position in straight line after t_predict time*/
		predictPos = handoverTraj->constVelocityPredictPos(avgVelSubj, ithPosSubj, t_predict);

		/*get predicted way points between left ef and Subj*/	/*** GET NEW VELOCITY PROFILE ****/	
		wp_efL_Subj=handoverTraj->constVelocity(ithPosSubj, predictPos, t_predict);
		
		wp = get<0>(wp_efL_Subj);
		
		initRefPos << wp(0,it), wp(1,it), wp(2,it);


		/*get unit vectors XYZ of subject LEFT hand*/
		auto sizeStr = subjMarkersName.size();
		curPosLshp = markersPos[markers_name_index[subjMarkersName[sizeStr-2]]].col(i);//C
		y = markersPos[markers_name_index[subjMarkersName[sizeStr-1]]].col(i) - curPosLshp;//vCD=Y
		x = markersPos[markers_name_index[subjMarkersName[1]]].col(i) - curPosLshp;//vCA=X

		lshp_X = x/x.norm();
		lshp_Y = y/y.norm();
		lshp_Z = lshp_X.cross(lshp_Y);

		ready = true;
		return std::make_tuple(ready, wp, initRefPos, lshp_Z);
	}



	bool ApproachObject::goToHandoverPose(double min, double max, bool& enableHand, const sva::PTransformd& robotEf, std::shared_ptr<mc_tasks::PositionTask>& posTask, std::shared_ptr<mc_tasks::VectorOrientationTask>& vecOriTask, std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> handPredict)
	{
		Eigen::Vector3d curEfPos, refPos, handoverPos;

		it+= (int)tuner(2);

		curEfPos = robotEf.translation();

		if(it<get<1>(handPredict).cols())
		{
			refPos << get<1>(handPredict)(0,it), get<1>(handPredict)(1,it), get<1>(handPredict)(2,it);
			
			handoverPos = curEfPos + (refPos - get<2>(handPredict));//initRefPos

			 /*robot constraint*/
			if(enableHand &&
				(handoverPos(0)>= 0.10) && (handoverPos(0)<= 0.7) &&
				(handoverPos(1)>= min)  && (handoverPos(1)<= max) &&
				(handoverPos(2)>= 0.90) && (handoverPos(2)<= 1.5)
				)
			{
				/*handover pose*/
				if( objectPos(1)>=-0.15 && objectPos(1)<=0.15)
				{ posTask->position(objectPos); }
				else { posTask->position(handoverPos); }
				
				// vecOriTask->targetVector(get<3>(handPredict));// vecOriTask->targetVector(lshp_Z);
			}
			return true;
		}
		if(it==get<1>(handPredict).cols())
		{
			useLeftEf = false;
			useRightEf = false;
			return false;
		}
	}



	bool ApproachObject::handoverForceController(bool& enableHand, Eigen::Vector3d initPos, Eigen::Vector3d handForce, Eigen::Vector3d Th, std::shared_ptr<mc_tasks::PositionTask>& posTask, std::shared_ptr<mc_tasks::VectorOrientationTask>& vecOriTask, std::string gripperName, std::vector<std::string> robotMarkersName, std::vector<std::string> subjMarkersName)
	{
		Eigen::Vector3d ef_wA_O, ef_wA_wB, ef_wA_gA, ef_wA_gB, ef_wA_f;

		double ef_wAB_theta_wAO;
		double ef_wAB_theta_wAgA;
		double ef_wAB_theta_wAgB;
		double ef_wAB_theta_wAf;

		double ef_area_wAB_O;
		double ef_area_wAB_gA;
		double ef_area_wAB_gB;
		double ef_area_wAB_f;

		double subj_rel_ef;

		double efMass, FNormAtClose;
		std::vector<double> Floadx, Floady, Floadz;
		std::vector<Eigen::Vector3d> efPos, efVel;
		Eigen::Vector3d fingerPos, gripperEf, efAce, Finert, Fzero, Fload, Fpull;

		efPos.resize(3);
		efVel.resize(2);

		fingerPos = markersPos[markers_name_index[subjMarkersName[0]]].col(i);
		
		/*direction vectors, projections and area*/
		ef_wA_O  =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - objectPos;
		ef_wA_f  =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - fingerPos;
		ef_wA_wB =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[robotMarkersName[1]]].col(i);
		ef_wA_gA =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[robotMarkersName[2]]].col(i);
		ef_wA_gB =
		markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[robotMarkersName[3]]].col(i);

		ef_wAB_theta_wAO = acos( ef_wA_wB.dot(ef_wA_O)/( ef_wA_wB.norm()*ef_wA_O.norm() ) );
		ef_wAB_theta_wAf = acos( ef_wA_wB.dot(ef_wA_f)/( ef_wA_wB.norm()*ef_wA_f.norm() ) );
		ef_wAB_theta_wAgA = acos( ef_wA_wB.dot(ef_wA_gA)/( ef_wA_wB.norm()*ef_wA_gA.norm() ) );
		ef_wAB_theta_wAgB = acos( ef_wA_wB.dot(ef_wA_gB)/( ef_wA_wB.norm()*ef_wA_gB.norm() ) );

		ef_area_wAB_O  = 0.5*ef_wA_wB.norm()*ef_wA_O.norm()*sin(ef_wAB_theta_wAO);
		ef_area_wAB_f  = 0.5*ef_wA_wB.norm()*ef_wA_f.norm()*sin(ef_wAB_theta_wAf);
		ef_area_wAB_gA = 0.5*ef_wA_wB.norm()*ef_wA_gA.norm()*sin(ef_wAB_theta_wAgA);
		ef_area_wAB_gB = 0.5*ef_wA_wB.norm()*ef_wA_gB.norm()*sin(ef_wAB_theta_wAgB);


		gripperEf = 0.5*( markersPos[markers_name_index[robotMarkersName[2]]].col(i) + markersPos[markers_name_index[robotMarkersName[3]]].col(i) );

		subj_rel_ef = (gripperEf - fingerPos).norm();


		auto checkForce = [&](const char *axis_name, int idx)
		{
			/*get acceleration of lHand*/
			if(i>3)
			{
				for(int g=1; g<=3; g++)
				{
					efPos[3-g] = 0.25*(
						markersPos[markers_name_index[robotMarkersName[0]]].col(i-g) +
						markersPos[markers_name_index[robotMarkersName[1]]].col(i-g) +
						markersPos[markers_name_index[robotMarkersName[2]]].col(i-g) +
						markersPos[markers_name_index[robotMarkersName[3]]].col(i-g)
						);
				}
				efVel[0] = (efPos[1]-efPos[0])*fps;
				efVel[1] = (efPos[2]-efPos[1])*fps;
				efAce = (efVel[1]-efVel[0])*fps;

				efMass = Fload.norm()/9.8; //HandMass + objMass;
				Finert = efMass*efAce; //inertial Force at ef

				Fpull[0] = abs(handForce[0])-abs(Finert[0]); //-abs(Fzero[0]);
				Fpull[1] = abs(handForce[1])-abs(Finert[1]); //-abs(Fzero[1]);
				Fpull[2] = abs(handForce[2])-abs(Finert[2]); //-abs(Fzero[2]);
			}

			/*new threshold*/
			auto newTh = Fload + Th;

			/* MAY BE check torque too ???? */
			if( (abs(Fpull[idx]) > newTh[idx]) && ( (ef_area_wAB_gA > ef_area_wAB_f) || (ef_area_wAB_gB > ef_area_wAB_f) ) )
			{
				gOpen=true;/*open_gripper();*/
				restartHandover=true;
				takeBackObject=false;

				if(goBackInit)
				{
					goBackInit=false;
					enableHand=false;

					cout << gripperName + "_Forces at Grasp "<< handForce.transpose() <<endl;
					cout << "Finert "<< Finert.transpose() << " object mass " << efMass <<endl;
					LOG_SUCCESS("object returned, threshold on " << axis_name << " with pull forces " << Fpull.transpose()<< " reached on "<< gripperName + " with newTh " << newTh.transpose())
				}
			}
			return false;
		};



		
		if( subj_rel_ef < 0.2 )
		{
			/*open empty gripper when subject come near to robot*/
			if( (!openGripper) && (handForce.norm()<1.0) )
			{
				Fzero = handForce; //this has Finertia too
				gOpen = true;/*open_gripper();*/
				openGripper = true;
				LOG_WARNING("opening " + gripperName<< " with Force Norm "<< handForce.norm())
			}

			/*close gripper*/
			if( (openGripper) && (!closeGripper) && (!restartHandover) && ( (ef_area_wAB_gA > ef_area_wAB_O) || (ef_area_wAB_gB > ef_area_wAB_O) ) )
			{
				gClose = true;/*close_gripper();*/
				closeGripper = true;
				enableHand=false; //when subject hand is very close to efL
			}
			
			/*when closed WITH object*/
			if( graspObject && closeGripper && (handForce.norm()>=2.0) )
			{
				graspObject = false;
				FNormAtClose = handForce.norm();
				LOG_INFO(" object is inside gripper "<< handForce.norm() )

				/*move EF to initial position*/
				posTask->position(initPos);
				vecOriTask->bodyVector({0., 1., 0.});
				vecOriTask->targetVector({0., 1., 0.});
			}

			/*check if object is being pulled*/
			if(takeBackObject)
			{
				return checkForce("x-axis", 0) || checkForce("y-axis", 1) || checkForce("z-axis", 2);
			}
		}


		/*restart handover*/
		if( subj_rel_ef > 0.5 )
		{
			/*here comes only after object is grasped*/
			if( (closeGripper) && (!restartHandover) && (FNormAtClose>=2.0) )
			{
				if(e%200==0)//wait xx sec
				{
					enableHand=true;
					takeBackObject=true;

					Fload <<
					accumulate( Floadx.begin(), Floadx.end(), 0.0)/double(Floadx.size()),
					accumulate( Floady.begin(), Floady.end(), 0.0)/double(Floady.size()),
					accumulate( Floadz.begin(), Floadz.end(), 0.0)/double(Floadz.size());

					// LOG_INFO("ready to grasp again, avg itr size "<< Floadx.size())

					/*clear vector memories*/
					Floadx.clear(); Floady.clear(); Floadz.clear();
				}
				/*divide by 9.8 and you will get object mass*/
				else
				{
					Floadx.push_back( abs( abs(handForce[0])-abs(Fzero[0]) ) );
					Floady.push_back( abs( abs(handForce[1])-abs(Fzero[1]) ) );
					Floadz.push_back( abs( abs(handForce[2])-abs(Fzero[2]) ) );
				}
				e+=1;//cout << "e " << e << endl;
			}

			if(restartHandover)
			{
				/*move EF to initial position*/
				if(!goBackInit)
				{
					posTask->position(initPos);
					vecOriTask->bodyVector({0., 1., 0.});
					vecOriTask->targetVector({0., 1., 0.});
				}

				openGripper=false;
				closeGripper=false;
				restartHandover=false;

				graspObject=true;
				goBackInit=true;
				enableHand=true;
				cout<<"/*******restarting handover*******/"<<endl;
			}
		}
		return false;
	}

}//namespace mc_handover




// if( obj_rel_robotLtHand < obj_rel_subjLtHand || obj_rel_robotRtHand < obj_rel_subjLtHand ||
// 	obj_rel_robotLtHand < obj_rel_subjRtHand || obj_rel_robotRtHand < obj_rel_subjRtHand )
// {
// 	handoverPos = fingerPos;
// }
// else
// {
// 	handoverPos = objectPos;
// }