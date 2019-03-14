#include "handover_approachObject.h"

namespace mc_handover
{
	ApproachObject::ApproachObject()
	{
		cout<<"constructor created " <<endl;
	}
	



	/*allocate memory*/
	void ApproachObject::initials()
	{
		/*markers Name strings*/
		robotLtMarkers = {"wristLtEfA", "wristLtEfB", "gripperLtEfA", "gripperLtEfB"};
		robotRtMarkers = {"wristRtEfA", "wristRtEfB", "gripperRtEfA", "gripperRtEfB"};

		lShapeLtMarkers = {"fingerSubjLt", "lShapeLtA", "lShapeLtB", "lShapeLtC", "lShapeLtD"};
		lShapeRtMarkers = {"fingerSubjRt", "lShapeRtA", "lShapeRtB", "lShapeRtC", "lShapeRtD"};

		strMarkersName.insert(strMarkersName.begin(), robotLtMarkers.begin(), robotLtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), robotRtMarkers.begin(), robotRtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), "object");
		strMarkersName.insert(strMarkersName.end(), lShapeLtMarkers.begin(), lShapeLtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), lShapeRtMarkers.begin(), lShapeRtMarkers.end());


		for(unsigned int i=0; i<strMarkersName.size(); i++)
			markers_name_index[strMarkersName[i]] = i;


		predictedPositions.resize(1);

		Markers.resize(totalMarkers);
		markersPos.resize(totalMarkers);
		for(int m=0; m<totalMarkers; m++)
			{ markersPos[m] = Eigen::MatrixXd::Zero(3,60000); }

		efPos.resize(3);
		efVel.resize(2);


		/*prediction controller parameter*/
		tuner << 400., 20., 60.;
	
		t_predict = (int)tuner(0);
		t_observe = (int)tuner(1);
		it = (int)tuner(2);

		X_ef_S.resize(t_observe);
		newPosSubj = Eigen::MatrixXd::Zero(3, t_observe);

		LOG_ERROR("***OK so far***")
	}




	
	// bool ApproachObject::checkFrameOfData(std::vector<Eigen::Vector3d>)
	// {
	// 	bool checkNonZero{false};

	// 	if(Flag_withoutRobot)
	// 	{
	// 		Markers[wristLtEfA] << 0.208955, 0.350982, 0.552377;
	// 		Markers[wristLtEfB] << 0.15638, 0.352496, 0.547814;
	// 		Markers[gripperLtEfA] << 0.217815, 0.334505, 0.432962;
	// 		Markers[gripperLtEfB] << 0.162401,  0.33451, 0.42898;

	// 		Markers[wristRtEfA] << 0.14048, -0.309184, 0.550067;
	// 		Markers[wristRtEfB] << 0.198771, -0.308889,  0.555116;
	// 		Markers[gripperRtEfA] << 0.148997, -0.29622, 0.418868;
	// 		Markers[gripperRtEfB] << 0.202506, -0.293441, 0.425241;
	// 	}

	// 	for(unsigned int i=0; i<strMarkersName.size(); i++)
	// 	{
	// 		//LOG_WARNING(<<i<<" "<<strMarkersName[i]<<" "<< Markers[ markers_name_index[ strMarkersName[i] ] ].transpose())

	// 		if( Markers[i](0)>-10 && Markers[i](0)!=0 && Markers[i](0)<10 )
	// 			{ checkNonZero = true; }
	// 		else
	// 		{ return false; }
	// 	}
	// 	return checkNonZero;
	// }


	
	// bool ApproachObject::handoverRun()
	// {
	// 	/*check for non zero frame only and store them*/
	// 	if( checkFrameOfData(Markers) )
	// 	{
	// 		for(int m=0; m<totalMarkers; m++)
	// 			{ markersPos[m].col(i) << Markers[m]; }

	// 		/*for GUI*/
	// 		objectPos = markersPos[object].col(i);

	// 		/*move EF when subject approaches object 1st time*/
	// 		obj_rel_subjLtHand = ( markersPos[fingerSubjLt].col(i) - objectPos ).norm();
	// 		obj_rel_subjRtHand = ( markersPos[fingerSubjRt].col(i) - objectPos ).norm();

	// 		i+=1;
	// 		LOG_INFO("i " << i)

	// 		return true;
	// 	}
	// 	else
	// 	{ return false; }
	// }


	
	// bool ApproachObject::predictionController(Eigen::Vector3d p, Eigen::Quaterniond q, double subjHandReady, const sva::PTransformd& robotEf, const Eigen::Matrix3d & curRotLink6, std::vector<std::string> lShpMarkersName, std::vector<std::string> robotMarkersName)
	// {
	// 	LOG_ERROR("i " << i)

	// 	/*prediction_ tuner*/
	// 	t_predict = (int)tuner(0);
	// 	t_observe = (int)tuner(1);
	// 	it = (int)tuner(2);


	// 	/*get robot ef current pose*/
	// 	curRotEf = robotEf.rotation();
	// 	curPosEf = robotEf.translation();


	// 	/*get robot ef marker(s) current pose*/
	// 	curPosEfMarker << 0.25*( 
	// 		markersPos[markers_name_index[robotMarkersName[0]]].col((i-t_observe)+1) +
	// 		markersPos[markers_name_index[robotMarkersName[1]]].col((i-t_observe)+1) +
	// 		markersPos[markers_name_index[robotMarkersName[2]]].col((i-t_observe)+1) + 
	// 		markersPos[markers_name_index[robotMarkersName[3]]].col((i-t_observe)+1) );


	// 	X_R_ef = sva::PTransformd(curRotLink6, curPosEf);
	// 	X_M_efMarker = sva::PTransformd(curRotLink6, curPosEfMarker);
	// 	X_R_M = X_M_efMarker.inv() * X_R_ef;
	// 	X_R_ef_const = sva::PTransformd(q.toRotationMatrix(), p);

	// 	/*check subj hand's relative orientation*/
	// 	if(!subjHandReady)
	// 	{
	// 		LOG_ERROR("markersPos[lShapeLtC] " << markersPos[lShapeLtC].col(i))
	// 		curPosLshp = markersPos[markers_name_index[lShpMarkersName[3]]].col(i); //markersPos[lShapeLtC].col(i);
	// 		LOG_INFO("curPosLshp " << curPosLshp)

	// 		/*get unit vectors XYZ of subject LEFT hand*/
	// 		x = markersPos[markers_name_index[lShpMarkersName[1]]].col(i) - curPosLshp;//vCA=X
	// 		y = markersPos[markers_name_index[lShpMarkersName[4]]].col(i) - curPosLshp;//vCD=Y

	// 		lshp_X = x/x.norm();
	// 		lshp_Y = y/y.norm();
	// 		lshp_Z = lshp_X.cross(lshp_Y);//X.cross(Y)=Z

	// 		subjHandRot.col(0) = lshp_X;
	// 		subjHandRot.col(1) = lshp_Y;
			
	// 		/*convert to 2D rotation method*/
	// 		subjHandRot.row(2) = lshp_Z/lshp_Z.norm();
	// 		// LOG_ERROR(subjHandRot<<"\n\n")

	// 		X_M_lshp = sva::PTransformd(subjHandRot, curPosLshp);
			
	// 		X_e_l =  X_R_ef_const.inv() * X_M_lshp;// * X_R_M;
	// 		// LOG_SUCCESS(X_e_l.rotation()<<"\n")

	// 		// sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];
	// 		// BodyW.rotation();

	// 		handoverRot = X_e_l.rotation();// * idtMat;
	// 		// handoverRot = X_e_l.rotation()*BodyW.rotation();
	// 	}

	// 	/*subj marker(s) pose w.r.t to robot EF frame*/
	// 	for(int j=1;j<=t_observe; j++)
	// 	{
	// 		// X_M_Subj =
	// 		// sva::PTransformd(idtMat, markersPos[markers_name_index[lShpMarkersName[0]]].middleCols((i-t_observe)+j,i));
			
	// 		X_M_Subj =
	// 		sva::PTransformd(handoverRot, markersPos[markers_name_index[lShpMarkersName[0]]].middleCols((i-t_observe)+j,i));
			
	// 		X_ef_Subj = X_R_ef.inv()*X_M_Subj*X_M_efMarker.inv()*X_R_ef;

	// 		X_ef_S[j-1] = X_ef_Subj;

	// 		newPosSubj.col(j-1) = X_ef_Subj.translation();
			
	// 		if(j==t_observe)
	// 			{ ithPosSubj = newPosSubj.col(t_observe-1); }
	// 	}

	// 	/*get average velocity of previous *t_observe* sec Subj motion*/
	// 	curVelSubj  = handoverTraj->diff(newPosSubj)*fps;//ignore diff > XXXX
	// 	avgVelSubj  << handoverTraj->takeAverage(curVelSubj);

	// 	/*predict position in straight line after t_predict time*/
	// 	predictPos = handoverTraj->constVelocityPredictPos(avgVelSubj, ithPosSubj, t_predict);
	// 	predictedPositions[0] << predictPos;
	// 	// cout << "predicted pos " << predictPos.transpose()<<endl;

	// 	/*get predicted way points between left ef and Subj*/	/*** GET NEW VELOCITY PROFILE ****/
	// 	wp_efL_Subj=handoverTraj->constVelocity(ithPosSubj, predictPos, t_predict);
	// 	wp = get<0>(wp_efL_Subj);

	// 	initRefPos << wp(0,it), wp(1,it), wp(2,it);

	// 	return true;
	// }



	// void ApproachObject::goToHandoverPose(const sva::PTransformd& robotEf, const std::shared_ptr<mc_tasks::OrientationTask>& oriTask, const std::shared_ptr<mc_tasks::PositionTask>& posTask)
	// {
	// 	it+= (int)tuner(2);//40+(int)t_predict/t_observe;

	// 	auto curEfPos = robotEf.translation();

	// 	if(it<=wp.cols())
	// 	{
	// 		refPos << wp(0,it), wp(1,it), wp(2,it);
	// 		handoverPos = curEfPos + refPos - initRefPos;
			
	// 		// idtMat << RotX(90*DegToRad) * RotY(90*DegToRad) * RotZ(90*DegToRad);
	// 		// handoverRot = idtMat.transpose()*subjLtHandRot.transpose();

	// 		 /*robot constraint*/
	// 		if(motion &&
	// 			(handoverPos(0)>= 0.20) && (handoverPos(0)<= 0.7) && 
	// 			(handoverPos(2)>= 0.90) && (handoverPos(2)<= 1.5)
	// 			)
	// 		{
	// 			/*head pos*/
	// 			if(	(handoverPos(1)>= 0.00) && (handoverPos(1)<= 0.7) )
	// 			{
	// 				if(handoverPos(1) >.45){controller_->set_joint_pos("HEAD_JOINT0",  0.8);} //y //+ve to move head left
	// 				else{controller_->set_joint_pos("HEAD_JOINT0",  0.); }//-ve to move head right
	// 			}
	// 			else if( (handoverPos(1)<= 0.00) && (handoverPos(1)>= -0.7) )
	// 			{
	// 				if(handoverPos(1) <-.45){controller_->set_joint_pos("HEAD_JOINT0",  -0.4);} //y //+ve to move head left
	// 				else{controller_->set_joint_pos("HEAD_JOINT0",  0.); }//-ve to move head right
	// 			}
	// 			if(handoverPos(2) <1.1){controller_->set_joint_pos("HEAD_JOINT1",  0.6);} //z //+ve to move head down
	// 			else{controller_->set_joint_pos("HEAD_JOINT1",  -0.4);} //-ve to move head up

	// 			/*handover pose*/
	// 			sva::PTransformd new_pose(handoverRot, handoverPos);
	// 			oriTask->orientation(new_pose.rotation());
	// 			posTask->position(new_pose.translation());
	// 		}
	// 		if(it==wp.cols())
	// 			{ collected  = false; LOG_ERROR("collected "<< collected)}
	// 	}
	// }



	// bool ApproachObject::handoverForceController(Eigen::Vector3d handForce, Eigen::Vector3d Th, std::string gripperName, std::vector<std::string> lShpMarkersName, std::vector<std::string> robotMarkersName)
	// {
	// 	// robotLtMarkers = {"wristLtEfA", "wristLtEfB", "gripperLtEfA", "gripperLtEfB"};
	// 	// lShapeLtMarkers = {"fingerSubjLt", "lShapeLtA", "lShapeLtB", "lShapeLtC", "lShapeLtD"};
	// 	// markersPos[markers_name_index[robotMarkersName[0]]]

	// 	/*direction vectors, projections and area*/
	// 	ef_wA_O  = markersPos[markers_name_index[robotMarkersName[0]]].col(i) - objectPos;
	// 	ef_wA_wB = markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[robotMarkersName[1]]].col(i);
	// 	ef_wA_gA = markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[robotMarkersName[2]]].col(i);
	// 	ef_wA_gB = markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[robotMarkersName[3]]].col(i);
	// 	ef_wA_lf = markersPos[markers_name_index[robotMarkersName[0]]].col(i) - markersPos[markers_name_index[lShpMarkersName[0]]].col(i);

	// 	ef_wAB_theta_wAO = acos( ef_wA_wB.dot(ef_wA_O)/( ef_wA_wB.norm()*ef_wA_O.norm() ) );
	// 	ef_wAB_theta_wAgA = acos( ef_wA_wB.dot(ef_wA_gA)/( ef_wA_wB.norm()*ef_wA_gA.norm() ) );
	// 	ef_wAB_theta_wAgB = acos( ef_wA_wB.dot(ef_wA_gB)/( ef_wA_wB.norm()*ef_wA_gB.norm() ) );
	// 	ef_wAB_theta_wAf = acos( ef_wA_wB.dot(ef_wA_lf)/( ef_wA_wB.norm()*ef_wA_lf.norm() ) );

	// 	ef_area_wAB_O  = 0.5*ef_wA_wB.norm()*ef_wA_O.norm()*sin(ef_wAB_theta_wAO);
	// 	ef_area_wAB_gA = 0.5*ef_wA_wB.norm()*ef_wA_gA.norm()*sin(ef_wAB_theta_wAgA);
	// 	ef_area_wAB_gB = 0.5*ef_wA_wB.norm()*ef_wA_gB.norm()*sin(ef_wAB_theta_wAgB);
	// 	ef_area_wAB_f  = 0.5*ef_wA_wB.norm()*ef_wA_lf.norm()*sin(ef_wAB_theta_wAf);


	// 	/*gripper control*/
	// 	auto close_gripper = [&]()
	// 	{
	// 		auto gripper = controller_->grippers[gripperName].get();
	// 		gripper->setTargetQ({closeGrippers});
	// 	};

	// 	auto open_gripper = [&]()
	// 	{
	// 		auto gripper = controller_->grippers[gripperName].get();
	// 		gripper->setTargetQ({openGrippers});
	// 	};


	// 	auto checkForce = [&](const char *axis_name, int idx)
	// 	{
	// 		/*get acceleration of lHand*/
	// 		if(i>3)
	// 		{
	// 			for(int g=1; g<=3; g++)
	// 			{
	// 				efPos[3-g] = 0.25*(
	// 					markersPos[markers_name_index[robotMarkersName[0]]].col(i-g) +
	// 					markersPos[markers_name_index[robotMarkersName[1]]].col(i-g) +
	// 					markersPos[markers_name_index[robotMarkersName[2]]].col(i-g) +
	// 					markersPos[markers_name_index[robotMarkersName[3]]].col(i-g)
	// 					);
	// 			}
	// 			efVel[0] = (efPos[1]-efPos[0])*fps;
	// 			efVel[1] = (efPos[2]-efPos[1])*fps;
	// 			efAce = (efVel[1]-efVel[0])*fps;

	// 			efMass = Fload.norm()/9.8; //lHandMass + objMass;
	// 			Finert = efMass*efAce; //inertial Force at efL

	// 			Fpull[0] = abs(handForce[0])-abs(Finert[0]); //-abs(Fzero[0]);
	// 			Fpull[1] = abs(handForce[1])-abs(Finert[1]); //-abs(Fzero[1]);
	// 			Fpull[2] = abs(handForce[2])-abs(Finert[2]); //-abs(Fzero[2]);
	// 		}

	// 		/*new threshold*/
	// 		auto newTh = Fload + Th;

	// 		/* MAY BE check torque too ???? */
	// 		if( (abs(Fpull[idx]) > newTh[idx]) && ( (ef_area_wAB_gA > ef_area_wAB_f) || (ef_area_wAB_gB > ef_area_wAB_f) ) )
	// 		{
	// 			open_gripper();
	// 			restartHandover=true;
	// 			readyToGrasp=false;
	// 			dum1=false;
	// 			if(dum3)
	// 			{
	// 				dum3=false;
	// 				cout << gripperName + "_Forces at Grasp "<< handForce.transpose() <<endl;
	// 				cout << "Finert "<< Finert.transpose() << " object mass " << efMass <<endl;
	// 				LOG_SUCCESS("object returned, threshold on " << axis_name << " with pull forces " << Fpull.transpose()<< " reached on "<< gripperName + " with newTh " << newTh.transpose())
	// 			}
	// 		}
	// 		return false;
	// 	};


	// 	auto  gripperEf = 0.5*( markersPos[markers_name_index[robotMarkersName[2]]].col(i) + markersPos[markers_name_index[robotMarkersName[3]]].col(i) );
					
		
	// 	if( ( (gripperEf - markersPos[markers_name_index[lShpMarkersName[0]]].col(i) ).norm() <0.2 ) )
	// 	{
	// 		/*open empty gripper when subject come near to robot*/
	// 		if( (!openGripper) && (handForce.norm()<1.0) )
	// 		{
	// 			Fzero = handForce; //this has Finertia too
	// 			open_gripper();
	// 			LOG_WARNING("opening " + gripperName<< " with Force Norm "<< handForce.norm())
	// 			openGripper = true;
	// 		}

	// 		/*close gripper*/
	// 		if( (openGripper) && (!closeGripper) && (!restartHandover) && ( (ef_area_wAB_gA > ef_area_wAB_O) || (ef_area_wAB_gB > ef_area_wAB_O) ) )
	// 		{
	// 			close_gripper();
	// 			motion=false; //when subject hand is very close to efL
	// 			closeGripper = true;
	// 		}
			
	// 		/*when closed WITH object*/
	// 		if( dum2 && closeGripper && (handForce.norm()>=2.0) )
	// 		{
	// 			FNormAtClose = handForce.norm();
	// 			LOG_INFO(" object is inside gripper "<< handForce.norm() )
	// 			dum2 = false;
	// 		}

	// 		/*check if object is being pulled*/
	// 		if(readyToGrasp)
	// 		{
	// 			return checkForce("x-axis", 0) || checkForce("y-axis", 1) || checkForce("z-axis", 2);
	// 		}
	// 	}


	// 	/*restart handover*/
	// 	if( (gripperEf - markersPos[markers_name_index[lShpMarkersName[0]]].col(i)).norm() > 0.50 )
	// 	{
	// 		/*here comes only after object is grasped*/
	// 		if( (closeGripper) && (!restartHandover) && (FNormAtClose>=2.0) )
	// 		{
	// 			if(e%200==0)//wait xx sec
	// 			{
	// 				motion=true;
	// 				readyToGrasp=true;

	// 				Fload <<
	// 				accumulate( Floadx.begin(), Floadx.end(), 0.0)/double(Floadx.size()),
	// 				accumulate( Floady.begin(), Floady.end(), 0.0)/double(Floady.size()),
	// 				accumulate( Floadz.begin(), Floadz.end(), 0.0)/double(Floadz.size());

	// 				// LOG_INFO("ready to grasp again, avg itr size "<< Floadx.size())

	// 				/*clear vector memories*/
	// 				Floadx.clear(); Floady.clear(); Floadz.clear();
	// 			}
	// 			/*divide by 9.8 and you will get object mass*/
	// 			else
	// 			{
	// 				Floadx.push_back( abs( abs(handForce[0])-abs(Fzero[0]) ) );
	// 				Floady.push_back( abs( abs(handForce[1])-abs(Fzero[1]) ) );
	// 				Floadz.push_back( abs( abs(handForce[2])-abs(Fzero[2]) ) );
	// 			}
	// 			e+=1;//cout << "e " << e << endl;
	// 		}

	// 		if(restartHandover)
	// 		{
	// 			restartHandover=false;
	// 			openGripper=false;
	// 			closeGripper=false;

	// 			dum1=true;
	// 			dum2=true;
	// 			dum3=true;
	// 			cout<<"/*******restarting handover*******/"<<endl;
	// 		}
	// 	}
	// 	return false;
	// }



}//namespace mc_handover