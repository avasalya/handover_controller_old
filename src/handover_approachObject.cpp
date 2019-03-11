#include "handover_approachObject.h"

namespace mc_handover
{
	ApproachObject::ApproachObject()
	{
		cout << "ApproachObject constructor created " <<endl;
	}

	void ApproachObject::initials() /*may be put all that in the constructor ?????????*/
	{
		/*alllocate memory*/
		predictedPositions.resize(1);
				
		Markers.resize(totalMarkers);
		markersPos.resize(totalMarkers);
		for(int m=0; m<totalMarkers; m++)
			{ markersPos[m] = Eigen::MatrixXd::Zero(3,60000); }

		efPos.resize(3);
		efVel.resize(2);

		X_ef_S.resize(t_observe);
		newPosSubj = Eigen::MatrixXd::Zero(3, t_observe);
		

		/*prediction controller parameter*/
		tuner << 400., 20., 60.;
		t_predict = (int)tuner(0);
		t_observe = (int)tuner(1);
		it = (int)tuner(2);


		std::vector<std::string> robotLtMarkers = {"wristLtEfA", "wristLtEfB", "gripperLtEfA", "gripperLtEfB"};
		std::vector<std::string> robotRtMarkers = {"wristRtEfA", "wristRtEfB", "gripperRtEfA", "gripperRtEfB"};

		std::vector<std::string> lShapeLtMarkers = {"fingerSubjLt", "lShapeLtA", "lShapeLtB", "lShapeLtC", "lShapeLtD"};
		std::vector<std::string> lShapeRtMarkers = {"fingerSubjRt", "lShapeRtA", "lShapeRtB", "lShapeRtC", "lShapeRtD"};

		strMarkers.insert(strMarkers.begin(), robotLtMarkers.begin(), robotLtMarkers.end());
		strMarkers.insert(strMarkers.end(), robotRtMarkers.begin(), robotRtMarkers.end());
		strMarkers.insert(strMarkers.end(), lShapeLtMarkers.begin(), lShapeLtMarkers.end());
		strMarkers.insert(strMarkers.end(), lShapeRtMarkers.begin(), lShapeRtMarkers.end());
	}


	/*get markers position FrameByFrame*/
	bool ApproachObject::checkFrameOfData(std::vector<Eigen::Vector3d>)
	{
		bool checkNonZero;

		if(Flag_withoutRobot)
		{
			Markers[wristLtEfA] << 0.208955, 0.350982, 0.552377;
			Markers[wristLtEfB] << 0.15638, 0.352496, 0.547814;
			Markers[gripperLtEfA] << 0.217815, 0.334505, 0.432962;
			Markers[gripperLtEfB] << 0.162401,  0.33451, 0.42898;

			Markers[wristRtEfA] << 0.14048, -0.309184, 0.550067;
			Markers[wristRtEfB] << 0.198771, -0.308889,  0.555116;
			Markers[gripperRtEfA] << 0.148997, -0.29622, 0.418868;
			Markers[gripperRtEfB] << 0.202506, -0.293441, 0.425241;
		}

		for(int i=0; i<Markers.size(); i++)
		{
			if( Markers[i](0)>-10 && Markers[i](0)!=0 && Markers[i](0)<10 )
				{ checkNonzero = true; }
			else
				{ checkNonzero = false; }
		}
		return checkNonZero;
	}




	bool ApproachObject::handoverPresets()
	{
		/*check for non zero frame only and store them*/
		if( checkFrameOfData(Markers) )
		{
			i+=1;
			LOG_INFO("i " << i)
			for(int m=0; m<totalMarkers; m++)
				{ markersPos[m].col(i) << Markers[m]; }

			/*for GUI*/
			objectPos = markersPos[object].col(i);

			/*direction vectors, projections and area*/
			lEf_wA_O  = markersPos[wristLtEfA].col(i)-objectPos;
			lEf_wA_wB = markersPos[wristLtEfA].col(i)-markersPos[wristLtEfB].col(i);
			lEf_wA_gA = markersPos[wristLtEfA].col(i)-markersPos[gripperLtEfA].col(i);
			lEf_wA_gB = markersPos[wristLtEfA].col(i)-markersPos[gripperLtEfB].col(i);
			lEf_wA_lf = markersPos[wristLtEfA].col(i)-markersPos[fingerSubjLt].col(i);

			lEf_wAB_theta_wAO = acos( lEf_wA_wB.dot(lEf_wA_O)/( lEf_wA_wB.norm()*lEf_wA_O.norm() ) );
			lEf_wAB_theta_wAgA = acos( lEf_wA_wB.dot(lEf_wA_gA)/( lEf_wA_wB.norm()*lEf_wA_gA.norm() ) );
			lEf_wAB_theta_wAgB = acos( lEf_wA_wB.dot(lEf_wA_gB)/( lEf_wA_wB.norm()*lEf_wA_gB.norm() ) );
			lEf_wAB_theta_wAf = acos( lEf_wA_wB.dot(lEf_wA_lf)/( lEf_wA_wB.norm()*lEf_wA_lf.norm() ) );

			lEf_area_wAB_O  = 0.5*lEf_wA_wB.norm()*lEf_wA_O.norm()*sin(lEf_wAB_theta_wAO);
			lEf_area_wAB_gA = 0.5*lEf_wA_wB.norm()*lEf_wA_gA.norm()*sin(lEf_wAB_theta_wAgA);
			lEf_area_wAB_gB = 0.5*lEf_wA_wB.norm()*lEf_wA_gB.norm()*sin(lEf_wAB_theta_wAgB);
			lEf_area_wAB_f  = 0.5*lEf_wA_wB.norm()*lEf_wA_lf.norm()*sin(lEf_wAB_theta_wAf);

			
			rEf_wA_O  = markersPos[wristRtEfA].col(i)-objectPos;
			rEf_wA_wB = markersPos[wristRtEfA].col(i)-markersPos[wristRtEfB].col(i);
			rEf_wA_gA = markersPos[wristRtEfA].col(i)-markersPos[gripperRtEfA].col(i);
			rEf_wA_gB = markersPos[wristRtEfA].col(i)-markersPos[gripperRtEfB].col(i);
			rEf_wA_lf  = markersPos[wristRtEfA].col(i)-markersPos[fingerSubjRt].col(i);

			rEf_wAB_theta_wAO = acos( rEf_wA_wB.dot(rEf_wA_O)/( rEf_wA_wB.norm()*rEf_wA_O.norm() ) );
			rEf_wAB_theta_wAgA = acos( rEf_wA_wB.dot(rEf_wA_gA)/( rEf_wA_wB.norm()*rEf_wA_gA.norm() ) );
			rEf_wAB_theta_wAgB = acos( rEf_wA_wB.dot(rEf_wA_gB)/( rEf_wA_wB.norm()*rEf_wA_gB.norm() ) );
			rEf_wAB_theta_wAf = acos( rEf_wA_wB.dot(rEf_wA_lf)/( rEf_wA_wB.norm()*rEf_wA_lf.norm() ) );

			rEf_area_wAB_O  = 0.5*rEf_wA_wB.norm()*rEf_wA_O.norm()*sin(rEf_wAB_theta_wAO);
			rEf_area_wAB_gA = 0.5*rEf_wA_wB.norm()*rEf_wA_gA.norm()*sin(rEf_wAB_theta_wAgA);
			rEf_area_wAB_gB = 0.5*rEf_wA_wB.norm()*rEf_wA_gB.norm()*sin(rEf_wAB_theta_wAgB);
			rEf_area_wAB_f  = 0.5*rEf_wA_wB.norm()*rEf_wA_lf.norm()*sin(rEf_wAB_theta_wAf);


			/*move EF when subject approaches object 1st time*/
			obj_rel_subjLtHand = ( markersPos[fingerSubjLt].col(i) - objectPos ).norm();
			obj_rel_subjRtHand = ( markersPos[fingerSubjRt].col(i) - objectPos ).norm();


			/*get robot ef marker(s) current pose*/
			efLGripperPos = 0.25*( 
				markersPos[wristLtEfA].col((i-t_observe)+1) + markersPos[wristLtEfB].col((i-t_observe)+1) +
				markersPos[gripperLtEfA].col((i-t_observe)+1) + markersPos[gripperLtEfB].col((i-t_observe)+1) );

			efRGripperPos = 0.25*( 
				markersPos[wristRtEfA].col((i-t_observe)+1) + markersPos[wristRtEfB].col((i-t_observe)+1) +
				markersPos[gripperRtEfA].col((i-t_observe)+1) + markersPos[gripperRtEfB].col((i-t_observe)+1) );

			return true;
		}
		else
		{ return false; }
	}



	bool ApproachObject::predictionController(const Eigen::Vector3d& efGripperPos, double subjHandReady, const sva::PTransformd& robotEf, const Eigen::Matrix3d & curRotLink6)
	{
		/*observe subject motion for t_observe period*/
		if( (i%t_observe==0) )
		{
			LOG_ERROR("i " << i)

			/*prediction_ tuner*/
			t_predict = (int)tuner(0);
			t_observe = (int)tuner(1);
			it = (int)tuner(2);

			/*get robot ef current pose*/
			curRotEf = robotEf.rotation();
			curPosEf = robotEf.translation();
			
			curPosEfMarker << efGripperPos;

			X_R_ef = sva::PTransformd(curRotLink6, curPosEf);
			X_M_efMarker = sva::PTransformd(curRotLink6, curPosEfMarker);
			X_R_M = X_M_efMarker.inv() * X_R_ef;
			X_R_ef_const = sva::PTransformd(q1l.toRotationMatrix(), p_l);



/*)))))))))))))))))))))))))))) continue from below (((((((((((((((((((((((((((*/




			/*check subj hand's relative orientation*/
			if(!subjHandReady)
			{
				// sva::PTransformd BodyW = robot().mbc().bodyPosW[robot().bodyIndexByName("BODY")];
				// BodyW.rotation();

				curPosLshp = markersPos[lShapeLtC].col(i);

				/*get unit vectors XYZ of subject LEFT hand*/
				x = markersPos[lShapeLtA].col(i) - markersPos[lShapeLtC].col(i);//vCA=X
				y = markersPos[lShapeLtD].col(i) - markersPos[lShapeLtC].col(i);//vCD=Y

				lshp_X = x/x.norm();
				lshp_Y = y/y.norm();
				lshp_Z = lshp_X.cross(lshp_Y);//X.cross(Y)=Z

				subjHandRot.col(0) = lshp_X;
				subjHandRot.col(1) = lshp_Y;
				
				/*convert to 2D rotation method*/
				subjHandRot.row(2) = lshp_Z/lshp_Z.norm();
				// LOG_ERROR(subjHandRot<<"\n\n")

				X_M_lshp = sva::PTransformd(subjHandRot, curPosLshp);
				
				X_e_l =  X_R_ef_const.inv() * X_M_lshp;// * X_R_M;
				// LOG_SUCCESS(X_e_l.rotation()<<"\n")

				handoverRot = X_e_l.rotation();// * idt;
			}

			/*subj marker(s) pose w.r.t to robot EF frame*/
			for(int j=1;j<=t_observe; j++)
			{
				// X_M_Subj = sva::PTransformd(idtMat, markersPos[fingerSubjLt].middleCols((i-t_observe)+j,i));
				X_M_Subj = sva::PTransformd(handoverRot, markersPos[fingerSubjLt].middleCols((i-t_observe)+j,i))
				X_ef_Subj = X_R_ef.inv()*X_M_Subj*X_M_efMarker.inv()*X_R_ef;

				X_ef_S[j-1] = X_ef_Subj;

				newPosSubj.col(j-1) = X_ef_Subj.translation();
				
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
	}

}//namespace mc_handover