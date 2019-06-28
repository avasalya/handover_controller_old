#include "handover_approachObject.h"

namespace mc_handover
{
	ApproachObject::ApproachObject()
	{ cout<<"\033[1;50mhandover object created\033[0m\n"; }



	ApproachObject::~ApproachObject()
	{ cout<<"\033[1;50mhandover object destroyed\033[0m\n"; }



	/*allocate memory*/
	void ApproachObject::initials()
	{
		/*markers Name strings*/
		strMarkersBodyName = {"4mars_robot_left_hand", "4mars_robot_right_hand", "5mars_obj", "7mars_subj_hands"};

		robotLtMarkers = {"wristLtEfA", "wristLtEfB", "gripperLtEfA", "gripperLtEfB"};//0-3 + dummy
		robotRtMarkers = {"wristRtEfA", "wristRtEfB", "gripperRtEfA", "gripperRtEfB"};//4-7

		objMarkers = {"left", "center", "right", "centerX", "centerY"};//8-12

		subjRtMarkers = {"lShapeRtA", "lShapeRtB", "lShapeRtC", "lShapeRtD"};//13-16
		subjLtMarkers = {"lShapeLtA",              "lShapeLtC", "lShapeLtD"};//17-19
		subjMarkers = {"lShapeRtA", "lShapeRtB", "lShapeRtC", "lShapeRtD", "lShapeLtA", "lShapeLtC", "lShapeLtD"}; //13-19

		strMarkersName.insert(strMarkersName.begin(), robotLtMarkers.begin(), robotLtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), robotRtMarkers.begin(), robotRtMarkers.end());
		strMarkersName.insert(strMarkersName.end(), objMarkers.begin(), objMarkers.end());
		strMarkersName.insert(strMarkersName.end(), subjMarkers.begin(), subjMarkers.end());

		totalMarkers = strMarkersName.size();

		for(unsigned int k=0; k<totalMarkers; k++)
			markers_name_index[strMarkersName[k]] = k;

		Markers.resize(totalMarkers);
		object.resize(objMarkers.size());

		markersPos.resize(totalMarkers);
		for(int m=0; m<totalMarkers; m++)
			{ markersPos[m] = Eigen::MatrixXd::Zero(3,600000); }

		if(Flag_withoutRobot)
		{	LOG_ERROR("robot markers are not considered") }
		else
		{	LOG_WARNING("robot markers are considered")	}


		/*prediction controller parameter*/
		tuner << 100., 10., 10.;
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


		for(unsigned int k=8; k<totalMarkers; k++)
		{
			// LOG_WARNING(k<<" "<<strMarkersName[k]<<" "<< Markers[ markers_name_index[ strMarkersName[k] ] ].transpose())
			if( Markers[k](0)>-10 && Markers[k](0)!=0 && Markers[k](0)<10 )
				{ checkNonZero = true; }
			else
			{ return false; }
		}
		return checkNonZero;
	}



	bool ApproachObject::handoverRun()
	{
		Eigen::Vector3d xl, yl, lshp_Xl, lshp_Yl, lshp_Zl;
		Eigen::Vector3d xr, yr, lshp_Xr, lshp_Yr, lshp_Zr;

		Eigen::Vector3d xo, yo, lshp_Xo, lshp_Yo, lshp_Zo;

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

			objectPosL = object[0];
			objectPosC = object[1];
			objectPosR = object[2];
			objectPosCx = markersPos[11].col(i);//centerX
			objectPosCy = markersPos[12].col(i);//centerY

			fingerPosR = markersPos[13].col(i); //lShapeRtA
			fingerPosL = markersPos[17].col(i); //lShapeLtA

			gripperLtEfA = markersPos[2].col(i); //gripperLtEfA
			gripperLtEfB = markersPos[3].col(i); //gripperLtEfB

			gripperRtEfA = markersPos[6].col(i); //gripperRtEfA
			gripperRtEfB = markersPos[7].col(i); //gripperRtEfB

			gripperEfL = 0.5*( gripperLtEfA + gripperLtEfB );
			gripperEfR = 0.5*( gripperRtEfA + gripperRtEfB );

			/*move EF when subject approaches object 1st time*/
			obj_rel_robotRtHand = ( gripperEfR - object[0] ).norm();//gripperRtEfA - objLeft
			obj_rel_robotLtHand = ( gripperEfL - object[2] ).norm();//gripperLtEfA - objRight

			obj_rel_subjLtHand = ( fingerPosL - object[0] ).norm();//lshpLtA - objLeft
			obj_rel_subjRtHand = ( fingerPosR - object[2] ).norm();//lshpRtA - objRight

			/*right hand orientation*/
			yr = markersPos[16].col(i) - markersPos[15].col(i);//vCD=Yr
			xr = markersPos[15].col(i) - markersPos[13].col(i);//vAC=Xr

			lshp_Xr = xr/xr.norm();
			lshp_Yr = yr/yr.norm();
			lshp_Zr = lshp_Xr.cross(lshp_Yr);

			subjRHandRot.col(0) = lshp_Xr;
			subjRHandRot.col(1) = lshp_Yr;
			subjRHandRot.col(2) = lshp_Zr/lshp_Zr.norm();


			/*left hand orientation*/
			yl = markersPos[19].col(i) - markersPos[18].col(i);//vCD=Yl
			xl = markersPos[18].col(i) - markersPos[17].col(i);//vAC=Xl

			lshp_Xl = xl/xl.norm();
			lshp_Yl = yl/yl.norm();
			lshp_Zl = lshp_Xl.cross(lshp_Yl);

			subjLHandRot.col(0) = lshp_Xl;
			subjLHandRot.col(1) = lshp_Yl;
			subjLHandRot.col(2) = lshp_Zl/lshp_Zl.norm();


			/*object orientation*/
			yo = objectPosCy - objectPosC;//vCCy=yo
			xo = objectPosC - objectPosCx;//vCxC=xo

			lshp_Xo = xo/xo.norm();
			lshp_Yo = yo/yo.norm();
			lshp_Zo = lshp_Xo.cross(lshp_Yo);

			objRot.col(0) = lshp_Xo;
			objRot.col(1) = lshp_Yo;
			objRot.col(2) = lshp_Zo/lshp_Zo.norm();

			return true;
		}
		else
		{ return false; }
	}



	std::tuple<bool, Eigen::MatrixXd, Eigen::Vector3d, Eigen::Matrix3d> ApproachObject::predictionController(
		const Eigen::Vector3d& curPosEf,
		const Eigen::Matrix3d & constRotLink6,
		std::vector<std::string> subjMarkersName
		)
	{
		bool ready{false};

		Eigen::Vector3d x, y, lshp_X, lshp_Y, lshp_Z;
		Eigen::Vector3d initRefPos, curPosLshp, initPosSubj, ithPosSubj, avgVelSubj, predictPos;

		Eigen::MatrixXd curVelSubj, P_M_Subj, wp;

		Eigen::Matrix3d subjHandRot, handoverRot;

		sva::PTransformd X_R_ef;
		sva::PTransformd X_R_M;

		sva::PTransformd X_M_Subj;
		sva::PTransformd X_ef_Subj;

		std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_ef_Subj;

		/*prediction_ tuner*/
		t_predict = (int)tuner(0);
		t_observe = (int)tuner(1);
		it = (int)tuner(2); //t_predict/t_observe;

		/*Subject Hand L SHAPE*/
		auto sizeStr = subjMarkersName.size();
		curPosLshp = markersPos[markers_name_index[subjMarkersName[sizeStr-2]]].col(i);//C
		y = markersPos[markers_name_index[subjMarkersName[sizeStr-1]]].col(i) - curPosLshp;//vCD=Y
		x = curPosLshp - markersPos[markers_name_index[subjMarkersName[0]]].col(i);//vAC=X

		lshp_X = x/x.norm();
		lshp_Y = y/y.norm();
		lshp_Z = lshp_X.cross(lshp_Y);

		subjHandRot.col(0) = lshp_X;
		subjHandRot.col(1) = lshp_Y;
		subjHandRot.col(2) = lshp_Z/lshp_Z.norm();
		// LOG_ERROR(subjHandRot<<"\n\n")


		P_M_Subj = Eigen::MatrixXd::Zero(3, t_observe);
		for(int j=1;j<=t_observe; j++)
		{
			P_M_Subj.col(j-1) = markersPos[markers_name_index[subjMarkersName[0]]].col((i-t_observe)+j);
			// cout << "P_M_Subj.col(j-1) " << P_M_Subj.col(j-1).transpose() <<endl;

			if(j==1)
				{ initPosSubj = P_M_Subj.col(j-1); }

			if(j==t_observe)
				{ ithPosSubj = P_M_Subj.col(t_observe-1); }
		}

		/*get average velocity of previous *t_observe* sec Subj movement*/
		avgVelSubj = (ithPosSubj - initPosSubj)/0.1;
		// cout<< " vel " << avgVelSubj.transpose() <<endl;


		/*predict position in straight line after t_predict time*/
		predictPos = handoverTraj->constVelocityPredictPos(avgVelSubj, ithPosSubj, t_predict);

		if(subjHasObject)
		{ X_M_Subj = sva::PTransformd(objRot, predictPos); }
		else if(robotHasObject)
		{ X_M_Subj = sva::PTransformd(subjHandRot, predictPos); }


		X_R_ef = sva::PTransformd(constRotLink6, curPosEf);
		X_R_M = sva::PTransformd(idtMat, Eigen::Vector3d(0., 0., 0.));

		X_ef_Subj = X_M_Subj * X_R_M * X_R_ef.inv();

		handoverRot = X_ef_Subj.rotation().transpose();

		/*way points for robot ef to predict pos*/
		wp_ef_Subj=handoverTraj->constVelocity(curPosEf, predictPos, t_predict);
		// wp_ef_Subj=handoverTraj->constVelocity(curPosEf, X_ef_Subj.translation(), t_predict);
		wp = get<0>(wp_ef_Subj);
		initRefPos << wp(0,it), wp(1,it), wp(2,it);

		ready = true;

		return std::make_tuple(ready, wp, initRefPos, handoverRot);
	}


	void ApproachObject::goToHandoverPose(
		double min,
		double max,
		bool& enableHand,
		Eigen::Vector3d& curPosEf,
		std::shared_ptr<mc_tasks::PositionTask>& posTask,
		std::shared_ptr<mc_tasks::OrientationTask>& oriTask,
		std::tuple<bool,
		Eigen::MatrixXd,
		Eigen::Vector3d,
		Eigen::Matrix3d> handPredict,
		Eigen::Vector3d offsetPos)
	{
		Eigen::Vector3d wp, handoverPos;

		if(Flag_prediction)
		{
			it+= (int)tuner(2);
			if( it < get<1>(handPredict).cols() )
			{
				wp << get<1>(handPredict)(0,it), get<1>(handPredict)(1,it), get<1>(handPredict)(2,it);
				handoverPos = curPosEf + (wp - get<2>(handPredict));
			}
		}
		else
		{
			handoverPos = offsetPos;
		}

		/*robot constraint*/
		if(enableHand &&
			(handoverPos(0)>= 0.10) && (handoverPos(0)<= 0.8)
			&& (handoverPos(1)>= min)  && (handoverPos(1)<= max)
			&& (handoverPos(2)>= 0.80) /*&& (handoverPos(2)<= 1.4)*/
			)
		{
			sva::PTransformd new_pose(get<3>(handPredict), handoverPos);
			posTask->position(new_pose.translation());
			oriTask->orientation(new_pose.rotation());

			// LOG_INFO("it "<<it << "\npredictPos wp "<<handoverPos.transpose()<<"\n" << "offsetPos "<< offsetPos.transpose())
		}
	}


	bool ApproachObject::forceController(
		bool& enableHand,

		Eigen::Vector3d initPosR, Eigen::Matrix3d initRotR,
		Eigen::Vector3d initPosL, Eigen::Matrix3d initRotL,
		Eigen::Vector3d relaxPos, Eigen::Matrix3d relaxRot,
		Eigen::VectorXd thresh,
		Eigen::Vector3d leftForce, Eigen::Vector3d rightForce,
		Eigen::Vector3d leftForceLo, Eigen::Vector3d rightForceLo,
		Eigen::Vector3d efLAce, Eigen::Vector3d efRAce,
		std::shared_ptr<mc_tasks::PositionTask>& posTaskL,
		std::shared_ptr<mc_tasks::OrientationTask>& oriTaskL,
		std::shared_ptr<mc_tasks::PositionTask>& posTaskR,
		std::shared_ptr<mc_tasks::OrientationTask>& oriTaskR,
		std::shared_ptr<mc_tasks::EndEffectorTask>& objEfTask
	)
	{
		double finR_rel_efL, finL_rel_efR;
		Eigen::Vector3d leftTh, rightTh;

		finR_rel_efL = (gripperEfL - fingerPosR).norm();
		finL_rel_efR = (gripperEfR - fingerPosL).norm();

		leftTh = thresh.segment(3,3);
		rightTh = thresh.segment(9,3);


		if( (finR_rel_efL < 0.35) && (finL_rel_efR < 0.35) )
		{
			auto checkForce = [&](const char *axis_name, int idx)
			{
				objMass = ( FloadL.norm() + FloadR.norm() )/9.81;

				FinertL = (objMass/2) * efLAce;
				FinertR = (objMass/2) * efRAce;

				FpullL[0] = abs(leftForce[0]) - abs(FinertL[0]) - abs(FzeroL[0]);
				FpullL[1] = abs(leftForce[1]) - abs(FinertL[1]) - abs(FzeroL[1]);
				FpullL[2] = abs(leftForce[2]) - abs(FinertL[2]) - abs(FzeroL[2]);

				FpullR[0] = abs(rightForce[0]) - abs(FinertR[0]) - abs(FzeroR[0]);
				FpullR[1] = abs(rightForce[1]) - abs(FinertR[1]) - abs(FzeroR[1]);
				FpullR[2] = abs(rightForce[2]) - abs(FinertR[2]) - abs(FzeroR[2]);

				/*new threshold*/
				newThL = FloadL + leftTh;
				newThR = FloadR + rightTh;

				if( (finR_rel_efL < 0.20) || (finL_rel_efR < 0.20) )
				{
					if(enableHand)
					{
						enableHand = false;
						LOG_WARNING("trying to pull object, motion stopped")
					}

					if( (abs(FpullL[idx]) > newThL[idx]) || (abs(FpullR[idx]) > newThR[idx]) )
					{
						if(objHasContacts)
						{
							removeContacts = true;
							robotHasObject = false;
							subjHasObject = true;

							gOpen = true;
							restartHandover = true;
							takeBackObject = false;

							if(goBackInit)
							{
								goBackInit = false;
								LOG_SUCCESS("object pulled and has mass(kg) = " << objMass)
							}
						}
						else
						{
							LOG_ERROR("robot doesn't have contacts with the object")
						}
					}
				}
				return false;
			};

			/*check if object is being pulled*/
			if(takeBackObject)
			{
				return checkForce("x-axis", 0) || checkForce("y-axis", 1) || checkForce("z-axis", 2);
			}
			else
			{
				/*open empty gripper when subject come near to robot*/
				if( (!openGripper) )
				{
					gOpen = true;
					LOG_INFO("1st cycle, opening grippers")
				}

				/*stop motion*/
				else if( (openGripper) && (!closeGripper) && (!restartHandover) &&
						(enableHand) && (obj_rel_robotRtHand < 0.12) && (obj_rel_robotLtHand < 0.12) )
				{
					FzeroL = leftForce;
					FzeroR = rightForce;

					local_FzeroL = leftForceLo;
					local_FzeroR = rightForceLo;

					enableHand = false;
					LOG_WARNING("motion stopped with Fzero L & R Norms "<<FzeroL.norm()<<" & "<< FzeroR.norm())
				}

				/*closed WITH object*/
				else if( (!enableHand) &&
						(graspObject) && /*along localY direction*/
						( abs( (leftForceLo - local_FzeroL)(2) ) >2.0 ) &&
						( abs( (rightForceLo - local_FzeroL)(2) ) > 2.0 ) )
				{
					gClose = true;
					closeGripper = true;
					graspObject = false;

					FcloseL = leftForce;
					FcloseR = rightForce;
					LOG_WARNING("closing with Fclose L & R Norms "<<FcloseL.norm()<<" & "<<FcloseR.norm())
				}

				/*closed WITHOUT object*/
				else if(
						(!restartHandover) && (!graspObject)  &&
						(obj_rel_robotRtHand > 0.15) && (obj_rel_robotRtHand < 0.2) &&
						(obj_rel_robotLtHand > 0.15) && (obj_rel_robotLtHand < 0.2) )
				{
					if( (FcloseL.norm() < 2.0) || (FcloseR.norm() < 2.0) )
					{
						gClose = false;
						closeGripper = false;
						graspObject = true;

						gOpen = true;
						LOG_ERROR("false close, Fclose L & R Norms, try with object again"<<FcloseL.norm()<<" & "<<FcloseR.norm())
					}
					else
					{
						FcloseL = Eigen::Vector3d(1,1,1);
						FcloseR = Eigen::Vector3d(1,1,1);
					}
				}
			}
		}


		/*restart handover*/
		if( (finR_rel_efL > 0.8) && (finL_rel_efR > 0.8) )
		{
			/*come only once after object is grasped*/
			if( (closeGripper) && (!restartHandover) && (!enableHand) )
			{
				/*add contacts*/
				if(e == 2)
				{ addContacts = true; }


				if( (e%200==0) )//wait xx sec
				{
					FloadL <<
					accumulate( FloadLx.begin(), FloadLx.end(), 0.0)/double(FloadLx.size()),
					accumulate( FloadLy.begin(), FloadLy.end(), 0.0)/double(FloadLy.size()),
					accumulate( FloadLz.begin(), FloadLz.end(), 0.0)/double(FloadLz.size());

					FloadR <<
					accumulate( FloadRx.begin(), FloadRx.end(), 0.0)/double(FloadRx.size()),
					accumulate( FloadRy.begin(), FloadRy.end(), 0.0)/double(FloadRy.size()),
					accumulate( FloadRz.begin(), FloadRz.end(), 0.0)/double(FloadRz.size());


					if(objHasContacts)
					{
						/*move right EF to relax pose*/
						posTaskR->stiffness(2.0);
						posTaskR->position(relaxPos);
						oriTaskR->orientation(relaxRot);

						LOG_SUCCESS("Robot has object, Ef(s) returning to relax pose, FloadL & FloadR are "<< FloadL.transpose() <<" :: "<< FloadR.transpose())
					}

					if( subjHasObject &&
						(obj_rel_subjRtHand > obj_rel_robotLtHand) &&
						(obj_rel_subjLtHand > obj_rel_robotRtHand)
						)
					{

						subjHasObject = false;
						robotHasObject = true;
					}

					if(robotHasObject)
					{
						enableHand = true;
						takeBackObject = true;
					}

					/*clear vector memories*/
					FloadLx.clear(); FloadLy.clear(); FloadLz.clear();
					FloadRx.clear(); FloadRy.clear(); FloadRz.clear();
				}
				else /*divide by 9.81 and you will get object mass*/
				{
					FloadLx.push_back( abs( abs(leftForce[0])-abs(FzeroL[0]) ) );
					FloadLy.push_back( abs( abs(leftForce[1])-abs(FzeroL[1]) ) );
					FloadLz.push_back( abs( abs(leftForce[2])-abs(FzeroL[2]) ) );

					FloadRx.push_back( abs( abs(rightForce[0])-abs(FzeroR[0]) ) );
					FloadRy.push_back( abs( abs(rightForce[1])-abs(FzeroR[1]) ) );
					FloadRz.push_back( abs( abs(rightForce[2])-abs(FzeroR[2]) ) );
				}
				e+=1;
			}


			if(restartHandover)
			{
				/*move EF to initial position*/
				if(!goBackInit)
				{
					posTaskL->stiffness(2.0);
					posTaskL->position(initPosL);
					oriTaskL->orientation(initRotL);

					posTaskR->stiffness(2.0);
					posTaskR->position(initPosR);
					oriTaskR->orientation(initRotR);

					gClose = true;
				}

				openGripper = false;
				closeGripper = false;

				graspObject = true;
				goBackInit = true;
				enableHand = true;
				e = 1;

				if( restartHandover && (posTaskL->eval().norm()) <0.1 && (posTaskR->eval().norm() <0.1) )
				{
					posTaskL->stiffness(4.0);
					posTaskR->stiffness(4.0);
					restartHandover = false;

					useLeftEf = true;
					useRightEf = true;
					startNow = false;

					addContacts = false;
					removeContacts = false;

					LOG_INFO("object returned to subject, motion enabled, restarting handover\n")
				}
			}
		}


		// /*return robot to relax pose, if human is too far*/
		// if( (robotHasObject) &&
		// 	(fingerPosL(0)>1.5) &&
		// 	(fingerPosR(0)>1.5) )
		// {
		// 	posTaskR->position(relaxPos);
		// 	oriTaskR->orientation(relaxRot);
		// 	enableHand = true;
		// 	cout<<i<<" relax\n";
		// }

		return false;
	}

}//namespace mc_handover