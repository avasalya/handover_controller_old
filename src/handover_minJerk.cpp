//
/// ... min_jerk_traj ...
///

#include "handover_minJerk.h"


namespace mc_handover
{
	MinJerk::MinJerk()
  	{
    	std::cout << "minJerk constructor created " <<std::endl; 
  	}


	void MinJerk::setPosA(const Matrix3d & a)
	{
		pos1 = a;	
	}


	void MinJerk::setPosB(const Matrix3d & b)
	{
		pos2 = b;		
	}


	Matrix3d MinJerk::getPosA() 
	{
		return pos1;
	}


	Matrix3d MinJerk::getPosB() 
	{ 
		return pos2;
	}


	//  mjObj.produceWp(MatrixXd::Random(sample,3), MatrixXd::Random(sample,3), i, sample);
	void MinJerk::produceWp(const Matrix3d & posA, const Matrix3d & posB, int T, int N)
	{ 
		pos1 = posA;
		pos2 = posB;
		n    = N;
		t    = T;

		// // calculate Pos, Vel, Ace between A & B markers at each sample --real-time
		wpPos << pos1.row(0) + (pos2.row(t) - pos1.row(t)) * (
		10.0*pow((double)t/(double)(n-1),3)-
		15.0*pow((double)t/(double)(n-1),4)+
		6.0*pow((double)t/(double)(n-1),5));

		wpVel << (pos2.row(t) - pos1.row(t)) * (
		30.0/pow((double)(n-1),3)*pow((double)t,2)-
		60.0/pow((double)(n-1),4)*pow((double)t,3)+
		30.0/pow((double)(n-1),5)*pow((double)t,4));

		wpAce << (pos2.row(t) - pos1.row(t)) * (
		60.0/pow((double)(n-1),3)*pow((double)t,1)-
		180.0/pow((double)(n-1),4)*pow((double)t,2)+
		120.0/pow((double)(n-1),5)*pow((double)t,3));

		// save waypoints as they generate // Matrix3d to Vector

		xPos.push_back(wpPos(0,0));
		yPos.push_back(wpPos(0,1));
		zPos.push_back(wpPos(0,2));

		xVel.push_back(wpVel(0,0));
		yVel.push_back(wpVel(0,1));
		zVel.push_back(wpVel(0,2));

		xAce.push_back(wpAce(0,0));
		yAce.push_back(wpAce(0,1));
		zAce.push_back(wpAce(0,2));
	}

} // namespace mc_handover



