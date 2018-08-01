
#include "handover_trajectories.h"


namespace mc_handover
{
	HandoverTrajectory::HandoverTrajectory()
	{
    	std::cout << " handover trajectory constructor created " <<std::endl;
    }


    /* returns way points between xi and xf in tf time using nonZero boundary conditions */
	std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd> HandoverTrajectory::minJerkZeroBoundary(const Eigen::Vector3d & xi, const Eigen::Vector3d & xf, double tf)
	{ 
		Eigen::MatrixXd a, b1, b2, b3, pos, vel, ace;

		a.resize(tf,3);

		b1.resize(tf,1); b2.resize(tf,1); b3.resize(tf,1);

		pos.resize(tf,3); vel.resize(tf,3);	ace.resize(tf,3);

		for(int i=0; i<tf; i++)
		{
			a.row(i) = (xf-xi);
			b1(i) = (10*pow((i/tf),3) -  15*pow((i/tf),4) +    6*pow((i/tf),5));
			b2(i) = (30*pow((i/tf),2) -  60*pow((i/tf),3) +   30*pow((i/tf),4));
			b3(i) = (60*(i/tf)        - 180*pow((i/tf),2) +  120*pow((i/tf),3));

			pos.row(i) =  xi + (a.row(i).transpose()*b1(i));
			vel.row(i) =       (a.row(i).transpose()*b2(i));
			ace.row(i) =       (a.row(i).transpose()*b3(i));
		}
		// cout << pos << endl<< endl;
		// cout << vel << endl<< endl;
		// cout << ace << endl<< endl;
		return std::make_tuple(pos, vel, ace);
	}




    /* returns way points between xi and xf in tf time using nonZero boundary conditions */
	std::tuple<Eigen::MatrixXd, Eigen::MatrixXd, Eigen::MatrixXd>  HandoverTrajectory::minJerkNonZeroBoundary(const Eigen::Vector3d & xi, const Eigen::Vector3d & vi, const Eigen::Vector3d & ai, const Eigen::Vector3d & xc, double tf)
	{

		Eigen::MatrixXd a0, a1, a2, a3, a4, a5;
		Eigen::MatrixXd b1, b2, b3, pos, vel, ace;

		double tau, D;

		a0.resize(tf,3); a1.resize(tf,3); a2.resize(tf,3);
		a3.resize(tf,3); a4.resize(tf,3); a5.resize(tf,3);

		b1.resize(tf,1); b2.resize(tf,1); b3.resize(tf,1);

		pos.resize(tf,3); vel.resize(tf,3); ace.resize(tf,3);

		for(int i=0; i<tf; i++)
		{	
			D  = 1/tf;
			tau = i*D;

			a0.row(i) = xi;
			a1.row(i) = tf*vi;
			a2.row(i) = tf*ai.array()/2;
			a3.row(i) = (3*ai*pow((tf),2))/2 - (6*tf*vi) + (10*(xc-xi));
			a4.row(i) = (3*ai*pow((tf),2))/2 + (8*tf*vi) - (15*(xc-xi));
			a5.row(i) = -(ai*pow((tf),2))/2  - (3*tf*vi) + (6*(xc-xi));

			pos.row(i) = a0.row(i) + a1.row(i)*tau + a2.row(i)*pow((tau),2) + a3.row(i)*pow((tau),3)
			+ a4.row(i)*pow((tau),4) + a5.row(i)*pow((tau),5);

			vel.row(i) =			a1.row(i)/D + 2*a2.row(i)*tau/D + 3*a3.row(i)*pow((tau),2)/D
			+ 4*a4.row(i)*pow((tau),3)/D + 5*a5.row(i)*pow((tau),4)/D;


			ace.row(i) = 						 2*a2.row(i)/pow((D),2) + 6*a3.row(i)*tau/pow((D),2)
			+ 12*a4.row(i)*pow((tau),2)/pow((D),2) + 20*a5.row(i)*pow((tau),3)/pow((D),2);
		}
		// cout << pos << endl<< endl;
		// cout << vel << endl<< endl;
		// cout << ace << endl<< endl;
		return std::make_tuple(pos, vel, ace);
	}



	/* return xf in time tf based on current xc */
	std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d> HandoverTrajectory::minJerkPredictPos(const Eigen::Vector3d & xi, const Eigen::Vector3d & xc, double t0, double tc, double tf)
	{
		Eigen::Vector3d a, pos, vel, ace;
		
		double b1, b2, b3;
		double t = tc-t0;
		double d = tf-t0;
		double T = (t/d);
		
		a  << (xc-xi);
		b1 = (10*pow((T),3) -  15*pow((T),4) +    6*pow((T),5));
		b2 = (30*pow((T),2) -  60*pow((T),3) +   30*pow((T),4));
		b3 = (60*(T)        - 180*pow((T),2) +  120*pow((T),3));

		pos <<	xi.array() + (a.array()/b1);
		vel <<				 (a.array()/b2);
		ace <<				 (a.array()/b3);
		
		// cout << pos << endl<< endl;
		// cout << vel << endl<< endl;
		// cout << ace << endl<< endl;
		return std::make_tuple(pos, vel, ace);
	}



	/* position based on const velocity */
	std::tuple<Eigen::MatrixXd, Eigen::MatrixXd>HandoverTrajectory::constVelocity(const Eigen::Vector3d & xi, const Eigen::Vector3d & xf, double tf)
	{	
		MatrixXd pos, vel;

		pos.resize(tf,3); vel.resize(tf,3);

		Eigen::Vector3d slope 	 = -(xi-xf)/tf;
		Eigen::Vector3d constant  =  xi; //-((xi-xf)*t0)/(t0-tf);

		for(int i=0; i<tf; i++)
		{
			pos.row(i) = slope*i + constant;
			vel.row(i) = slope;
		}

		// cout << pos << endl<< endl;
		// cout << vel << endl<< endl;
		return std::make_tuple(pos, vel);
	}


	/* position at tf time based on const velocity */
	Eigen::Vector3d HandoverTrajectory::constVelocityPredictPos(const Eigen::Vector3d & xi, const Eigen::Vector3d xdot, double tf)
	{	
		Eigen::Vector3d pos;
		pos << xdot*tf + xi;
		// cout << pos << endl<< endl;
		return pos;
	}

} // namespace mc_handover



