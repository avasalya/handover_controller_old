//
/// ... min_jerk_traj ...
///

#include "handover_trajectories.h"


namespace mc_handover
{
	MinJerk::MinJerk(){
    	std::cout << "minJerk constructor created " <<std::endl;}


	//  mjObj.minJerkZeroBoundry(MatrixXd::Random(sample,3), MatrixXd::Random(sample,3), i, sample);
	void MinJerk::minJerkZeroBoundary(const MatrixXd & xi, const MatrixXd & xf, double tf)
	{ 


/*d = t(end);
N = length(t);
T = (t/d);
a = repmat((xf - xi), N, 1);
b = repmat((10 * T.^3 - 15 * T.^4 + 6 * T.^5)',1,3) ;
output = repmat(xi, N, 1) + a .* b;*/


		// // calculate Pos, Vel, Ace between A & B markers at each sample --real-time
		for(int t=1; t<tf; t++)
		{
			wpPos << xi.row(0) + (xf.row(t) - xi.row(t)) * (
			10.0*pow((double)t/(double)(tf-1),3)-
			15.0*pow((double)t/(double)(tf-1),4)+
			6.0*pow((double)t/(double)(tf-1),5));
	
			wpVel << (xf.row(t) - xi.row(t)) * (
			30.0*pow((double)t/(double)(tf-1),2)-
			60.0*pow((double)t/(double)(tf-1),3)+
			30.0*pow((double)t/(double)(tf-1),4));
	
			wpAce << (xf.row(t) - xi.row(t)) * (
			60.0*(double)t/(double)(tf-1)-
			180.0*pow((double)t/(double)(tf-1),2)+
			120.0*pow((double)t/(double)(tf-1),3));


	
		}
	}



void MinJerk::minJerkPredictPos(const MatrixXd & xi, const MatrixXd & xc, double tc, double tf)
{
/*function [xf, xdot] = min_jerk_final(xi, xc, t0, tc, tf)
t = tc-t0;
d = tf-t0;
T = (t/d);
xf   = xi +  (xc - xi)./ (10 * T.^3 - 15 * T.^4 + 6 * T.^5);
xdot =       (xc - xi)./ (30 * T.^2 - 60 * T.^3 + 30 * T.^4);
end
*/

}

void MinJerk::minJerkNonZeroBoundary(const MatrixXd & xi, const MatrixXd & vi, const MatrixXd & ai,
															 const MatrixXd & xc, double tf)
{
/*function output = min_jerk_vel(xi, vi, ai, xc, t)
d   = t(end) -t(1);
N   = length(t);
T   = t-t(1);
tau = T/d;

a0 = repmat(xi, N, 1);
a1 = d*repmat(vi, N, 1);
a2 = d*repmat(ai, N, 1)/2;
a3 = 3*repmat(ai, N, 1)*d.^2/2 - 6*d*repmat(vi, N, 1) + 10*(xc-xi);
a4 = 3*repmat(ai, N, 1)*d.^2/2 + 8*d*repmat(vi, N, 1) - 15*(xc-xi);
a5 =  -repmat(ai, N, 1)*d.^2/2 - 3*d*repmat(vi, N, 1) +  6*(xc-xi);

output = a0 + a1.*tau' + a2.*tau.^2' + a3.*tau.^3' + a4.*tau.^4' + a5.*tau.^5';
end
*/
}


} // namespace mc_handover



