//
/// ... min_jerk_traj ...
///

#include "handover_minJerk.h"


namespace mc_control
{

  using namespace std;

  ////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                                        //
  //                               minJerk::minJerk()                                       //
  //                                                                                        //
  ////////////////////////////////////////////////////////////////////////////////////////////
  minJerk::minJerk()
  {
    std::cout << "minJerk constructor created " <<std::endl; 
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                                        //
  //                               minJerk::setPosA()                                       //
  //                                                                                        //
  ////////////////////////////////////////////////////////////////////////////////////////////
  void minJerk::setPosA(const Eigen::MatrixXd & a, int i)
  {
    pos1 = a;
    n   = i;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                                        //
  //                               minJerk::setPosB()                                       //
  //                                                                                        //
  ////////////////////////////////////////////////////////////////////////////////////////////
  void minJerk::setPosB(const Eigen::MatrixXd & b, int i)
  {
    pos2 = b;
    n   = i;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                                        //
  //                               minJerk::getPosA()                                       //
  //                                                                                        //
  ////////////////////////////////////////////////////////////////////////////////////////////
  MatrixXd minJerk::getPosA() 
  { 
    return pos1;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                                        //
  //                               minJerk::getPosB()                                       //
  //                                                                                        //
  ////////////////////////////////////////////////////////////////////////////////////////////
  MatrixXd minJerk::getPosB() 
  { 
    return pos2;
  }

  ////////////////////////////////////////////////////////////////////////////////////////////
  //                                                                                        //
  //                               minJerk::produceWp()                                     //
  //                                                                                        //
  ////////////////////////////////////////////////////////////////////////////////////////////
  void minJerk::produceWp(const Eigen::MatrixXd & posA, const Eigen::MatrixXd & posB, int T, int N)
  { 

    pos1 = posA;
    pos2 = posB;
    n    = N;
    t    = T;

    
    // // calculate Pos, Vel, Ace between A & B markers at each sample --real-time
    wpPos = pos1.row(0) + (pos2.row(t) - pos1.row(t)) * (
    10.0*pow((double)t/(double)(n-1),3)-
    15.0*pow((double)t/(double)(n-1),4)+
    6.0*pow((double)t/(double)(n-1),5));

    wpVel = (pos2.row(t) - pos1.row(t)) * (
    30.0/pow((double)(n-1),3)*pow((double)t,2)-
    60.0/pow((double)(n-1),4)*pow((double)t,3)+
    30.0/pow((double)(n-1),5)*pow((double)t,4));

    wpAce = (pos2.row(t) - pos1.row(t)) * (
    60.0/pow((double)(n-1),3)*pow((double)t,1)-
    180.0/pow((double)(n-1),4)*pow((double)t,2)+
    120.0/pow((double)(n-1),5)*pow((double)t,3));

    // // save waypoints as they generate // MatrixXd to Vector
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

}



