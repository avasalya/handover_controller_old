#pragma once


#include <cmath>
#include <array>
#include <vector>
#include <chrono>
#include <thread>
#include <string>
#include <fstream>
#include <iostream>
#include <algorithm>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include <mc_control/mc_controller.h>

#include <mc_tasks/TrajectoryTask.h>

#include <mc_rbdyn/Robot.h>

#include <mc_rtc/logging.h>

#include <Tasks/QPTasks.h>

using namespace std;
using namespace mc_control;
using namespace Eigen;

namespace mc_handover 
{

    struct MinJerk {

    public:

    MinJerk();

    ~MinJerk();

    void setPosA(const Matrix3d & a);
    void setPosB(const Matrix3d & b);
    
    Matrix3d getPosA();
    Matrix3d getPosB();

    void produceWp(const Matrix3d & posA, const Matrix3d & posB, int T, int N);
    
    void minJerkPredictPos(const Matrix3d & posA, const Matrix3d & posB, int T, int N);

    void minJerkZeroBoundry(const Matrix3d & posA, const Matrix3d & posB, int T, int N);
    
    void minJerkNonZeroBoundry(const Matrix3d & posA, const Matrix3d & posB, int T, int N);

    Matrix3d wpPos;
    Matrix3d wpVel;
    Matrix3d wpAce;

    std::vector<double> xPos;
    std::vector<double> yPos;
    std::vector<double> zPos;

    std::vector<double> xVel;
    std::vector<double> yVel;
    std::vector<double> zVel;

    std::vector<double> xAce;
    std::vector<double> yAce;
    std::vector<double> zAce;

    private:

    Matrix3d pos2;
    Matrix3d pos1;

    double n; 
    double t;


  };



} // namespace mc_handover


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// function output = min_jerk(xi, xf, t)
// d = t(end);
// N = length(t);
// T = (t/d);
// a = repmat((xf - xi), N, 1);
// b = repmat((10 * T.^3 - 15 * T.^4 + 6 * T.^5)',1,3) ;
// output = repmat(xi, N, 1) + a .* b;
// end


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// function [xf, xdot] = min_jerk_final(xi, xc, t0, tc, tf)
// t = tc-t0;
// d = tf-t0;
// T = (t/d);
// xf   = xi +  (xc - xi)./ (10 * T.^3 - 15 * T.^4 + 6 * T.^5);
// xdot =       (xc - xi)./ (30 * T.^2 - 60 * T.^3 + 30 * T.^4);
// end


// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

// function output = min_jerk_vel(xi, vi, ai, xc, t)
// d   = t(end) -t(1);
// N   = length(t);
// T   = t-t(1);
// tau = T/d;

// a0 = repmat(xi, N, 1);
// a1 = d*repmat(vi, N, 1);
// a2 = d*repmat(ai, N, 1)/2;
// a3 = 3*repmat(ai, N, 1)*d.^2/2 - 6*d*repmat(vi, N, 1) + 10*(xc-xi);
// a4 = 3*repmat(ai, N, 1)*d.^2/2 + 8*d*repmat(vi, N, 1) - 15*(xc-xi);
// a5 =  -repmat(ai, N, 1)*d.^2/2 - 3*d*repmat(vi, N, 1) +  6*(xc-xi);

// output = a0 + a1.*tau' + a2.*tau.^2' + a3.*tau.^3' + a4.*tau.^4' + a5.*tau.^5';
// end
