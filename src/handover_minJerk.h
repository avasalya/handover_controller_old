#ifndef _H_HANDOVER_MINJERK_H_
#define _H_HANDOVER_MINJERK_H


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

namespace mc_control 
{

    using namespace Eigen;
    class MinJerk {

    public:

    MinJerk();

    ~MinJerk();

    void setPosA(const Eigen::MatrixXd & a, int i);
    void setPosB(const Eigen::MatrixXd & b, int i);
    
    Eigen::MatrixXd getPosA();
    Eigen::MatrixXd getPosB();

    void produceWp(const Eigen::MatrixXd & posA, const Eigen::MatrixXd & posB, int T, int N);
    
    Eigen::MatrixXd wpPos;
    Eigen::MatrixXd wpVel;
    Eigen::MatrixXd wpAce;

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

    Eigen::MatrixXd pos1;
    Eigen::MatrixXd pos2;


    double n; 
    double t;


  };



}
#endif