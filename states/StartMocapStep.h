#pragma once

#include <iostream>
#include <fstream>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>


#include <mc_rbdyn/Robot.h>

#include "../cortex/cortex.h"


#include "handover_approachObject.h"


// Generated by cmake
// #include "datapath.h"

using namespace sva;

namespace mc_handover
{
	namespace states
	{
		struct StartMocapStep : mc_control::fsm::State
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		public:
			void configure(const mc_rtc::Configuration&) override {}
			void start(mc_control::fsm::Controller&) override;
			bool run(mc_control::fsm::Controller&) override;
			void teardown(mc_control::fsm::Controller&) override;
			
			double pi = 3.14;
			double DegToRad = pi/180;
			double RadToDeg = 180/pi;

			bool taskOK;

			int fps{200};
			int maxMarkers, markersCount, until;
			int body{0};

			Eigen::Vector3d move, target, initialCom = Eigen::Vector3d::Zero();

			Eigen::VectorXd thresh = Eigen::VectorXd::Zero(12);
			Eigen::Vector3d leftTh, rightTh;
			
			sva::PTransformd ltHand, rtHand;
			Eigen::Matrix3d ltRotW, rtRotW;

			Eigen::Vector3d p1r, p1l;
			Eigen::Quaterniond ql, qr, q1l, q1r;
			
			Eigen::Vector3d leftForce, rightForce;

			std::shared_ptr<mc_tasks::CoMTask> comTask;

			std::shared_ptr<mc_tasks::PositionTask> chestPosTask;
			std::shared_ptr<mc_tasks::OrientationTask> chestOriTask;
			
			
			// std::shared_ptr<mc_handover::ApproachObject> approachObj;
			// ApproachObject * approachObj = nullptr;


		private:
			sBodyDefs* pBodyDefs{NULL};
			sBodyDef* pBody{NULL};
			sFrameOfData* getCurFrame{NULL};
			sFrameOfData FrameofData;

			std::vector<int> bodyMarkers;

			void *pResponse;

			int nBytes;
			int totalBodies;
			int retval = RC_Okay;

			double del{0};

			bool startCapture{false};
			bool subjRtHandReady{true};
			bool subjLtHandReady{true};

		};
	} // namespace states
} // namespace mc_handover

EXPORT_SINGLE_STATE("StartMocapStep", mc_handover::states::StartMocapStep)


/*

normal = (u/u.norm()).cross(v/v.norm());
Eigen::Matrix3d rotation_;
rotation_.row(0) = (u/u.norm()).transpose();
rotation_.row(1) = (v/v.norm()).transpose();
rotation_.row(2) = (normal/normal.norm()).transpose();


// orthogonalization method Kevin
double angle = M_PI / 2. - std::acos(u.dot(v) / (u.norm() * v.norm()));
Eigen::Vector3d axis = u.cross(v);
axis = axis / axis.norm();
angle = angle / 2.;

//Rodrigues' rotation formula to rotate each of the vertices//
Eigen::Vector3d urot = u * std::cos(angle) + axis.cross(u) * std::sin(-angle) +
axis * axis.dot(u) * (1 - std::cos(angle));
Eigen::Vector3d vrot = v * std::cos(angle) + axis.cross(v) * std::sin(angle) +
axis * axis.dot(v) * (1 - std::cos(angle));

Eigen::Vector3d right = urot / urot.norm();
Eigen::Vector3d forward = vrot / vrot.norm();
Eigen::Vector3d up;
up = right.cross(forward);
rotation_.row(0) = right.transpose();
rotation_.row(1) = forward.transpose();
rotation_.row(2) = up.transpose();



//https://gamedev.stackexchange.com/questions/20097/how-to-calculate-a-3x3-rotation-matrix-from-2-direction-vectors
Matrix3x3 MakeMatrix( Vector3 X, Vector3 Y )  
{  
    // make sure that we actually have two unique vectors.
    assert( X != Y );

    Matrix3x3 M;  
    M.X = normalise( X );  
    M.Z = normalise( cross_product(X,Y) );
    M.Y = normalise( cross_product(M.Z,X) ); //-- to make sure it is orthogonal
    //otherwise just
    //M.Y = normalise( Y );
    return M;
}


//https://en.wikipedia.org/wiki/Rodrigues'_rotation_formula
//https://math.stackexchange.com/questions/871867/rotation-matrix-of-triangle-in-3d
//v′i=vicosθ+(k×vi)sinθ+k(k⋅vi)(1−cosθ)

*/
