#pragma once

#include <iostream>
#include <fstream>

#include <mc_control/fsm/Controller.h>
#include <mc_control/fsm/State.h>

#include <mc_rbdyn/Robot.h>

#include "handover_controller.h"
#include "../cortex/cortex.h"

// Generated by cmake
#include "datapath.h"

namespace mc_handover
{
	namespace states
	{
		struct StartMocapStep : mc_control::fsm::State
		{
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		public:
			void configure(const mc_rtc::Configuration & config) override;

			void start(mc_control::fsm::Controller&) override;

			bool run(mc_control::fsm::Controller&) override;

			void teardown(mc_control::fsm::Controller&) override;


			int fps{200};
			int t_predict;
			int t_observe;
			int it;
			Eigen::Vector3d tuner;

			/*mocap_simulaton*/
			double pt;
			std::vector<Eigen::MatrixXd> pos;
			std::vector<double> pts;
			std::string name;
			int i{1};
			int j{1};
			int s{0};

			/*mocap*/
			bool Flag_CORTEX{true}; // default True for MOCAP

			sva::PTransformd Subj_X_efL;
			sva::PTransformd ltHand, rtHand;
			
			std::vector<sva::PTransformd> S_X_efL;
			std::vector<Eigen::MatrixXd> markersPos;
			std::vector<Eigen::Vector3d> Markers;
			std::vector<Eigen::Vector3d> predictedPositions;
			std::vector<Eigen::Vector3d> efLPos, efLVel;
			Eigen::Vector3d efLAce;

			int body{0};

			int maxMarkers{13};//17

			int wristLtEfA{0}, wristLtEfB{1};
			int gripperLtEfA{2}, gripperLtEfB{3};

			int wristRtEfA{4}, wristRtEfB{5};
			int gripperRtEfA{6}, gripperRtEfB{7};

			int object{8};

			int fingerSubjLt{9};
			int lShapeLtA{10}, lShapeLtB{11}, lShapeLtC{12};

			// int fingerSubjRt{13};
			// int lShapeRtA{14}, lShapeRtB{15}, lShapeRtC{16};

			double leftForceNormAtGrasp;
			
			Eigen::Vector3d leftForce, leftForcesAtGrasp, leftTh, leftThAtGrasp;
			Eigen::Vector3d curPosLeftEf, curPosLeftEfMarker;
			Eigen::Vector3d randPos, initPosSubj, ithPosSubj, avgVelSubj, predictPos;

			Eigen::Quaterniond q, q1, q2, q3;

			/*Eigen::Matrix3d::Identity();*/
			Eigen::Matrix3d curRotLeftEfMarker;
			Eigen::Matrix3d curRotLeftEf;
			Eigen::Matrix3d rotSubj;
			Eigen::Matrix3d subjLtHandRot,subjRtHandRot;


			Eigen::MatrixXd curVelSubj, wp, newPosSubj;

			std::tuple<Eigen::MatrixXd, Eigen::Vector3d, Eigen::Vector3d> wp_efL_Subj;


			std::shared_ptr<mc_tasks::PositionTask> chestPosTask;
			std::shared_ptr<mc_tasks::OrientationTask> chestOriTask;

			double closeGrippers =0.14;
			double openGrippers = 0.5;



		private:

			std::shared_ptr<mc_tasks::CoMTask> comTask;

			std::vector<bool> handsWrenchDir;

			Eigen::VectorXd thresh = Eigen::VectorXd::Zero(12);
			Eigen::VectorXd baseTh = Eigen::VectorXd::Zero(12);			

			Eigen::Vector3d move, target;
			Eigen::Vector3d initialCom = Eigen::Vector3d::Zero();
			Eigen::Vector3d refPos, refPosPrev, refVel, refAcc, initRefPos, handoverPos, handoverPosPrev;

			/*mocap*/
			bool startCapture{false};
			bool collected{false};
			bool motion{true};

			bool restartHandover{false};
			bool openGripper{false};
			bool closeGripper{false};
			bool readyToGrasp{false};

			bool dum1{true};
			bool dum2{true};
			bool dum3{true};

			bool option1{false};
			bool option2{false};
			bool option3{false};

			bool oneTime{true};

			/*cortex*/
			sBodyDefs* pBodyDefs{NULL};
			sBodyDef* pBody{NULL};
			sFrameOfData* getCurFrame{NULL};
			sFrameOfData FrameofData;

			std::vector<int> bodyMarkers;

			void *pResponse;

			int nBytes;
			int retval = RC_Okay;
			int totalBodies;

			double del{0};

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
