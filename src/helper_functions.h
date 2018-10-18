#pragma once

#include <vector>
#include <iostream>

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;

using namespace std;
using namespace Eigen;


namespace mc_handover 
{
	struct HelperFunctions
	{

		HelperFunctions();
		~HelperFunctions();

		/*helper function*/
		void plotPos(Eigen::MatrixXd m, int d);
		void plotVel(Eigen::MatrixXd m, int d);

		bool plotSize{true};

	};


} // namespace mc_handover
