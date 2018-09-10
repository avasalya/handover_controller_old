#pragma once

#include "handover_controller.h"

namespace mc_handover 
{
	struct HelperFunctions
	{
		/*helper function*/
		void plotPos(Eigen::MatrixXd m, int d);
		void plotVel(Eigen::MatrixXd m, int d);

	};


} // namespace mc_handover
