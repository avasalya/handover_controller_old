#include "helper_functions.h"


namespace mc_handover
{
	void StartMocapStep::plotPos(Eigen::MatrixXd m, int d)
	{
		if(plotSize)
		{
			plotSize = false;
			plt::figure_size(1200, 780);
		}

		std::vector<double> x, y, z, tp;
		for(int j=1;j<d; j++)
		{ 
			x.push_back(m(0,j));
			y.push_back(m(1,j));
			z.push_back(m(2,j));
			tp.push_back(j);
			if(j%(d/50)==0)
			{
				// cout <<"x y z   " << x.at(j-1) << "  " << y.at(j-1) << "  " << z.at(j-1) << endl;

				plt::clf();

				plt::subplot(2,1,1);
				plt::plot(x,y,"r--");
				plt::plot(x,z,"g--");
				plt::plot(y,z,"b--");
				plt::ylim(-2,2);
				plt::xlim(-2,2);

				plt::subplot(2,1,2);
				plt::plot(tp,x,"r--");
				plt::plot(tp,y,"g--");
				plt::plot(tp,z,"b--");
				plt::ylim(-2,2);
				plt::xlim(0,1000);

					// plt::legend();

				plt::pause(1e-10);
			}
		}
	}



	void StartMocapStep::plotVel(Eigen::MatrixXd m, int d)
	{
		if(plotSize)
		{
			plotSize = false;
			plt::figure_size(1200, 780);
		}

		std::vector<double> x, y, z, tp;
		for(int j=0;j<d; j++)
		{ 
			x.push_back(m(0,j));
			y.push_back(m(1,j));
			z.push_back(m(2,j));
			tp.push_back(j);
			if(j%(d/5)==0)
			{
				// cout <<"x y z   " << x.at(j) << "  " << y.at(j) << "  " << z.at(j) << endl;

				plt::clf();
				plt::plot(tp,x,"r--");
				plt::plot(tp,y,"g--");
				plt::plot(tp,z,"b--");
					// plt::ylim(-2,2);
					// plt::xlim(0,d);


					// plt::legend();
				plt::pause(1e-10);
			}
		}cout <<"end loop "<<endl;
	}


}// namespace mc_handover
