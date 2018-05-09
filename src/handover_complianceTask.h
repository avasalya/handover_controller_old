// #pragma once
// #include "mc_handover_controller.h"

// namespace mc_control
// {
// 	struct MCHandoverComplianceController : public mc_control::MCController
// 	{
// 		public:
// 		bool enableSensor(const std::string& sensorName);
// 		bool disableSensor(const std::string& sensorName);

// 		double forceGain;
// 		double torqueGain;

// 		double forceTh;
// 		double torqueTh;


//       protected:
//       	bool setDouble(double& d, const std::string& name, std::stringstream& ss, std::string& out);

//       private:
//         sva::ForceVecd wrenchInFrame(std::size_t wrench_index, sva::PTransformd& X_0_p);
      
//         void updateTorsoOri();
//         void updateCoMHeight();

//         // Calibrator calibrator_;
//         std::vector<bool> enabled;
//         std::vector<bool> interacting;
        
//         std::vector<mc_rbdyn::ForceSensor> sensors;
//         std::map<std::string, std::size_t> sensorIndexByName;
        


// 	};

// }
