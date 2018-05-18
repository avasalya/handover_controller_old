// #pragma once

// #include <mc_tasks/MetaTask.h>
// #include <mc_tasks/RelativeEndEffectorTask.h>

// #include "handover_controller.h"


// namespace mc_control
// {	

// 	namespace
// 	{
// 	  static const std::pair<double, double> defaultFGain = {0.02, 0.005};
// 	  static const std::pair<double, double> defaultTGain = {0.2, 0.05};
// 	}

// 	struct MC_TASKS_DLLAPI AdmittanceTask : MetaTask
// 	{
// 		public:
// 			EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// 			/*! \brief Constructor with dof restrictions
// 			*
// 			* \param robots Robots where the task will be applied
// 			*
// 			* \param robotIndex Which robot among the robots
// 			*
// 			* \param body Name of the body controlled by this task.
// 			*
// 			* \param timestep Timestep of the controller
// 			*
// 			* \param dof Allows to enable/disable some axis in the control/wrench
// 			* monitoring
// 			*
// 			* \param stiffness Stiffness of the task
// 			*
// 			* \param weight Weight of the task
// 			*
// 			* \param forceThresh Force threshold to reach
// 			*
// 			* \param Torque threshold to reach
// 			*
// 			* \param forceGain PD gains on the force part
// 			*
// 			* \param torqueGain PD gains on the torque part
// 			*
// 			* \throws If the body the task is attempting to control does not have a
// 			* sensor attached to it
// 			*
// 			*/
// 			AdmittanceTask(const mc_rbdyn::Robots & robots,
// 			unsigned int robotIndex,
// 			const std::string & body,
// 			double timestep,
// 			const Eigen::Matrix6d& dof = Eigen::Matrix6d::Identity(),
// 			double stiffness = 5.0, double weight = 1000.0,
// 			double forceThresh = 3., double torqueThresh = 1.,
// 			std::pair<double,double> forceGain = defaultFGain,
// 			std::pair<double,double> torqueGain = defaultTGain);

// 			/*! \brief Reset the task
// 			*
// 			* Set the end effector objective to the current position of the end-effector
// 			*
// 			*/
// 			virtual void reset();

// 			/*! \brief Get the filtered wrench used by the task as a measure */
// 			sva::ForceVecd getFilteredWrench() const;

// 			/*! \brief Modify the target wrench */
// 			void setTargetWrench(const sva::ForceVecd& wrench)
// 			{
// 			obj_ = wrench;
// 			}

// 			/*! \brief Get the current target wrench */
// 			sva::ForceVecd getTargetWrench() { return obj_; }

// 			/*! \brief Set the task stiffness */
// 			void stiffness(double s)
// 			{
// 			efTask_->positionTask->stiffness(s);
// 			efTask_->orientationTask->stiffness(s);
// 			}
// 			/*! \brief Get the task stiffness */
// 			double stiffness() { return efTask_->positionTask->stiffness(); }

// 			/*! \brief Set the task weight */
// 			void weight(double w)
// 			{
// 			efTask_->positionTask->weight(w);
// 			efTask_->orientationTask->weight(w);
// 			}
// 			/*! \brief Get the task weight */
// 			double weight() { return efTask_->positionTask->weight(); }

// 			/*! \brief Set the force threshold */
// 			void forceThresh(double t) { forceThresh_ = t; }
// 			/*! \brief Get the force threshold */
// 			double forceThresh() { return forceThresh_; }

// 			/*! \brief Set the torque threshold */
// 			void torqueThresh(double t) { torqueThresh_ = t; }
// 			/*! \brief Get the torque threshold */
// 			double torqueThresh() { return torqueThresh_; }

// 			/*! \brief Set the force gain */
// 			void forceGain(std::pair<double, double> t) { forceGain_ = t; }
// 			/*! \brief Get the force gain */
// 			std::pair<double, double> forceGain() { return forceGain_; }

// 			/*! \brief Set the torque gain */
// 			void torqueGain(std::pair<double, double> t) { torqueGain_ = t; }
// 			/*! \brief Get the torque gain */
// 			std::pair<double, double> torqueGain() { return torqueGain_; }

// 			/*! \brief Set the current dof matrix */
// 			void dof(const Eigen::Matrix6d & dof) { dof_ = dof; }
// 			/*! \brief Get the current dof matrix */
// 			Eigen::Matrix6d dof() { return dof_; }

// 			void dimWeight(const Eigen::VectorXd & dimW) override;

// 			Eigen::VectorXd dimWeight() const override;

// 			void selectActiveJoints(mc_solver::QPSolver & solver,
// 			      const std::vector<std::string> & activeJointsName) override;

// 			void selectUnactiveJoints(mc_solver::QPSolver & solver,
// 			        const std::vector<std::string> & unactiveJointsName) override;

// 			void resetJointsSelector(mc_solver::QPSolver & solver) override;

// 			Eigen::VectorXd eval() const override
// 			{
// 			return wrench_.vector();
// 			}

// 			Eigen::VectorXd speed() const override
// 			{
// 			return robot_.mbc().bodyVelW[robot_.bodyIndexByName(sensor_.parentBody())].vector();
// 			}

// 		private:
// 			sva::PTransformd computePose();
// 			sva::ForceVecd wrench_;
// 			sva::ForceVecd obj_;
// 			sva::ForceVecd error_;
// 			sva::ForceVecd errorD_;

// 			const mc_rbdyn::Robot& robot_;
// 			const mc_rbdyn::ForceSensor& sensor_;

// 			double timestep_;
// 			double forceThresh_, torqueThresh_;

// 			std::pair<double,double> forceGain_, torqueGain_;
// 			std::function<double(double)> clampTrans_, clampRot_;
			
// 			std::shared_ptr<mc_tasks::RelativeEndEffectorTask> efTask_;
        
// 			Eigen::Matrix6d dof_;

// 			void addToSolver(mc_solver::QPSolver & solver) override;

// 			void removeFromSolver(mc_solver::QPSolver & solver) override;

// 			void update() override;
	
// 	};
// }// namspace mc_control









// // public:
// // 		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

// // 		AdmittanceTask(
// // 			mc_rbdyn::Robots & robots,
// // 			unsigned int robotIndex,
// // 			std::string body,
// // 			double timestep,
// // 			Eigen::Matrix6d & dof,
// // 	        double forceThresh = 3., double torqueThresh = 1.
// // 	        std::pair<double,double> forceGain = defaultFGain,
// //       		std::pair<double,double> torqueGain = defaultTGain,
// // 			std::string, sva::ForceVecd> & ctlWrenches,
// // 	        );
			
// // 		void update();
// // 		void reset();
// // 		void reset_dof(Eigen::Matrix6d dof_);
// // 		void resetForcePDGains(std::pair<double,double> forceGain);
// // 		void resetTorquePDGains(std::pair<double,double>torqueGain);

// // 		void resetComplVariableGains_M(Eigen::MatrixXd in);
// // 		void resetComplVariableGains_F(Eigen::MatrixXd in);

// // 		/*! \brief set the target wrench */
// // 		void setTargetWrench(const sva::ForceVecd& wrench)
// // 		{
// // 			obj_ = wrench;
// // 		}

// // 		/*! \brief Get the current target wrench */
// // 		sva::ForceVecd getTargetWrench() { return obj_; }

// // 		/*! \brief Set the force threshold */
// // 		void forceThresh(double t) { forceThresh_ = t; }
// // 		/*! \brief Get the force threshold */
// // 		double forceThresh() { return forceThresh_; }

// // 		/*! \brief Set the torque threshold */
// // 		void torqueThresh(double t) { torqueThresh_ = t; }
// // 		/*! \brief Get the torque threshold */
// // 		double torqueThresh() { return torqueThresh_; }


// // 		/*! \brief Returns the task's error */
// // 		Eigen::VectorXd eval()
// // 		{
// // 			return wrench_.vector();
// // 		}

// // 		/*! \brief Returns the task's speed */
// // 		Eigen::VectorXd speed()
// // 		{
// // 			return robot_.mbc().bodyVelW[robot_.bodyIndexByName(sensor_.parentBody())].vector();
// // 		}

// // 		void setGainsVariable(bool bool_variable_)
// // 		{
// // 			bool_variable = bool_variable_;
// // 		}

		
// // 		Eigen::MatrixXd gains_ODE = Eigen::MatrixXd::Zero(3,6);


// // 	private:

// // 		mc_rbdyn::Robot & robot_;
// // 		mc_rbdyn::ForceSensor & sensor_;
		
// // 		double forceThresh_, torqueThresh_;
// // 		double time;
// // 		double timestep_;

// // 		Eigen::Matrix6d dof;
				
// // 		sva::ForceVecd obj_;
// // 		sva::ForceVecd error_;
// // 		sva::ForceVecd errorD_;
// // 		sva::ForceVecd wrench_;
// // 		sva::PTransformd computePose();
		
// // 		std::string body_;
// // 		std::map<std::string, sva::ForceVecd>& ctlWrenches_;		
// // 		std::function<double(double)> clampMX, clampMY, clampMZ, clampFX, clampFY, clampFZ;

// // 		bool bool_variable = false;