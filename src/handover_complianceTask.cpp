
#include "handover_complianceTask.h"


namespace mc_handover
{

namespace
{

  std::function<double(double)> clamper(double value)
  {
    auto clamp = [value](const double & v) { return std::min(std::max(v, -value), value); };
    return clamp;
  }

}

HandoverComplianceTask::HandoverComplianceTask(const mc_rbdyn::Robots & robots,
      unsigned int robotIndex,
      const std::string & body,
      double timestep,
      const Eigen::Matrix6d& dof,
      double stiffness, double weight, double forceThresh, double torqueThresh,
      std::pair<double, double> forceGain, std::pair<double, double> torqueGain)
  : wrench_(Eigen::Vector6d::Zero()),
    obj_(Eigen::Vector6d::Zero()),
    error_(Eigen::Vector6d::Zero()),
    errorD_(Eigen::Vector6d::Zero()),
    robot_(robots.robots()[robotIndex]),
    sensor_(robot_.bodyForceSensor(body)),
    timestep_(timestep),
    forceThresh_(forceThresh),
    torqueThresh_(torqueThresh),
    forceGain_(forceGain),
    torqueGain_(torqueGain),
    dof_(dof)
{
  efTask_ = std::make_shared<mc_tasks::EndEffectorTask>(body, robots,
                                              robotIndex, stiffness, weight);
  clampTrans_ = clamper(0.01);
  clampRot_ = clamper(0.1);

  type_ = "compliance";
  name_ = "compliance_" + robot_.name() + "_" + body;
}


void HandoverComplianceTask::reset()
{
  efTask_->reset();
}

sva::ForceVecd HandoverComplianceTask::getFilteredWrench() const
{
  return wrench_ + sva::ForceVecd(dof_*obj_.vector());
}


void HandoverComplianceTask::update()
{
  error_ = wrench_;
  /* Get wrench, remove gravity, use dof_ to deactivate some axis */
  wrench_ = sensor_.removeGravity(robot_);
  wrench_ = sva::ForceVecd(dof_*(wrench_ - obj_).vector());
  errorD_ = (wrench_ - error_)/timestep_;
  efTask_->set_ef_pose(computePose());
  /* Does nothing for now, but is here in case of changes */
  MetaTask::update(*efTask_);
}


void HandoverComplianceTask::addToSolver(mc_solver::QPSolver & solver)
{
  MetaTask::addToSolver(*efTask_, solver);
}


void HandoverComplianceTask::removeFromSolver(mc_solver::QPSolver & solver)
{
  MetaTask::removeFromSolver(*efTask_, solver);
}


void HandoverComplianceTask::dimWeight(const Eigen::VectorXd & dimW)
{
  efTask_->dimWeight(dimW);
}

Eigen::VectorXd HandoverComplianceTask::dimWeight() const
{
  return efTask_->dimWeight();
}

void HandoverComplianceTask::selectActiveJoints(mc_solver::QPSolver & solver,
                                const std::vector<std::string> & activeJointsName)
{
  efTask_->selectActiveJoints(solver, activeJointsName);
}

void HandoverComplianceTask::selectUnactiveJoints(mc_solver::QPSolver & solver,
                                  const std::vector<std::string> & unactiveJointsName)
{
  efTask_->selectUnactiveJoints(solver, unactiveJointsName);
}

void HandoverComplianceTask::resetJointsSelector(mc_solver::QPSolver & solver)
{
  efTask_->resetJointsSelector(solver);
}



sva::PTransformd HandoverComplianceTask::computePose()
{
    Eigen::Vector3d trans = Eigen::Vector3d::Zero();
    Eigen::Matrix3d rot = Eigen::Matrix3d::Identity();
    if(wrench_.force().norm() > forceThresh_)
    {
      trans = (forceGain_.first*wrench_.force() + forceGain_.second*errorD_.force()).unaryExpr(clampTrans_);
    }
    if(wrench_.couple().norm() > torqueThresh_)
    {
      Eigen::Vector3d rpy = (torqueGain_.first*wrench_.couple() + torqueGain_.second*errorD_.couple()).unaryExpr(clampRot_);
      rot = mc_rbdyn::rpyToMat(rpy);
    }
    const auto & X_p_f = sensor_.X_p_f();
    const auto & X_0_p = robot_.mbc().bodyPosW[robot_.bodyIndexByName(sensor_.parentBody())];
    sva::PTransformd move(rot, trans);
    const auto & X_f_ds = sensor_.X_fsmodel_fsactual();
    return ((X_f_ds*X_p_f).inv()*move*(X_f_ds*X_p_f))*X_0_p;
}


} // mc_handover

