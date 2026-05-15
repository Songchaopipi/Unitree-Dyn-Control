
#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <ocs2_core/misc/Numerics.h>

#include <humanoid_common_mpc/HumanoidPreComputation.h>

namespace ocs2::humanoid {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidPreComputation::HumanoidPreComputation(PinocchioInterface pinocchioInterface,
                                               const SwingTrajectoryPlanner& swingTrajectoryPlanner,  // 变量说明：swingTrajectoryPlanner 表示摆动脚、轨迹、planner。
                                               const MpcRobotModelBase<scalar_t>& mpcRobotModel)
    // 函数说明：处理Pinocchio、interface，连接当前模块的数据流和控制逻辑。
    : pinocchioInterface_(std::move(pinocchioInterface)),
      swingTrajectoryPlannerPtr_(&swingTrajectoryPlanner),
      mpcRobotModelPtr_(&mpcRobotModel) {
  eeNormalVelConConfigs_.resize(N_CONTACTS);
  R_world_to_contacts_.resize(N_CONTACTS);
  footHeightReferences_.resize(N_CONTACTS);
  for (size_t i = 0; i < N_CONTACTS; i++) {
    R_world_to_contacts_[i] = matrix3_t::Identity();
    footHeightReferences_[i] = 0.0;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

HumanoidPreComputation::HumanoidPreComputation(const HumanoidPreComputation& rhs)
    // 函数说明：处理Pinocchio、interface，连接当前模块的数据流和控制逻辑。
    : pinocchioInterface_(rhs.pinocchioInterface_),
      swingTrajectoryPlannerPtr_(rhs.swingTrajectoryPlannerPtr_),
      mpcRobotModelPtr_(rhs.mpcRobotModelPtr_),
      R_world_to_contacts_(rhs.R_world_to_contacts_),
      eeNormalVelConConfigs_(rhs.eeNormalVelConConfigs_),
      footHeightReferences_(rhs.footHeightReferences_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
HumanoidPreComputation* HumanoidPreComputation::clone() const {  // 变量说明：HumanoidPreComputation 表示humanoid、pre、computation。
  return new HumanoidPreComputation(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/

void HumanoidPreComputation::updatePinocchioModelKinematics(const vector_t& q) {  // 变量说明：HumanoidPreComputation 表示humanoid、pre、computation。
  const pinocchio::Model& model = pinocchioInterface_.getModel();  // 变量说明：model 表示模型。
  pinocchio::Data& data = pinocchioInterface_.getData();  // 变量说明：data 表示数据。

  // Perform the forward kinematics over the kinematic tree
  pinocchio::forwardKinematics(model, data, q);
  // 函数说明：更新坐标系/帧、placements，连接当前模块的数据流和控制逻辑。
  pinocchio::updateFramePlacements(model, data);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void HumanoidPreComputation::request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) {  // 变量说明：HumanoidPreComputation 表示humanoid、pre、computation。
  if (!request.containsAny(Request::Cost + Request::Constraint + Request::SoftConstraint)) {
    return;
  }

  updatePinocchioModelKinematics(mpcRobotModelPtr_->getGeneralizedCoordinates(x));

  // lambda to set config for normal velocity constraints
  auto eeNormalVelConConfig = [&](size_t footIndex) {  // 变量说明：eeNormalVelConConfig 表示ee、法向、速度、con、config。
    EndEffectorKinematicsLinearVelConstraint::Config config;  // 变量说明：config 表示从配置文件读取出的参数集合。
    config.b = (vector_t(1) << -swingTrajectoryPlannerPtr_->getZvelocityConstraint(footIndex, t)).finished();
    config.Av = (matrix_t(1, 3) << 0.0, 0.0, 1.0).finished();
    const ModelSettings::FootConstraintConfig& footConstraintCfg = mpcRobotModelPtr_->modelSettings.footConstraintConfig;  // 变量说明：footConstraintCfg 表示足端、约束、cfg。
    if (!numerics::almost_eq(footConstraintCfg.positionErrorGain_z, 0.0)) {
      config.b(0) -= footConstraintCfg.positionErrorGain_z * swingTrajectoryPlannerPtr_->getZpositionConstraint(footIndex, t);
      config.Ax = (matrix_t(1, 3) << 0.0, 0.0, footConstraintCfg.positionErrorGain_z).finished();
    }
    return config;
  };

  if (request.contains(Request::Constraint)) {
    for (size_t i = 0; i < N_CONTACTS; i++) {
      eeNormalVelConConfigs_[i] = eeNormalVelConConfig(i);
      pinocchio::FrameIndex frameID = pinocchioInterface_.getModel().getFrameId(mpcRobotModelPtr_->modelSettings.contactNames6DoF[i]);  // 变量说明：frameID 表示坐标系/帧、编号。
      R_world_to_contacts_[i] = pinocchioInterface_.getData().oMf[frameID].rotation().inverse();
      footHeightReferences_[i] = swingTrajectoryPlannerPtr_->getZpositionConstraint(i, t);
    }
  }
}

}  // namespace ocs2::humanoid
