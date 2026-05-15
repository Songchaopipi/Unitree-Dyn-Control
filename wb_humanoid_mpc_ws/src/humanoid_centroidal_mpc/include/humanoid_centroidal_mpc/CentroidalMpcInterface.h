#pragma once
#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/SqpSettings.h>

#include "humanoid_centroidal_mpc/common/CentroidalMpcRobotModel.h"
#include "humanoid_centroidal_mpc/initialization/CentroidalWeightCompInitializer.h"
#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/reference_manager/ProceduralMpcMotionManager.h"
#include "humanoid_common_mpc/reference_manager/SwitchedModelReferenceManager.h"

namespace ocs2::humanoid {

// == CentroidalMpcInterface 是 OCS2 侧的总装配入口:
// == 1. 读取 task/reference/URDF，创建Pinocchio模型、centroidal dynamics、代价、约束、rollout、initializer 和 reference manager。
// == 2. standalone 仿真程序只需要拿到这个对象，就能构建SqpMpc并通过 MPC_MRT_Interface 在线取 policy。
class CentroidalMpcInterface final : public RobotInterface {
 public:
  /**
   * Constructor
   *
   * @throw Invalid argument error if input task file or urdf file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] urdfFile: The absolute path to the URDF file for the robot.
   * @param [in] referenceFile: The absolute path to the reference configuration file.
   */
  CentroidalMpcInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile, bool setupOCP = true);

  ~CentroidalMpcInterface() override = default;

  // OptimalControlProblem 包含 dynamics、cost、constraints，是 SqpMpc 真正求解的最优控制问题。
  const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

  // CAREFUL: This function is not const, so it can easily be abused. It is currently only for gui purposes. Use with care!
  OptimalControlProblem& getOptimalControlProblemRef() const { return *problemPtr_; }

  // 下面这些 getter 暴露配置、模型和初始状态，standalone/mujoco 桥接层主要通过它们拿维度和关节映射。
  const ModelSettings& modelSettings() const { return modelSettings_; }
  // 函数说明：处理ddp、配置，连接当前模块的数据流和控制逻辑。
  const ddp::Settings& ddpSettings() const { return ddpSettings_; }
  // 函数说明：处理MPC、配置，连接当前模块的数据流和控制逻辑。
  const mpc::Settings& mpcSettings() const { return mpcSettings_; }
  // 函数说明：处理rollout、配置，连接当前模块的数据流和控制逻辑。
  const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }
  // 函数说明：处理sqp、配置，连接当前模块的数据流和控制逻辑。
  const sqp::Settings& sqpSettings() const { return sqpSettings_; }

  // 函数说明：获取初始、状态，连接当前模块的数据流和控制逻辑。
  const vector_t& getInitialState() const { return initialState_; }
  // 函数说明：获取rollout，连接当前模块的数据流和控制逻辑。
  const RolloutBase& getRollout() const { return *rolloutPtr_; }
  // 函数说明：获取Pinocchio、interface，连接当前模块的数据流和控制逻辑。
  PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
  // 函数说明：获取质心动力学、模型、模型信息，连接当前模块的数据流和控制逻辑。
  const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
  std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

  // 函数说明：获取initializer，连接当前模块的数据流和控制逻辑。
  const CentroidalWeightCompInitializer& getInitializer() const override { return *initializerPtr_; }
  std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

  // 函数说明：获取MPC、机器人、模型，连接当前模块的数据流和控制逻辑。
  const CentroidalMpcRobotModel<scalar_t>& getMpcRobotModel() const { return *mpcRobotModelPtr_; }
  // 函数说明：获取MPC、机器人、模型、自动微分，连接当前模块的数据流和控制逻辑。
  const CentroidalMpcRobotModel<ad_scalar_t>& getMpcRobotModelAD() const { return *mpcRobotModelADPtr_; }

  std::vector<std::string> getCostNames() const;
  std::vector<std::string> getTerminalCostNames() const;
  std::vector<std::string> getStateSoftConstraintNames() const;
  std::vector<std::string> getSoftConstraintNames() const;
  std::vector<std::string> getEqualityConstraintNames() const;

 private:
  // == 组装 OCP: 添加 centroidal dynamics、状态/输入代价、足端约束、摩擦锥、关节限制、mimic 约束等。
  void setupOptimalControlProblem(); 

  std::unique_ptr<StateInputConstraint> getStanceFootConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                size_t contactPointIndex);  // 变量说明：contactPointIndex 表示接触、point、索引。
  std::unique_ptr<StateInputConstraint> getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                    size_t contactPointIndex);  // 变量说明：contactPointIndex 表示接触、point、索引。
  std::unique_ptr<StateInputConstraint> getJointMimicConstraint(size_t mimicIndex);

  // 函数说明：添加task、space、kinematics、costs，连接当前模块的数据流和控制逻辑。
  void addTaskSpaceKinematicsCosts(const CentroidalModelPinocchioMappingCppAd& pinocchioMappingCppAd,
                                   const PinocchioEndEffectorKinematicsCppAd::update_pinocchio_interface_callback& velocityUpdateCallback);  // 变量说明：velocityUpdateCallback 表示速度、update、callback。

  // 配置对象：关节/接触命名、CppAD、足端约束参数。
  ModelSettings modelSettings_;  // 变量说明：modelSettings_ 表示模型、配置。
  ddp::Settings ddpSettings_;  // 变量说明：ddpSettings_ 表示ddp、配置。
  mpc::Settings mpcSettings_;  // 变量说明：mpcSettings_ 表示MPC、配置。
  sqp::Settings sqpSettings_;  // 变量说明：sqpSettings_ 表示sqp、配置。

  // Pinocchio 模型和 centroidal 模型信息。前者做运动学/动力学，后者记录 OCS2 centroidal 状态输入维度和 frame 索引。
  std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;  // 变量说明：pinocchioInterfacePtr_ 表示Pinocchio、interface、指针。
  CentroidalModelInfo centroidalModelInfo_;  // 变量说明：centroidalModelInfo_ 表示质心动力学、模型、模型信息。

  // OCP 和参考管理器。referenceManager 同时管理 mode schedule 和 target trajectories。
  std::unique_ptr<OptimalControlProblem> problemPtr_;  // 变量说明：problemPtr_ 表示优化问题、指针。
  std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;  // 变量说明：referenceManagerPtr_ 表示切换模型参考管理器指针。

  // scalar/ad_scalar 两套模型：运行时用 scalar，CppAD 代码生成用 ad_scalar。
  std::unique_ptr<CentroidalMpcRobotModel<scalar_t>> mpcRobotModelPtr_;  // 变量说明：mpcRobotModelPtr_ 表示MPC、机器人、模型、指针。
  std::unique_ptr<CentroidalMpcRobotModel<ad_scalar_t>> mpcRobotModelADPtr_;  // 变量说明：mpcRobotModelADPtr_ 表示MPC、机器人、模型、adptr。

  // rollout 用于 MPC 内部前向积分；initializer 给 SQP 初始解补偿重力/接触力。
  rollout::Settings rolloutSettings_;  // 变量说明：rolloutSettings_ 表示rollout、配置。
  std::unique_ptr<RolloutBase> rolloutPtr_;  // 变量说明：rolloutPtr_ 表示rollout、指针。
  std::unique_ptr<CentroidalWeightCompInitializer> initializerPtr_;  // 变量说明：initializerPtr_ 表示initializer、指针。

  // reference.info 中加载出的初始 centroidal state。
  vector_t initialState_;  // 变量说明：initialState_ 表示初始、状态。

  const std::string taskFile_;  // 变量说明：taskFile_ 表示task、file。
  const std::string urdfFile_;  // 变量说明：urdfFile_ 表示urdf、file。
  const std::string referenceFile_;  // 变量说明：referenceFile_ 表示参考、file。
  bool verbose_;  // 变量说明：verbose_ 表示本对象是否打印详细初始化信息。
};

}  // namespace ocs2::humanoid
