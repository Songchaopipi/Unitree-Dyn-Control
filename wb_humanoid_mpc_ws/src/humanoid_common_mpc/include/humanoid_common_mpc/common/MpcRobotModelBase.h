
#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "humanoid_common_mpc/common/ModelSettings.h"
#include "humanoid_common_mpc/common/Types.h"

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <stdexcept>

#include <ocs2_core/misc/LoadData.h>

namespace ocs2::humanoid {

// MPC 机器人模型的抽象基类。它统一回答三类问题：
// 1. 状态/输入向量里每一段从哪里开始；
// 2. 如何从 OCS2 的 state/input 取出 Pinocchio 需要的 q/v；
// 3. 如何读写接触力/力矩和关节角速度。
template <typename SCALAR_T>
class MpcRobotModelBase {
 public:
  MpcRobotModelBase(const ModelSettings& modelSettings, scalar_t state_dim, scalar_t input_dim)
      // 函数说明：处理模型、配置，连接当前模块的数据流和控制逻辑。
      : modelSettings(modelSettings),
        state_dim(state_dim),
        input_dim(input_dim),
        base_dim(6),
        gen_coordinates_dim(base_dim + modelSettings.mpc_joint_dim){};

  virtual ~MpcRobotModelBase() = default;
  // 函数说明：处理clone，连接当前模块的数据流和控制逻辑。
  virtual MpcRobotModelBase* clone() const = 0;  // 变量说明：MpcRobotModelBase 表示MPC、机器人、模型、浮动基/机身。

  /******************************************************************************************************/
  /*                                           Dimensions                                               */
  /******************************************************************************************************/

  // state_dim/input_dim 是 OCS2 优化问题维度；base_dim 固定为 6，表示 [x y z yaw pitch roll]。
  size_t getStateDim() const { return state_dim; };
  // 函数说明：获取输入、维度，连接当前模块的数据流和控制逻辑。
  size_t getInputDim() const { return input_dim; };
  // 函数说明：获取浮动基/机身、维度，连接当前模块的数据流和控制逻辑。
  size_t getBaseDim() const { return base_dim; };
  // 函数说明：获取关节、维度，连接当前模块的数据流和控制逻辑。
  size_t getJointDim() const { return modelSettings.mpc_joint_dim; };
  // 函数说明：获取完整模型、模型、关节、维度，连接当前模块的数据流和控制逻辑。
  size_t getFullModelJointDim() const { return modelSettings.full_joint_dim; };
  // 函数说明：获取gen、coordinates、维度，连接当前模块的数据流和控制逻辑。
  size_t getGenCoordinatesDim() const { return gen_coordinates_dim; };

  /******************************************************************************************************/
  /*                                          Start indices                                             */
  /******************************************************************************************************/

  virtual size_t getBaseStartindex() const = 0;  // 变量说明：size_t 表示大小、t。
  // 函数说明：获取关节、startindex，连接当前模块的数据流和控制逻辑。
  virtual size_t getJointStartindex() const = 0;  // 变量说明：size_t 表示大小、t。
  // 函数说明：获取关节、velocities、startindex，连接当前模块的数据流和控制逻辑。
  virtual size_t getJointVelocitiesStartindex() const = 0;  // 变量说明：size_t 表示大小、t。

  // 接触 wrench 在 input 中按 [f_x, f_y, f_z, M_x, M_y, M_z]^T 排列，且使用世界系表达。
  virtual size_t getContactWrenchStartIndices(size_t contactIndex) const = 0;  // 变量说明：size_t 表示大小、t。
  // 函数说明：获取接触、力、start、indices，连接当前模块的数据流和控制逻辑。
  virtual size_t getContactForceStartIndices(size_t contactIndex) const { return getContactWrenchStartIndices(contactIndex); }  // 变量说明：size_t 表示大小、t。
  // 函数说明：获取接触、moment、start、indices，连接当前模块的数据流和控制逻辑。
  virtual size_t getContactMomentStartIndices(size_t contactIndex) const { return 6 * contactIndex + 3; }  // 变量说明：size_t 表示大小、t。

  /******************************************************************************************************/
  /*                                     Generalized coordinates                                        */
  /******************************************************************************************************/

  // generalized coordinates 是 Pinocchio 的最小坐标 [base pose, active joint angles]。
  virtual VECTOR_T<SCALAR_T> getGeneralizedCoordinates(const VECTOR_T<SCALAR_T>& state) const = 0;  // 变量说明：VECTOR_T 表示向量、t。

  // 函数说明：获取浮动基/机身、pose，连接当前模块的数据流和控制逻辑。
  virtual VECTOR6_T<SCALAR_T> getBasePose(const VECTOR_T<SCALAR_T>& state) const = 0;  // 变量说明：VECTOR6_T 表示vector6、t。

  // 函数说明：获取浮动基/机身、位置，连接当前模块的数据流和控制逻辑。
  virtual VECTOR3_T<SCALAR_T> getBasePosition(const VECTOR_T<SCALAR_T>& state) const = 0;  // 变量说明：VECTOR3_T 表示vector3、t。

  // 函数说明：获取浮动基/机身、姿态、欧拉角、zyx，连接当前模块的数据流和控制逻辑。
  virtual VECTOR3_T<SCALAR_T> getBaseOrientationEulerZYX(const VECTOR_T<SCALAR_T>& state) const = 0;  // 变量说明：VECTOR3_T 表示vector3、t。

  // 函数说明：获取浮动基/机身、质心、linear、速度，连接当前模块的数据流和控制逻辑。
  virtual VECTOR3_T<SCALAR_T> getBaseComLinearVelocity(const VECTOR_T<SCALAR_T>& state) const = 0;  // 变量说明：VECTOR3_T 表示vector3、t。

  // 函数说明：获取浮动基/机身、质心、速度，连接当前模块的数据流和控制逻辑。
  virtual VECTOR6_T<SCALAR_T> getBaseComVelocity(const VECTOR_T<SCALAR_T>& state) const = 0;  // 变量说明：VECTOR6_T 表示vector6、t。

  // 函数说明：获取关节、angles，连接当前模块的数据流和控制逻辑。
  virtual VECTOR_T<SCALAR_T> getJointAngles(const VECTOR_T<SCALAR_T>& state) const = 0;  // 变量说明：VECTOR_T 表示向量、t。

  // 函数说明：获取关节、velocities，连接当前模块的数据流和控制逻辑。
  virtual VECTOR_T<SCALAR_T> getJointVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) const = 0;  // 变量说明：VECTOR_T 表示向量、t。

  // generalized velocities 是 [base linear/angular velocity, active joint velocities]，
  // 其中 base 速度由 centroidal momentum 和当前构型反推，关节速度直接来自 input 尾部。
  virtual VECTOR_T<SCALAR_T> getGeneralizedVelocities(const VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& input) = 0;  // 变量说明：VECTOR_T 表示向量、t。

  /******************************************************************************************************/

  virtual void setGeneralizedCoordinates(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& generalizedCorrdinates) const = 0;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  // 函数说明：设置浮动基/机身、pose，连接当前模块的数据流和控制逻辑。
  virtual void setBasePose(VECTOR_T<SCALAR_T>& state, const VECTOR6_T<SCALAR_T>& basePose) const = 0;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  // 函数说明：设置浮动基/机身、位置，连接当前模块的数据流和控制逻辑。
  virtual void setBasePosition(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& position) const = 0;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  // 函数说明：设置浮动基/机身、姿态、欧拉角、zyx，连接当前模块的数据流和控制逻辑。
  virtual void setBaseOrientationEulerZYX(VECTOR_T<SCALAR_T>& state, const VECTOR3_T<SCALAR_T>& eulerAnglesZYX) const = 0;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  // 函数说明：设置关节、angles，连接当前模块的数据流和控制逻辑。
  virtual void setJointAngles(VECTOR_T<SCALAR_T>& state, const VECTOR_T<SCALAR_T>& jointAngles) const = 0;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  // 函数说明：设置关节、velocities，连接当前模块的数据流和控制逻辑。
  virtual void setJointVelocities(VECTOR_T<SCALAR_T>& state,  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。
                                  VECTOR_T<SCALAR_T>& input,  // 变量说明：input 表示输入。
                                  const VECTOR_T<SCALAR_T>& jointVelocities) const = 0;  // 变量说明：jointVelocities 表示关节、velocities。

  // 函数说明：处理adapt、浮动基/机身、pose、height，连接当前模块的数据流和控制逻辑。
  virtual void adaptBasePoseHeight(VECTOR_T<SCALAR_T>& state, scalar_t heightChange) const = 0;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  /******************************************************************************************************/
  /*                                          Contacts                                                  */
  /******************************************************************************************************/

  // 这些函数是代价/约束和控制器读写足端接触 wrench 的统一入口。
  virtual VECTOR6_T<SCALAR_T> getContactWrench(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const = 0;  // 变量说明：VECTOR6_T 表示vector6、t。

  // 函数说明：获取接触、力，连接当前模块的数据流和控制逻辑。
  virtual VECTOR3_T<SCALAR_T> getContactForce(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const = 0;  // 变量说明：VECTOR3_T 表示vector3、t。

  // 函数说明：获取接触、moment，连接当前模块的数据流和控制逻辑。
  virtual VECTOR3_T<SCALAR_T> getContactMoment(const VECTOR_T<SCALAR_T>& input, size_t contactIndex) const = 0;  // 变量说明：VECTOR3_T 表示vector3、t。

  /******************************************************************************************************/

  virtual void setContactWrench(VECTOR_T<SCALAR_T>& input, const VECTOR6_T<SCALAR_T>& wrench, size_t contactIndex) const = 0;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  // 函数说明：设置接触、力，连接当前模块的数据流和控制逻辑。
  virtual void setContactForce(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& force, size_t contactIndex) const = 0;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  // 函数说明：设置接触、moment，连接当前模块的数据流和控制逻辑。
  virtual void setContactMoment(VECTOR_T<SCALAR_T>& input, const VECTOR3_T<SCALAR_T>& moment, size_t contactIndex) const = 0;  // 变量说明：该函数通过引用参数或成员变量产生副作用，不返回数值。

  /******************************************************************************************************/
  /*                                         Joint angle                                                */
  /******************************************************************************************************/

  // 通过关节名取 MPC 内部索引。注意这里不是完整 URDF 索引，而是剔除 fixedJointNames 后的 active joint 索引。
  size_t getJointIndex(const std::string& jointName) const {
    auto it = modelSettings.jointIndexMap.find(jointName);  // 变量说明：it 表示名称查找迭代器。
    if (it != modelSettings.jointIndexMap.end()) {
      return it->second;  // Return the found index
    } else {
      throw std::runtime_error("Joint name " + jointName + " is not contained in MPC model!");
    }
  }

  // 把 MPC active joints 填回完整模型关节向量；固定关节沿用 defaultFullModelJointAngles。
  VECTOR_T<SCALAR_T> getFullModelJointAngles(const VECTOR_T<SCALAR_T>& mpcModelJointAngles,
                                             const VECTOR_T<SCALAR_T>& defaultFullModelJointAngles) const {  // 变量说明：defaultFullModelJointAngles 表示默认、完整模型、模型、关节、angles。
    VECTOR_T<SCALAR_T> fullModelJointAngles = VECTOR_T<SCALAR_T>(defaultFullModelJointAngles);  // 变量说明：fullModelJointAngles 表示完整模型、模型、关节、angles。
    assert(mpcModelJointAngles.size() == modelSettings.mpc_joint_dim);
    assert(defaultFullModelJointAngles.size() == modelSettings.full_joint_dim);
    for (size_t i = 0; i < modelSettings.mpc_joint_dim; ++i) {
      size_t currJointFullIndex = modelSettings.mpcModelToFullJointsIndices[i];  // 变量说明：currJointFullIndex 表示curr、关节、完整模型、索引。
      fullModelJointAngles[currJointFullIndex] = mpcModelJointAngles[i];
    }
    return fullModelJointAngles;
  }

  // 从完整模型关节角里抽取 MPC 关节角，顺序与 mpcModelJointNames 保持一致。
  VECTOR_T<SCALAR_T> getMpcModelJointAngles(const VECTOR_T<SCALAR_T>& fullModelJointAngles) const {
    VECTOR_T<SCALAR_T> mpcModelJointAngles(modelSettings.mpc_joint_dim);
    assert(fullModelJointAngles.size() == modelSettings.full_joint_dim);
    for (size_t i = 0; i < modelSettings.mpc_joint_dim; ++i) {
      mpcModelJointAngles[i] = fullModelJointAngles[modelSettings.mpcModelToFullJointsIndices[i]];
    }
    return mpcModelJointAngles;
  }

 protected:
  MpcRobotModelBase(const MpcRobotModelBase& rhs)
      // 函数说明：处理模型、配置，连接当前模块的数据流和控制逻辑。
      : modelSettings(rhs.modelSettings),
        state_dim(rhs.state_dim),
        input_dim(rhs.input_dim),
        base_dim(6),
        gen_coordinates_dim(rhs.gen_coordinates_dim){};

 public:
  // modelSettings 是全局关节/接触命名与映射表，外部模块经常需要直接读它。
  const ModelSettings& modelSettings;  // 变量说明：modelSettings 表示机器人关节和接触配置。

  const size_t state_dim;  // 变量说明：state_dim 表示状态、维度。
  const size_t input_dim;  // 变量说明：input_dim 表示输入、维度。
  const size_t base_dim;  // 变量说明：base_dim 表示浮动基/机身、维度。
  const size_t gen_coordinates_dim;  // 变量说明：gen_coordinates_dim 表示gen、coordinates、维度。
};

}  // namespace ocs2::humanoid
