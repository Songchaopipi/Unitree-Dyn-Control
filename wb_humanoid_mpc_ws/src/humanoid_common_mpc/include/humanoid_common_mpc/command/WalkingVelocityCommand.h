
#pragma once
// 自动中文注释：src/humanoid_common_mpc/include/humanoid_common_mpc/command/WalkingVelocityCommand.h 用于本工程 MPC/仿真链路，下面变量和函数均按用途补充中文说明。

#include <algorithm>

#include "humanoid_common_mpc/common/Types.h"

namespace ocs2::humanoid {

struct WalkingVelocityCommand {
 public:
  WalkingVelocityCommand() = default;
  WalkingVelocityCommand(scalar_t v_x, scalar_t v_y, scalar_t desired_pelvis_h, scalar_t v_yaw)
      : linear_velocity_x(v_x), linear_velocity_y(v_y), desired_pelvis_height(desired_pelvis_h), angular_velocity_z(v_yaw){};
  WalkingVelocityCommand(const vector4_t& velCommand)
      // 函数说明：处理linear、速度、x，连接当前模块的数据流和控制逻辑。
      : linear_velocity_x(velCommand(0)),
        linear_velocity_y(velCommand(1)),
        desired_pelvis_height(velCommand(2)),
        angular_velocity_z(velCommand(3)){};
  scalar_t linear_velocity_x = 0.0;  // 变量说明：linear_velocity_x 表示linear、速度、x。
  scalar_t linear_velocity_y = 0.0;  // 变量说明：linear_velocity_y 表示linear、速度、y。
  scalar_t desired_pelvis_height = 0.8;  // Above ground
  scalar_t angular_velocity_z = 0.0;  // 变量说明：angular_velocity_z 表示angular、速度、z。

  // 函数说明：设置to、默认、命令，连接当前模块的数据流和控制逻辑。
  void setToDefaultCommand() {
    linear_velocity_x = 0.0;
    linear_velocity_y = 0.0;
    desired_pelvis_height = 0.8;  // Above ground
    angular_velocity_z = 0.0;
  }

  vector4_t toVector() { return vector4_t(linear_velocity_x, linear_velocity_y, desired_pelvis_height, angular_velocity_z); };
};

}  // namespace ocs2::humanoid
