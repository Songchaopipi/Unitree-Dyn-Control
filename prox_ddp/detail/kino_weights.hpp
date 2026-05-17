#pragma once

#include <Eigen/Dense>

namespace g1_kino_detail
{

// 函数说明：生成完整 q/v 状态跟踪权重，腿部、腰部、上身使用不同尺度。
Eigen::VectorXd makeStateWeightDiagonal(int nv);

// 函数说明：生成控制权重，wrench 很轻、腿部关节加速度略轻、其他关节加速度更重。
Eigen::MatrixXd makeControlWeight(int nu, int nv);

} // namespace g1_kino_detail
