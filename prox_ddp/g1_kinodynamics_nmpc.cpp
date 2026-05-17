#include "g1_kinodynamics_nmpc.h"

#include "detail/solver_cache.hpp"

#include <algorithm>

// 函数说明：构造 NMPC wrapper，确定控制维度并分配输出缓存。
G1KinodynamicsNmpc::G1KinodynamicsNmpc(
    const pinocchio::Model &model,
    const std::array<pinocchio::FrameIndex, kNumFeet> &footFrameIds,
    int horizon,
    double dt)
    : model_(model),
      footFrameIds_(footFrameIds),
      horizon_(std::max(1, horizon)),
      dt_(std::max(1e-4, dt)),
      nu_(model.nv - 6 + kNumFeet * kForceSize)
{
    firstJointAccelerations_ = Eigen::VectorXd::Zero(std::max(0, model_.nv - 6));
    firstState_ = Eigen::VectorXd::Zero(model_.nq + model_.nv);
    firstControl_ = Eigen::VectorXd::Zero(nu_);
}

// 函数说明：默认析构，unique_ptr 会自动释放 solver cache。
G1KinodynamicsNmpc::~G1KinodynamicsNmpc() = default;
