#pragma once

#include "g1_kino_types.hpp"

#include <memory>
#include <vector>

#include <pinocchio/multibody/model.hpp>

namespace g1_kino_detail
{

struct DirectSemiImplicitKinodynamicsData;

// 类说明：把 Aligator 连续 kino-dyn 动力学包装成半隐式 Euler 离散模型。
class DirectSemiImplicitKinodynamics final : public ExplicitDynamicsModel
{
public:
    using Base = ExplicitDynamicsModel;
    using Data = ExplicitDynamicsData;

    // 函数说明：保存多体空间、Pinocchio 模型、接触状态和离散时间。
    DirectSemiImplicitKinodynamics(const Space &space,
                                   const pinocchio::Model &model,
                                   const Eigen::Vector3d &gravity,
                                   const std::vector<bool> &contactStates,
                                   const std::vector<pinocchio::FrameIndex> &contactIds,
                                   int forceSize,
                                   double dt);

    // 函数说明：计算离散一步 x_{k+1}=f(x_k,u_k)。
    void forward(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;

    // 函数说明：计算离散动力学一阶导数 Fx/Fu。
    void dForward(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;

    // 函数说明：创建保存连续动力学缓存和临时矩阵的数据对象。
    std::shared_ptr<Data> createData() const override;

    // 函数说明：暴露内部连续动力学，供 Data 构造时创建对应缓存。
    const KinodynamicsFwdDynamics &continuousDynamics() const { return ode_; }

private:
    // 函数说明：先用连续模型算 qdd，再按半隐式 Euler 积分速度和位置。
    void computeForwardState(const ConstVectorRef &x,
                             const ConstVectorRef &u,
                             DirectSemiImplicitKinodynamicsData &data,
                             ContinuousDynamicsData &continuousData) const;

    KinodynamicsFwdDynamics ode_;
    double timestep_{0.0};
};

} // namespace g1_kino_detail
