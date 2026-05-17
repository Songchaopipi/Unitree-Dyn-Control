#pragma once

#include "g1_kino_types.hpp"

#include <memory>

#include <aligator/core/constraint-set.hpp>
#include <aligator/modelling/centroidal/centroidal-wrench-cone.hpp>
#include <aligator/modelling/multibody/frame-velocity.hpp>

namespace g1_kino_detail
{

// 数据说明：一个 stage/foot 的接触激活量。true 表示支撑，false 表示摆动。
struct ContactActivation
{
    bool active{true};
};

// 数据说明：可在线更新的 box 上下界，用于避免接触切换时重建 constraint set。
struct MutableBoxBounds
{
    // 函数说明：按指定维度分配上下界。
    explicit MutableBoxBounds(int dim);

    Eigen::VectorXd lower;
    Eigen::VectorXd upper;
};

// 类说明：上下界由 shared bounds 提供，problem 复用时只改 bounds 数值。
class MutableBoxConstraint final : public aligator::ConstraintSetTpl<double>
{
public:
    using Base = aligator::ConstraintSetTpl<double>;
    using ActiveType = typename Base::ActiveType;

    // 函数说明：绑定共享上下界。
    explicit MutableBoxConstraint(std::shared_ptr<MutableBoxBounds> bounds);

    // 函数说明：投影到当前上下界 box。
    void projection(const ConstVectorRef &z, VectorRef zout) const override;

    // 函数说明：计算 normal cone 投影。
    void normalConeProjection(const ConstVectorRef &z, VectorRef zout) const override;

    // 函数说明：判断当前 z 是否越界。
    void computeActiveSet(const ConstVectorRef &z, Eigen::Ref<ActiveType> out) const override;

private:
    std::shared_ptr<MutableBoxBounds> bounds_;
};

// 类说明：带接触 mask 的 wrench cone。摆动脚时返回严格可行的常量 residual。
class MaskedWrenchConeResidual final : public aligator::StageFunctionTpl<double>
{
public:
    using Base = aligator::StageFunctionTpl<double>;
    using Data = aligator::StageFunctionDataTpl<double>;

    // 函数说明：绑定标准 cone residual 和共享接触激活量。
    MaskedWrenchConeResidual(int ndx,
                             int nu,
                             int foot,
                             double mu,
                             double footHalfLength,
                             double footHalfWidth,
                             std::shared_ptr<ContactActivation> activation);

    // 函数说明：支撑脚计算真实 cone；摆动脚给出严格可行 residual。
    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;

    // 函数说明：支撑脚计算 cone Jacobian；摆动脚 Jacobian 清零。
    void computeJacobians(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;

    // 函数说明：复用标准 cone residual 的 data。
    std::shared_ptr<Data> createData() const override;

private:
    aligator::CentroidalWrenchConeResidualTpl<double> residual_;
    std::shared_ptr<ContactActivation> activation_;
};

// 类说明：带接触 mask 的足端零速度等式。摆动脚时 residual/Jacobian 都为零。
class MaskedFrameVelocityResidual final : public aligator::StageFunctionTpl<double>
{
public:
    using Base = aligator::StageFunctionTpl<double>;
    using Data = aligator::StageFunctionDataTpl<double>;

    // 函数说明：绑定标准 frame velocity residual 和共享接触激活量。
    MaskedFrameVelocityResidual(int ndx,
                                int nu,
                                const pinocchio::Model &model,
                                const pinocchio::Motion &velocity,
                                pinocchio::FrameIndex frameId,
                                pinocchio::ReferenceFrame type,
                                std::shared_ptr<ContactActivation> activation);

    // 函数说明：支撑脚计算足端速度；摆动脚返回零。
    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;

    // 函数说明：支撑脚计算足端速度 Jacobian；摆动脚 Jacobian 清零。
    void computeJacobians(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;

    // 函数说明：复用标准 frame velocity residual 的 Pinocchio data。
    std::shared_ptr<Data> createData() const override;

private:
    aligator::FrameVelocityResidualTpl<double> residual_;
    std::shared_ptr<ContactActivation> activation_;
};

} // namespace g1_kino_detail
