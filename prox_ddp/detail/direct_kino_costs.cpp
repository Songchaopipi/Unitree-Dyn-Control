#include "direct_kino_costs.hpp"

#include <algorithm>
#include <stdexcept>

namespace g1_kino_detail
{

namespace
{

// 函数说明：清空 Aligator cost data，避免上一次求值残留。
void resetCostData(CostDataAbstract &data)
{
    data.value_ = 0.0;
    data.grad_.setZero();
    data.hess_.setZero();
}

// 函数说明：清空 residual data 的 value/Jacobian/二阶缓存。
void resetResidualData(StageFunctionData &data)
{
    data.value_.setZero();
    data.jac_buffer_.setZero();
    data.vhp_buffer_.setZero();
}

// 函数说明：把带 x/u Jacobian 的二次残差累加到 value/gradient/Gauss-Newton Hessian。
void addQuadraticResidual(CostDataAbstract &data,
                          const Eigen::Ref<const Eigen::VectorXd> &residual,
                          const Eigen::Ref<const Eigen::MatrixXd> &Jx,
                          const Eigen::Ref<const Eigen::MatrixXd> &Ju,
                          const Eigen::Ref<const Eigen::MatrixXd> &weight)
{
    const Eigen::VectorXd weightedResidual = weight * residual;
    data.value_ += 0.5 * residual.dot(weightedResidual);
    data.Lx_.noalias() += Jx.transpose() * weightedResidual;
    data.Lu_.noalias() += Ju.transpose() * weightedResidual;

    const Eigen::MatrixXd weightedJx = weight * Jx;
    const Eigen::MatrixXd weightedJu = weight * Ju;
    data.Lxx_.noalias() += Jx.transpose() * weightedJx;
    data.Lxu_.noalias() += Jx.transpose() * weightedJu;
    data.Lux_.noalias() += Ju.transpose() * weightedJx;
    data.Luu_.noalias() += Ju.transpose() * weightedJu;
}

// 函数说明：把只依赖状态的二次残差累加到 cost data。
void addStateResidual(CostDataAbstract &data,
                      const Eigen::Ref<const Eigen::VectorXd> &residual,
                      const Eigen::Ref<const Eigen::MatrixXd> &Jx,
                      const Eigen::Ref<const Eigen::MatrixXd> &weight)
{
    const Eigen::VectorXd weightedResidual = weight * residual;
    data.value_ += 0.5 * residual.dot(weightedResidual);
    data.Lx_.noalias() += Jx.transpose() * weightedResidual;
    const Eigen::MatrixXd weightedJx = weight * Jx;
    data.Lxx_.noalias() += Jx.transpose() * weightedJx;
}

// 函数说明：把控制二次正则累加到 cost data。
void addControlResidual(CostDataAbstract &data,
                        const Eigen::Ref<const Eigen::VectorXd> &residual,
                        const Eigen::Ref<const Eigen::MatrixXd> &weight)
{
    const Eigen::VectorXd weightedResidual = weight * residual;
    data.value_ += 0.5 * residual.dot(weightedResidual);
    data.Lu_.noalias() += weightedResidual;
    data.Luu_.noalias() += weight;
}

} // namespace

// 数据说明：running cost 的状态、控制、动量和足端 residual 缓存。
struct DirectKinoRunningCostData final : CostDataAbstract
{
    // 函数说明：按 ndx/nu 和各 residual 维度分配缓存。
    DirectKinoRunningCostData(int ndx,
                              int nu,
                              const CentroidalMomentumResidual &centroidalResidual,
                              const CentroidalMomentumDerivativeResidual &centroidalDerivativeResidual,
                              const std::array<FramePlacementResidual, G1KinodynamicsNmpc::kNumFeet> &footPoseResiduals)
        : CostDataAbstract(ndx, nu),
          stateResidual(ndx),
          stateJacobian(ndx, ndx),
          controlResidual(nu),
          centroidalData(centroidalResidual.createData()),
          centroidalDerivativeData(centroidalDerivativeResidual.createData())
    {
        stateResidual.setZero();
        stateJacobian.setZero();
        controlResidual.setZero();
        for (int foot = 0; foot < G1KinodynamicsNmpc::kNumFeet; ++foot)
        {
            footPoseData[foot] = footPoseResiduals[foot].createData();
        }
    }

    Eigen::VectorXd stateResidual;
    Eigen::MatrixXd stateJacobian;
    Eigen::VectorXd controlResidual;
    std::shared_ptr<StageFunctionData> centroidalData;
    std::shared_ptr<StageFunctionData> centroidalDerivativeData;
    std::array<std::shared_ptr<StageFunctionData>, G1KinodynamicsNmpc::kNumFeet> footPoseData;
};

DirectKinoRunningCost::DirectKinoRunningCost(
    const Space &space,
    int nu,
    int knot,
    std::shared_ptr<DirectKinoCostReferences> references,
    const pinocchio::Model &model,
    const Eigen::Vector3d &gravity,
    const std::vector<bool> &contactStates,
    const std::vector<pinocchio::FrameIndex> &contactIds,
    const std::array<pinocchio::FrameIndex, G1KinodynamicsNmpc::kNumFeet> &footFrameIds)
    : CostAbstract(space, nu),
      references_(std::move(references)),
      knot_(knot),
      centroidalResidual_(space.ndx(), nu, model, Vector6d::Zero()),
      centroidalDerivativeResidual_(space.ndx(), model, gravity, contactStates, contactIds, G1KinodynamicsNmpc::kForceSize),
      footPoseResiduals_{{
          FramePlacementResidual(space.ndx(), nu, model, pinocchio::SE3::Identity(), footFrameIds[0]),
          FramePlacementResidual(space.ndx(), nu, model, pinocchio::SE3::Identity(), footFrameIds[1]),
      }}
{
    if (!references_)
    {
        throw std::invalid_argument("DirectKinoRunningCost requires references");
    }
}

void DirectKinoRunningCost::evaluate(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const
{
    evaluatePack(x, u, data);
}

void DirectKinoRunningCost::computeGradients(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const
{
    evaluatePack(x, u, data);
}

void DirectKinoRunningCost::computeHessians(const ConstVectorRef &, const ConstVectorRef &, CostData &) const {}

std::shared_ptr<DirectKinoRunningCost::CostData> DirectKinoRunningCost::createData() const
{
    return std::make_shared<DirectKinoRunningCostData>(
        ndx(), nu, centroidalResidual_, centroidalDerivativeResidual_, footPoseResiduals_);
}

void DirectKinoRunningCost::evaluatePack(const ConstVectorRef &x, const ConstVectorRef &u, CostData &rawData) const
{
    auto &data = static_cast<DirectKinoRunningCostData &>(rawData);
    resetCostData(data);

    this->space->difference(references_->xRef, x, data.stateResidual);
    this->space->Jdifference(references_->xRef, x, data.stateJacobian, 1);
    addStateResidual(data, data.stateResidual, data.stateJacobian, references_->stateWeight);

    const Eigen::VectorXd *uRef = &references_->uRef;
    if (!references_->uRefHorizon.empty())
    {
        const int row = std::clamp(knot_, 0, static_cast<int>(references_->uRefHorizon.size()) - 1);
        uRef = &references_->uRefHorizon[static_cast<std::size_t>(row)];
    }
    data.controlResidual = u - *uRef;
    addControlResidual(data, data.controlResidual, references_->controlWeight);

    resetResidualData(*data.centroidalData);
    centroidalResidual_.evaluate(x, *data.centroidalData);
    centroidalResidual_.computeJacobians(x, *data.centroidalData);
    addQuadraticResidual(data,
                         data.centroidalData->value_,
                         data.centroidalData->Jx_,
                         data.centroidalData->Ju_,
                         references_->centroidalWeight);

    resetResidualData(*data.centroidalDerivativeData);
    centroidalDerivativeResidual_.evaluate(x, u, *data.centroidalDerivativeData);
    centroidalDerivativeResidual_.computeJacobians(x, u, *data.centroidalDerivativeData);
    addQuadraticResidual(data,
                         data.centroidalDerivativeData->value_,
                         data.centroidalDerivativeData->Jx_,
                         data.centroidalDerivativeData->Ju_,
                         references_->centroidalDerivativeWeight);

    for (int foot = 0; foot < G1KinodynamicsNmpc::kNumFeet; ++foot)
    {
        const std::array<pinocchio::SE3, G1KinodynamicsNmpc::kNumFeet> *footPoseRef =
            &references_->footPoseReference;
        if (!references_->footPoseReferenceHorizon.empty())
        {
            const int row = std::clamp(
                knot_, 0, static_cast<int>(references_->footPoseReferenceHorizon.size()) - 1);
            footPoseRef = &references_->footPoseReferenceHorizon[static_cast<std::size_t>(row)];
        }
        footPoseResiduals_[foot].setReference((*footPoseRef)[foot]);
        resetResidualData(*data.footPoseData[foot]);
        footPoseResiduals_[foot].evaluate(x, *data.footPoseData[foot]);
        footPoseResiduals_[foot].computeJacobians(x, *data.footPoseData[foot]);
        addQuadraticResidual(data,
                             data.footPoseData[foot]->value_,
                             data.footPoseData[foot]->Jx_,
                             data.footPoseData[foot]->Ju_,
                             references_->footPoseWeight);
    }
}

// 数据说明：terminal cost 只需要状态残差和 centroidal momentum 残差。
struct DirectKinoTerminalCostData final : CostDataAbstract
{
    // 函数说明：按 terminal cost 需要的 residual 分配缓存。
    DirectKinoTerminalCostData(int ndx,
                               int nu,
                               const CentroidalMomentumResidual &centroidalResidual)
        : CostDataAbstract(ndx, nu),
          stateResidual(ndx),
          stateJacobian(ndx, ndx),
          centroidalData(centroidalResidual.createData())
    {
        stateResidual.setZero();
        stateJacobian.setZero();
    }

    Eigen::VectorXd stateResidual;
    Eigen::MatrixXd stateJacobian;
    std::shared_ptr<StageFunctionData> centroidalData;
};

DirectKinoTerminalCost::DirectKinoTerminalCost(
    const Space &space,
    int nu,
    std::shared_ptr<DirectKinoCostReferences> references,
    const pinocchio::Model &model)
    : CostAbstract(space, nu),
      references_(std::move(references)),
      centroidalResidual_(space.ndx(), nu, model, Vector6d::Zero())
{
    if (!references_)
    {
        throw std::invalid_argument("DirectKinoTerminalCost requires references");
    }
}

void DirectKinoTerminalCost::evaluate(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const
{
    evaluatePack(x, u, data);
}

void DirectKinoTerminalCost::computeGradients(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const
{
    evaluatePack(x, u, data);
}

void DirectKinoTerminalCost::computeHessians(const ConstVectorRef &, const ConstVectorRef &, CostData &) const {}

std::shared_ptr<DirectKinoTerminalCost::CostData> DirectKinoTerminalCost::createData() const
{
    return std::make_shared<DirectKinoTerminalCostData>(ndx(), nu, centroidalResidual_);
}

void DirectKinoTerminalCost::evaluatePack(const ConstVectorRef &x, const ConstVectorRef &u, CostData &rawData) const
{
    auto &data = static_cast<DirectKinoTerminalCostData &>(rawData);
    resetCostData(data);

    this->space->difference(references_->xRef, x, data.stateResidual);
    this->space->Jdifference(references_->xRef, x, data.stateJacobian, 1);
    addStateResidual(data, data.stateResidual, data.stateJacobian, references_->terminalWeight);

    resetResidualData(*data.centroidalData);
    centroidalResidual_.evaluate(x, *data.centroidalData);
    centroidalResidual_.computeJacobians(x, *data.centroidalData);
    addQuadraticResidual(data,
                         data.centroidalData->value_,
                         data.centroidalData->Jx_,
                         data.centroidalData->Ju_,
                         references_->centroidalWeight * 5.0);
}

Eigen::VectorXd safeVectorOrZero(const Eigen::VectorXd &value, int size)
{
    if (value.size() == size && value.allFinite())
    {
        return value;
    }
    return Eigen::VectorXd::Zero(size);
}

} // namespace g1_kino_detail
