#pragma once

#include "g1_kino_types.hpp"
#include "../g1_kinodynamics_nmpc.h"

#include <array>
#include <memory>
#include <vector>

#include <pinocchio/spatial/se3.hpp>

namespace g1_kino_detail
{

// 数据说明：所有 cost term 共享的 reference 和权重，problem 复用时只更新这里。
struct DirectKinoCostReferences
{
    Eigen::VectorXd xRef;
    Eigen::VectorXd uRef;
    std::vector<Eigen::VectorXd> uRefHorizon;
    std::array<pinocchio::SE3, G1KinodynamicsNmpc::kNumFeet> footPoseReference{
        pinocchio::SE3::Identity(), pinocchio::SE3::Identity()};
    std::vector<std::array<pinocchio::SE3, G1KinodynamicsNmpc::kNumFeet>> footPoseReferenceHorizon;
    Eigen::MatrixXd stateWeight;
    Eigen::MatrixXd controlWeight;
    Eigen::MatrixXd terminalWeight;
    Eigen::Matrix<double, 6, 6> centroidalWeight = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> centroidalDerivativeWeight = Eigen::Matrix<double, 6, 6>::Zero();
    Eigen::Matrix<double, 6, 6> footPoseWeight = Eigen::Matrix<double, 6, 6>::Zero();
};

struct DirectKinoRunningCostData;
struct DirectKinoTerminalCostData;

// 类说明：单个 shooting node 的直接 packed cost，不再走 CostStack/QuadraticResidualCost。
class DirectKinoRunningCost final : public CostAbstract
{
public:
    // 函数说明：绑定 knot 编号、reference、Pinocchio 模型和接触状态。
    DirectKinoRunningCost(const Space &space,
                          int nu,
                          int knot,
                          std::shared_ptr<DirectKinoCostReferences> references,
                          const pinocchio::Model &model,
                          const Eigen::Vector3d &gravity,
                          const std::vector<bool> &contactStates,
                          const std::vector<pinocchio::FrameIndex> &contactIds,
                          const std::array<pinocchio::FrameIndex, G1KinodynamicsNmpc::kNumFeet> &footFrameIds);

    // 函数说明：计算 running cost value。
    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override;

    // 函数说明：同一次打包计算里已经同时写入梯度。
    void computeGradients(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override;

    // 函数说明：Hessian 在 evaluatePack 中按 Gauss-Newton 一并写入，这里无需重复计算。
    void computeHessians(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override;

    // 函数说明：创建当前 cost 所需的 residual data。
    std::shared_ptr<CostData> createData() const override;

private:
    // 函数说明：按 state/control/centroidal/foot pose 顺序直接累加 cost、梯度和 Hessian。
    void evaluatePack(const ConstVectorRef &x, const ConstVectorRef &u, CostData &rawData) const;

    std::shared_ptr<DirectKinoCostReferences> references_;
    int knot_{0};
    CentroidalMomentumResidual centroidalResidual_;
    CentroidalMomentumDerivativeResidual centroidalDerivativeResidual_;
    mutable std::array<FramePlacementResidual, G1KinodynamicsNmpc::kNumFeet> footPoseResiduals_;
};

// 类说明：终端 cost，跟踪末端状态并抑制末端 centroidal momentum。
class DirectKinoTerminalCost final : public CostAbstract
{
public:
    // 函数说明：绑定终端 reference 和 centroidal residual。
    DirectKinoTerminalCost(const Space &space,
                           int nu,
                           std::shared_ptr<DirectKinoCostReferences> references,
                           const pinocchio::Model &model);

    // 函数说明：计算 terminal cost value。
    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override;

    // 函数说明：同一次打包计算里已经同时写入梯度。
    void computeGradients(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override;

    // 函数说明：Hessian 在 evaluatePack 中按 Gauss-Newton 一并写入。
    void computeHessians(const ConstVectorRef &x, const ConstVectorRef &u, CostData &data) const override;

    // 函数说明：创建 terminal cost data。
    std::shared_ptr<CostData> createData() const override;

private:
    // 函数说明：累加 terminal state cost 和 terminal centroidal cost。
    void evaluatePack(const ConstVectorRef &x, const ConstVectorRef &u, CostData &rawData) const;

    std::shared_ptr<DirectKinoCostReferences> references_;
    CentroidalMomentumResidual centroidalResidual_;
};

// 函数说明：输入向量尺寸或数值异常时返回指定尺寸的零向量。
Eigen::VectorXd safeVectorOrZero(const Eigen::VectorXd &value, int size);

} // namespace g1_kino_detail
