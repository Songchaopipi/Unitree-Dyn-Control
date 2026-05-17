#pragma once

#include "g1_kino_types.hpp"

namespace g1_kino_detail
{

// 类说明：从完整控制向量中切出一段，用来给 wrench/关节加速度加 box 约束。
class ControlSliceResidual final : public aligator::StageFunctionTpl<double>
{
public:
    using Base = aligator::StageFunctionTpl<double>;
    using Data = aligator::StageFunctionDataTpl<double>;

    // 函数说明：记录要切片的控制维度、起点和长度。
    ControlSliceResidual(int ndx, int nu, int offset, int size);

    // 函数说明：约束残差就是 u 的指定片段。
    void evaluate(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;

    // 函数说明：切片残差对 x 无导数，对 u 是对应列的单位阵。
    void computeJacobians(const ConstVectorRef &x, const ConstVectorRef &u, Data &data) const override;

private:
    int offset_{0};
    int size_{0};
};

} // namespace g1_kino_detail
