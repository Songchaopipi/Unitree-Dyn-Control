#include "kino_weights.hpp"
#include "../g1_kinodynamics_nmpc.h"

#include <algorithm>

namespace g1_kino_detail
{

Eigen::VectorXd makeStateWeightDiagonal(int nv)
{
    Eigen::VectorXd w = Eigen::VectorXd::Zero(2 * nv);
    if (nv < 6)
    {
        return w;
    }

    w.segment<3>(0) << 50.0, 50.0, 150.0;
    w.segment<3>(3) << 250.0, 250.0, 120.0;
    const int actuated = nv - 6;
    for (int i = 0; i < actuated; ++i)
    {
        const int id = 6 + i;
        if (i < 12)
        {
            w(id) = 1.0;
        }
        else if (i < 15)
        {
            w(id) = 20.0;
        }
        else
        {
            w(id) = 2.0;
        }
    }

    // 行走时水平速度参考直接决定接触切向力，权重太小会导致 NMPC 只看位置误差但不主动追速度。
    w.segment<3>(nv) << 25.0, 25.0, 8.0;
    w.segment<3>(nv + 3) << 20.0, 20.0, 8.0;
    for (int i = 0; i < actuated; ++i)
    {
        const int id = nv + 6 + i;
        w(id) = (i < 12) ? 0.2 : 1.0;
    }
    return w;
}

Eigen::MatrixXd makeControlWeight(int nu, int nv)
{
    Eigen::MatrixXd w = Eigen::MatrixXd::Zero(nu, nu);
    for (int foot = 0; foot < G1KinodynamicsNmpc::kNumFeet; ++foot)
    {
        const int offset = foot * G1KinodynamicsNmpc::kForceSize;
        w.diagonal().segment<6>(offset) << 2e-4, 2e-4, 2e-5, 2e-2, 2e-2, 2e-2;
    }
    const int accOffset = G1KinodynamicsNmpc::kNumFeet * G1KinodynamicsNmpc::kForceSize;
    if (nu > accOffset)
    {
        w.diagonal().segment(accOffset, nu - accOffset).setConstant(5e-1);
        const int legCount = std::min(12, nv - 6);
        if (legCount > 0)
        {
            w.diagonal().segment(accOffset, legCount).setConstant(2e-1);
        }
    }
    return w;
}

} // namespace g1_kino_detail
