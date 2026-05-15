#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <vector>

#include "qpOASES.hpp"
#include "data_bus.h"

class G1SrbdMpc
{
public:
    struct Input
    {
        Eigen::Matrix<double, 13, 1> current = Eigen::Matrix<double, 13, 1>::Zero();
        Eigen::Quaterniond orientation = Eigen::Quaterniond::Identity();
        std::array<Eigen::Vector3d, 2> contactPositionWorld{};
        std::vector<std::array<Eigen::Vector3d, 2>> contactPositionWorldHorizon;
        Eigen::Matrix<double, 13, Eigen::Dynamic> reference;
        Eigen::Matrix<int, Eigen::Dynamic, 2> contactTable;
        double mass{1.0};
        Eigen::Matrix3d inertiaBody = Eigen::Matrix3d::Identity();
    };

    explicit G1SrbdMpc(int horizon = 10, double dt = 0.04);

    bool solve(const Input &input);
    bool solveFromDataBus(const DataBus &robotState, double mass, const Eigen::Matrix3d &inertiaBody);
    void dataBusWrite(DataBus &robotState) const;

    Eigen::Matrix<double, 12, 1> firstPlaneForces() const { return firstPlaneForces_; }
    Eigen::Matrix<double, 12, 1> firstWrenches() const { return firstWrenches_; }
    int status() const { return qpStatus_; }
    int horizon() const { return horizon_; }
    double dt() const { return dt_; }

    double mu{1.0};
    double fMax{1400.0};
    double fzLow{20.0};
    double footFront{0.1};
    double footBack{-0.04};
    double footHalfWidth{0.025};
    double yawFriction{0.046};
    double nominalForceWeight{1.0};

private:
    int horizon_{10};
    double dt_{0.04};
    int qpStatus_{0};
    Eigen::Matrix<double, 12, 1> firstPlaneForces_ = Eigen::Matrix<double, 12, 1>::Zero();
    Eigen::Matrix<double, 12, 1> firstWrenches_ = Eigen::Matrix<double, 12, 1>::Zero();

    Input buildInputFromDataBus(const DataBus &robotState, double mass, const Eigen::Matrix3d &inertiaBody) const;
    void buildDiscreteModel(const Input &input,
                            int knot,
                            Eigen::Matrix<double, 13, 13> &A,
                            Eigen::Matrix<double, 13, 12> &B) const;
    void copyEigenToReal(qpOASES::real_t *target, const Eigen::MatrixXd &source) const;
    void copyEigenToReal(qpOASES::real_t *target, const Eigen::VectorXd &source) const;
};
