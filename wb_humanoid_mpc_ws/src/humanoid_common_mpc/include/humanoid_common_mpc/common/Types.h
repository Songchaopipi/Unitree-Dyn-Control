
#pragma once

#include <array>
#include <cstddef>

#include <ocs2_core/Types.h>
#include <ocs2_core/automatic_differentiation/Types.h>

namespace ocs2 {

template <typename SCALAR_T>
using VECTOR_T = Eigen::Matrix<SCALAR_T, -1, 1>;
template <typename SCALAR_T>
using VECTOR2_T = Eigen::Matrix<SCALAR_T, 2, 1>;
template <typename SCALAR_T>
using VECTOR3_T = Eigen::Matrix<SCALAR_T, 3, 1>;
template <typename SCALAR_T>
using VECTOR4_T = Eigen::Matrix<SCALAR_T, 4, 1>;
template <typename SCALAR_T>
using VECTOR6_T = Eigen::Matrix<SCALAR_T, 6, 1>;
template <typename SCALAR_T>
using VECTOR12_T = Eigen::Matrix<SCALAR_T, 12, 1>;
template <typename SCALAR_T>
using MATRIX_T = Eigen::Matrix<SCALAR_T, -1, -1>;
template <typename SCALAR_T>
using MATRIX3_T = Eigen::Matrix<SCALAR_T, 3, 3>;
template <typename SCALAR_T>
using MATRIX4_T = Eigen::Matrix<SCALAR_T, 4, 4>;
template <typename SCALAR_T>
using MATRIX6_T = Eigen::Matrix<SCALAR_T, 6, 6>;
template <typename SCALAR_T>
using QUATERNION_T = Eigen::Quaternion<SCALAR_T>;

using vector2_t = VECTOR2_T<scalar_t>;
using vector3_t = VECTOR3_T<scalar_t>;
using vector4_t = VECTOR4_T<scalar_t>;
using vector6_t = VECTOR6_T<scalar_t>;
using vector12_t = VECTOR12_T<scalar_t>;
using matrix3_t = MATRIX3_T<scalar_t>;
using matrix4_t = MATRIX4_T<scalar_t>;
using matrix6_t = MATRIX6_T<scalar_t>;
using quaternion_t = QUATERNION_T<scalar_t>;

using ad_vector2_t = VECTOR2_T<ad_scalar_t>;
using ad_vector3_t = VECTOR3_T<ad_scalar_t>;
using ad_vector4_t = VECTOR4_T<ad_scalar_t>;
using ad_vector6_t = VECTOR6_T<ad_scalar_t>;
using ad_vector12_t = VECTOR12_T<ad_scalar_t>;
using ad_matrix3_t = MATRIX3_T<ad_scalar_t>;
using ad_matrix4_t = MATRIX4_T<ad_scalar_t>;
using ad_matrix6_t = MATRIX6_T<ad_scalar_t>;
using ad_quaternion_t = QUATERNION_T<ad_scalar_t>;

/******************************************************************************************************/
/* Contacts definition

  Contact wrench F = [f_x, f_y, f_z, M_x, M_y, M_z]^T

*/
/******************************************************************************************************/

constexpr size_t N_CONTACTS = 2;  // 变量说明：N_CONTACTS 表示n、contacts。

template <typename T>
using feet_array_t = std::array<T, N_CONTACTS>;

template <typename T>
using feet_vec_t = std::vector<T>;

using contact_flag_t = feet_array_t<bool>;  // describes which feet are in contacts [left_contact, right_contact]

constexpr size_t CONTACT_WRENCH_DIM = 6;  // 变量说明：CONTACT_WRENCH_DIM 表示接触、力和力矩、维度。

}  // namespace ocs2
