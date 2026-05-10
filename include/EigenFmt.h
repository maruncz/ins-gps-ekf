#ifndef EKF_EIGEN_FMT_H
#define EKF_EIGEN_FMT_H

#include <Eigen/Core>
#include <fmt/format.h>
#include <sstream>
#include <type_traits>

namespace fmt {

namespace detail {

template <typename T, typename = void>
struct is_eigen : std::false_type {};

template <typename T>
struct is_eigen<T, std::enable_if_t<std::is_base_of_v<Eigen::EigenBase<std::remove_cv_t<T>>, std::remove_cv_t<T>>>>
    : std::true_type {};

} // namespace detail

template <typename T, typename Char>
struct formatter<T, Char, std::enable_if_t<detail::is_eigen<T>::value>>
    : formatter<std::string> {
  auto format(const T& mat, format_context& ctx) const {
    Eigen::IOFormat fmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n", "[", "]");
    std::stringstream ss;
    ss << mat.format(fmt);
    return formatter<std::string>::format(ss.str(), ctx);
  }
};

} // namespace fmt

#endif // EKF_EIGEN_FMT_H
