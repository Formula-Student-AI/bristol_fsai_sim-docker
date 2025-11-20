// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eufs_msgs:msg/ConeWithCovariancePlus.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__TRAITS_HPP_
#define EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__TRAITS_HPP_

#include "eufs_msgs/msg/detail/cone_with_covariance_plus__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'point'
#include "geometry_msgs/msg/detail/point__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const eufs_msgs::msg::ConeWithCovariancePlus & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: blue_prob
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "blue_prob: ";
    value_to_yaml(msg.blue_prob, out);
    out << "\n";
  }

  // member: yellow_prob
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yellow_prob: ";
    value_to_yaml(msg.yellow_prob, out);
    out << "\n";
  }

  // member: orange_prob
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "orange_prob: ";
    value_to_yaml(msg.orange_prob, out);
    out << "\n";
  }

  // member: big_orange_prob
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "big_orange_prob: ";
    value_to_yaml(msg.big_orange_prob, out);
    out << "\n";
  }

  // member: unknown_prob
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "unknown_prob: ";
    value_to_yaml(msg.unknown_prob, out);
    out << "\n";
  }

  // member: point
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "point:\n";
    to_yaml(msg.point, out, indentation + 2);
  }

  // member: covariance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.covariance.size() == 0) {
      out << "covariance: []\n";
    } else {
      out << "covariance:\n";
      for (auto item : msg.covariance) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const eufs_msgs::msg::ConeWithCovariancePlus & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<eufs_msgs::msg::ConeWithCovariancePlus>()
{
  return "eufs_msgs::msg::ConeWithCovariancePlus";
}

template<>
inline const char * name<eufs_msgs::msg::ConeWithCovariancePlus>()
{
  return "eufs_msgs/msg/ConeWithCovariancePlus";
}

template<>
struct has_fixed_size<eufs_msgs::msg::ConeWithCovariancePlus>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Point>::value> {};

template<>
struct has_bounded_size<eufs_msgs::msg::ConeWithCovariancePlus>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Point>::value> {};

template<>
struct is_message<eufs_msgs::msg::ConeWithCovariancePlus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_WITH_COVARIANCE_PLUS__TRAITS_HPP_
