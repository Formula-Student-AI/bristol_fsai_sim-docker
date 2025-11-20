// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eufs_msgs:msg/ConeAssociationArray.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__MSG__DETAIL__CONE_ASSOCIATION_ARRAY__TRAITS_HPP_
#define EUFS_MSGS__MSG__DETAIL__CONE_ASSOCIATION_ARRAY__TRAITS_HPP_

#include "eufs_msgs/msg/detail/cone_association_array__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'matched'
#include "eufs_msgs/msg/detail/cone_association__traits.hpp"
// Member 'unmatched'
#include "eufs_msgs/msg/detail/cone_with_covariance__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const eufs_msgs::msg::ConeAssociationArray & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: threshold
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "threshold: ";
    value_to_yaml(msg.threshold, out);
    out << "\n";
  }

  // member: matched
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.matched.size() == 0) {
      out << "matched: []\n";
    } else {
      out << "matched:\n";
      for (auto item : msg.matched) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: unmatched
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.unmatched.size() == 0) {
      out << "unmatched: []\n";
    } else {
      out << "unmatched:\n";
      for (auto item : msg.unmatched) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const eufs_msgs::msg::ConeAssociationArray & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<eufs_msgs::msg::ConeAssociationArray>()
{
  return "eufs_msgs::msg::ConeAssociationArray";
}

template<>
inline const char * name<eufs_msgs::msg::ConeAssociationArray>()
{
  return "eufs_msgs/msg/ConeAssociationArray";
}

template<>
struct has_fixed_size<eufs_msgs::msg::ConeAssociationArray>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<eufs_msgs::msg::ConeAssociationArray>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<eufs_msgs::msg::ConeAssociationArray>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // EUFS_MSGS__MSG__DETAIL__CONE_ASSOCIATION_ARRAY__TRAITS_HPP_
