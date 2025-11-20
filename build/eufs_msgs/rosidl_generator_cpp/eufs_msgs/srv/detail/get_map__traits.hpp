// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from eufs_msgs:srv/GetMap.idl
// generated code does not contain a copyright notice

#ifndef EUFS_MSGS__SRV__DETAIL__GET_MAP__TRAITS_HPP_
#define EUFS_MSGS__SRV__DETAIL__GET_MAP__TRAITS_HPP_

#include "eufs_msgs/srv/detail/get_map__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const eufs_msgs::srv::GetMap_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const eufs_msgs::srv::GetMap_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<eufs_msgs::srv::GetMap_Request>()
{
  return "eufs_msgs::srv::GetMap_Request";
}

template<>
inline const char * name<eufs_msgs::srv::GetMap_Request>()
{
  return "eufs_msgs/srv/GetMap_Request";
}

template<>
struct has_fixed_size<eufs_msgs::srv::GetMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<eufs_msgs::srv::GetMap_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<eufs_msgs::srv::GetMap_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const eufs_msgs::srv::GetMap_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: map_path
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.map_path.size() == 0) {
      out << "map_path: []\n";
    } else {
      out << "map_path:\n";
      for (auto item : msg.map_path) {
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

inline std::string to_yaml(const eufs_msgs::srv::GetMap_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<eufs_msgs::srv::GetMap_Response>()
{
  return "eufs_msgs::srv::GetMap_Response";
}

template<>
inline const char * name<eufs_msgs::srv::GetMap_Response>()
{
  return "eufs_msgs/srv/GetMap_Response";
}

template<>
struct has_fixed_size<eufs_msgs::srv::GetMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<eufs_msgs::srv::GetMap_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<eufs_msgs::srv::GetMap_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<eufs_msgs::srv::GetMap>()
{
  return "eufs_msgs::srv::GetMap";
}

template<>
inline const char * name<eufs_msgs::srv::GetMap>()
{
  return "eufs_msgs/srv/GetMap";
}

template<>
struct has_fixed_size<eufs_msgs::srv::GetMap>
  : std::integral_constant<
    bool,
    has_fixed_size<eufs_msgs::srv::GetMap_Request>::value &&
    has_fixed_size<eufs_msgs::srv::GetMap_Response>::value
  >
{
};

template<>
struct has_bounded_size<eufs_msgs::srv::GetMap>
  : std::integral_constant<
    bool,
    has_bounded_size<eufs_msgs::srv::GetMap_Request>::value &&
    has_bounded_size<eufs_msgs::srv::GetMap_Response>::value
  >
{
};

template<>
struct is_service<eufs_msgs::srv::GetMap>
  : std::true_type
{
};

template<>
struct is_service_request<eufs_msgs::srv::GetMap_Request>
  : std::true_type
{
};

template<>
struct is_service_response<eufs_msgs::srv::GetMap_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // EUFS_MSGS__SRV__DETAIL__GET_MAP__TRAITS_HPP_
