// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from piper_msgs:srv/SetGripWidth.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__SET_GRIP_WIDTH__TRAITS_HPP_
#define PIPER_MSGS__SRV__DETAIL__SET_GRIP_WIDTH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "piper_msgs/srv/detail/set_grip_width__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace piper_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetGripWidth_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: state
  {
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetGripWidth_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "state: ";
    rosidl_generator_traits::value_to_yaml(msg.state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetGripWidth_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace piper_msgs

namespace rosidl_generator_traits
{

[[deprecated("use piper_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const piper_msgs::srv::SetGripWidth_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  piper_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use piper_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const piper_msgs::srv::SetGripWidth_Request & msg)
{
  return piper_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<piper_msgs::srv::SetGripWidth_Request>()
{
  return "piper_msgs::srv::SetGripWidth_Request";
}

template<>
inline const char * name<piper_msgs::srv::SetGripWidth_Request>()
{
  return "piper_msgs/srv/SetGripWidth_Request";
}

template<>
struct has_fixed_size<piper_msgs::srv::SetGripWidth_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<piper_msgs::srv::SetGripWidth_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<piper_msgs::srv::SetGripWidth_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace piper_msgs
{

namespace srv
{

inline void to_flow_style_yaml(
  const SetGripWidth_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: success
  {
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SetGripWidth_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: success
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "success: ";
    rosidl_generator_traits::value_to_yaml(msg.success, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SetGripWidth_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace piper_msgs

namespace rosidl_generator_traits
{

[[deprecated("use piper_msgs::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const piper_msgs::srv::SetGripWidth_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  piper_msgs::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use piper_msgs::srv::to_yaml() instead")]]
inline std::string to_yaml(const piper_msgs::srv::SetGripWidth_Response & msg)
{
  return piper_msgs::srv::to_yaml(msg);
}

template<>
inline const char * data_type<piper_msgs::srv::SetGripWidth_Response>()
{
  return "piper_msgs::srv::SetGripWidth_Response";
}

template<>
inline const char * name<piper_msgs::srv::SetGripWidth_Response>()
{
  return "piper_msgs/srv/SetGripWidth_Response";
}

template<>
struct has_fixed_size<piper_msgs::srv::SetGripWidth_Response>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<piper_msgs::srv::SetGripWidth_Response>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<piper_msgs::srv::SetGripWidth_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<piper_msgs::srv::SetGripWidth>()
{
  return "piper_msgs::srv::SetGripWidth";
}

template<>
inline const char * name<piper_msgs::srv::SetGripWidth>()
{
  return "piper_msgs/srv/SetGripWidth";
}

template<>
struct has_fixed_size<piper_msgs::srv::SetGripWidth>
  : std::integral_constant<
    bool,
    has_fixed_size<piper_msgs::srv::SetGripWidth_Request>::value &&
    has_fixed_size<piper_msgs::srv::SetGripWidth_Response>::value
  >
{
};

template<>
struct has_bounded_size<piper_msgs::srv::SetGripWidth>
  : std::integral_constant<
    bool,
    has_bounded_size<piper_msgs::srv::SetGripWidth_Request>::value &&
    has_bounded_size<piper_msgs::srv::SetGripWidth_Response>::value
  >
{
};

template<>
struct is_service<piper_msgs::srv::SetGripWidth>
  : std::true_type
{
};

template<>
struct is_service_request<piper_msgs::srv::SetGripWidth_Request>
  : std::true_type
{
};

template<>
struct is_service_response<piper_msgs::srv::SetGripWidth_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // PIPER_MSGS__SRV__DETAIL__SET_GRIP_WIDTH__TRAITS_HPP_
