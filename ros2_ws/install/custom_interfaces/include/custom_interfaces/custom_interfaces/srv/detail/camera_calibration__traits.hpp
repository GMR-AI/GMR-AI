// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from custom_interfaces:srv/CameraCalibration.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__CAMERA_CALIBRATION__TRAITS_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__CAMERA_CALIBRATION__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "custom_interfaces/srv/detail/camera_calibration__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CameraCalibration_Request & msg,
  std::ostream & out)
{
  (void)msg;
  out << "null";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CameraCalibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CameraCalibration_Request & msg, bool use_flow_style = false)
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

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::srv::CameraCalibration_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::srv::CameraCalibration_Request & msg)
{
  return custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::srv::CameraCalibration_Request>()
{
  return "custom_interfaces::srv::CameraCalibration_Request";
}

template<>
inline const char * name<custom_interfaces::srv::CameraCalibration_Request>()
{
  return "custom_interfaces/srv/CameraCalibration_Request";
}

template<>
struct has_fixed_size<custom_interfaces::srv::CameraCalibration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<custom_interfaces::srv::CameraCalibration_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<custom_interfaces::srv::CameraCalibration_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'cam_info'
#include "sensor_msgs/msg/detail/camera_info__traits.hpp"

namespace custom_interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const CameraCalibration_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: cam_info
  {
    if (msg.cam_info.size() == 0) {
      out << "cam_info: []";
    } else {
      out << "cam_info: [";
      size_t pending_items = msg.cam_info.size();
      for (auto item : msg.cam_info) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CameraCalibration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cam_info
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.cam_info.size() == 0) {
      out << "cam_info: []\n";
    } else {
      out << "cam_info:\n";
      for (auto item : msg.cam_info) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CameraCalibration_Response & msg, bool use_flow_style = false)
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

}  // namespace custom_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use custom_interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const custom_interfaces::srv::CameraCalibration_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  custom_interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use custom_interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const custom_interfaces::srv::CameraCalibration_Response & msg)
{
  return custom_interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<custom_interfaces::srv::CameraCalibration_Response>()
{
  return "custom_interfaces::srv::CameraCalibration_Response";
}

template<>
inline const char * name<custom_interfaces::srv::CameraCalibration_Response>()
{
  return "custom_interfaces/srv/CameraCalibration_Response";
}

template<>
struct has_fixed_size<custom_interfaces::srv::CameraCalibration_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<custom_interfaces::srv::CameraCalibration_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<custom_interfaces::srv::CameraCalibration_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<custom_interfaces::srv::CameraCalibration>()
{
  return "custom_interfaces::srv::CameraCalibration";
}

template<>
inline const char * name<custom_interfaces::srv::CameraCalibration>()
{
  return "custom_interfaces/srv/CameraCalibration";
}

template<>
struct has_fixed_size<custom_interfaces::srv::CameraCalibration>
  : std::integral_constant<
    bool,
    has_fixed_size<custom_interfaces::srv::CameraCalibration_Request>::value &&
    has_fixed_size<custom_interfaces::srv::CameraCalibration_Response>::value
  >
{
};

template<>
struct has_bounded_size<custom_interfaces::srv::CameraCalibration>
  : std::integral_constant<
    bool,
    has_bounded_size<custom_interfaces::srv::CameraCalibration_Request>::value &&
    has_bounded_size<custom_interfaces::srv::CameraCalibration_Response>::value
  >
{
};

template<>
struct is_service<custom_interfaces::srv::CameraCalibration>
  : std::true_type
{
};

template<>
struct is_service_request<custom_interfaces::srv::CameraCalibration_Request>
  : std::true_type
{
};

template<>
struct is_service_response<custom_interfaces::srv::CameraCalibration_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__CAMERA_CALIBRATION__TRAITS_HPP_
