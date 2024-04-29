// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:srv/CameraCalibration.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__CAMERA_CALIBRATION__BUILDER_HPP_
#define CUSTOM_INTERFACES__SRV__DETAIL__CAMERA_CALIBRATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/srv/detail/camera_calibration__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::CameraCalibration_Request>()
{
  return ::custom_interfaces::srv::CameraCalibration_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace custom_interfaces


namespace custom_interfaces
{

namespace srv
{

namespace builder
{

class Init_CameraCalibration_Response_cam_info
{
public:
  Init_CameraCalibration_Response_cam_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::custom_interfaces::srv::CameraCalibration_Response cam_info(::custom_interfaces::srv::CameraCalibration_Response::_cam_info_type arg)
  {
    msg_.cam_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::srv::CameraCalibration_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::srv::CameraCalibration_Response>()
{
  return custom_interfaces::srv::builder::Init_CameraCalibration_Response_cam_info();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__CAMERA_CALIBRATION__BUILDER_HPP_
