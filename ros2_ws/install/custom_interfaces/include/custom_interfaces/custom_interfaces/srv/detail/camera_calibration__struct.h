// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:srv/CameraCalibration.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__SRV__DETAIL__CAMERA_CALIBRATION__STRUCT_H_
#define CUSTOM_INTERFACES__SRV__DETAIL__CAMERA_CALIBRATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/CameraCalibration in the package custom_interfaces.
typedef struct custom_interfaces__srv__CameraCalibration_Request
{
  uint8_t structure_needs_at_least_one_member;
} custom_interfaces__srv__CameraCalibration_Request;

// Struct for a sequence of custom_interfaces__srv__CameraCalibration_Request.
typedef struct custom_interfaces__srv__CameraCalibration_Request__Sequence
{
  custom_interfaces__srv__CameraCalibration_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__CameraCalibration_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'cam_info'
#include "sensor_msgs/msg/detail/camera_info__struct.h"

/// Struct defined in srv/CameraCalibration in the package custom_interfaces.
typedef struct custom_interfaces__srv__CameraCalibration_Response
{
  sensor_msgs__msg__CameraInfo__Sequence cam_info;
} custom_interfaces__srv__CameraCalibration_Response;

// Struct for a sequence of custom_interfaces__srv__CameraCalibration_Response.
typedef struct custom_interfaces__srv__CameraCalibration_Response__Sequence
{
  custom_interfaces__srv__CameraCalibration_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__srv__CameraCalibration_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__SRV__DETAIL__CAMERA_CALIBRATION__STRUCT_H_
