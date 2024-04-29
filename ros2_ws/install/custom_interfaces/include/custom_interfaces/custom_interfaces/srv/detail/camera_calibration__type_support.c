// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from custom_interfaces:srv/CameraCalibration.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "custom_interfaces/srv/detail/camera_calibration__rosidl_typesupport_introspection_c.h"
#include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "custom_interfaces/srv/detail/camera_calibration__functions.h"
#include "custom_interfaces/srv/detail/camera_calibration__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__srv__CameraCalibration_Request__init(message_memory);
}

void custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_fini_function(void * message_memory)
{
  custom_interfaces__srv__CameraCalibration_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__srv__CameraCalibration_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_message_members = {
  "custom_interfaces__srv",  // message namespace
  "CameraCalibration_Request",  // message name
  1,  // number of fields
  sizeof(custom_interfaces__srv__CameraCalibration_Request),
  custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_message_member_array,  // message members
  custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_message_type_support_handle = {
  0,
  &custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, srv, CameraCalibration_Request)() {
  if (!custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__srv__CameraCalibration_Request__rosidl_typesupport_introspection_c__CameraCalibration_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "custom_interfaces/srv/detail/camera_calibration__rosidl_typesupport_introspection_c.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "custom_interfaces/srv/detail/camera_calibration__functions.h"
// already included above
// #include "custom_interfaces/srv/detail/camera_calibration__struct.h"


// Include directives for member types
// Member `cam_info`
#include "sensor_msgs/msg/camera_info.h"
// Member `cam_info`
#include "sensor_msgs/msg/detail/camera_info__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  custom_interfaces__srv__CameraCalibration_Response__init(message_memory);
}

void custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_fini_function(void * message_memory)
{
  custom_interfaces__srv__CameraCalibration_Response__fini(message_memory);
}

size_t custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__size_function__CameraCalibration_Response__cam_info(
  const void * untyped_member)
{
  const sensor_msgs__msg__CameraInfo__Sequence * member =
    (const sensor_msgs__msg__CameraInfo__Sequence *)(untyped_member);
  return member->size;
}

const void * custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__get_const_function__CameraCalibration_Response__cam_info(
  const void * untyped_member, size_t index)
{
  const sensor_msgs__msg__CameraInfo__Sequence * member =
    (const sensor_msgs__msg__CameraInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void * custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__get_function__CameraCalibration_Response__cam_info(
  void * untyped_member, size_t index)
{
  sensor_msgs__msg__CameraInfo__Sequence * member =
    (sensor_msgs__msg__CameraInfo__Sequence *)(untyped_member);
  return &member->data[index];
}

void custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__fetch_function__CameraCalibration_Response__cam_info(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const sensor_msgs__msg__CameraInfo * item =
    ((const sensor_msgs__msg__CameraInfo *)
    custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__get_const_function__CameraCalibration_Response__cam_info(untyped_member, index));
  sensor_msgs__msg__CameraInfo * value =
    (sensor_msgs__msg__CameraInfo *)(untyped_value);
  *value = *item;
}

void custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__assign_function__CameraCalibration_Response__cam_info(
  void * untyped_member, size_t index, const void * untyped_value)
{
  sensor_msgs__msg__CameraInfo * item =
    ((sensor_msgs__msg__CameraInfo *)
    custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__get_function__CameraCalibration_Response__cam_info(untyped_member, index));
  const sensor_msgs__msg__CameraInfo * value =
    (const sensor_msgs__msg__CameraInfo *)(untyped_value);
  *item = *value;
}

bool custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__resize_function__CameraCalibration_Response__cam_info(
  void * untyped_member, size_t size)
{
  sensor_msgs__msg__CameraInfo__Sequence * member =
    (sensor_msgs__msg__CameraInfo__Sequence *)(untyped_member);
  sensor_msgs__msg__CameraInfo__Sequence__fini(member);
  return sensor_msgs__msg__CameraInfo__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_member_array[1] = {
  {
    "cam_info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(custom_interfaces__srv__CameraCalibration_Response, cam_info),  // bytes offset in struct
    NULL,  // default value
    custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__size_function__CameraCalibration_Response__cam_info,  // size() function pointer
    custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__get_const_function__CameraCalibration_Response__cam_info,  // get_const(index) function pointer
    custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__get_function__CameraCalibration_Response__cam_info,  // get(index) function pointer
    custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__fetch_function__CameraCalibration_Response__cam_info,  // fetch(index, &value) function pointer
    custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__assign_function__CameraCalibration_Response__cam_info,  // assign(index, value) function pointer
    custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__resize_function__CameraCalibration_Response__cam_info  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_members = {
  "custom_interfaces__srv",  // message namespace
  "CameraCalibration_Response",  // message name
  1,  // number of fields
  sizeof(custom_interfaces__srv__CameraCalibration_Response),
  custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_member_array,  // message members
  custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_type_support_handle = {
  0,
  &custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, srv, CameraCalibration_Response)() {
  custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, CameraInfo)();
  if (!custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_type_support_handle.typesupport_identifier) {
    custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &custom_interfaces__srv__CameraCalibration_Response__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "custom_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "custom_interfaces/srv/detail/camera_calibration__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers custom_interfaces__srv__detail__camera_calibration__rosidl_typesupport_introspection_c__CameraCalibration_service_members = {
  "custom_interfaces__srv",  // service namespace
  "CameraCalibration",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // custom_interfaces__srv__detail__camera_calibration__rosidl_typesupport_introspection_c__CameraCalibration_Request_message_type_support_handle,
  NULL  // response message
  // custom_interfaces__srv__detail__camera_calibration__rosidl_typesupport_introspection_c__CameraCalibration_Response_message_type_support_handle
};

static rosidl_service_type_support_t custom_interfaces__srv__detail__camera_calibration__rosidl_typesupport_introspection_c__CameraCalibration_service_type_support_handle = {
  0,
  &custom_interfaces__srv__detail__camera_calibration__rosidl_typesupport_introspection_c__CameraCalibration_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, srv, CameraCalibration_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, srv, CameraCalibration_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_custom_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, srv, CameraCalibration)() {
  if (!custom_interfaces__srv__detail__camera_calibration__rosidl_typesupport_introspection_c__CameraCalibration_service_type_support_handle.typesupport_identifier) {
    custom_interfaces__srv__detail__camera_calibration__rosidl_typesupport_introspection_c__CameraCalibration_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)custom_interfaces__srv__detail__camera_calibration__rosidl_typesupport_introspection_c__CameraCalibration_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, srv, CameraCalibration_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, custom_interfaces, srv, CameraCalibration_Response)()->data;
  }

  return &custom_interfaces__srv__detail__camera_calibration__rosidl_typesupport_introspection_c__CameraCalibration_service_type_support_handle;
}
