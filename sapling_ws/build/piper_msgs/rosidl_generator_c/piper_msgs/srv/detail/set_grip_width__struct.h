// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from piper_msgs:srv/SetGripWidth.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__SET_GRIP_WIDTH__STRUCT_H_
#define PIPER_MSGS__SRV__DETAIL__SET_GRIP_WIDTH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/SetGripWidth in the package piper_msgs.
typedef struct piper_msgs__srv__SetGripWidth_Request
{
  bool state;
} piper_msgs__srv__SetGripWidth_Request;

// Struct for a sequence of piper_msgs__srv__SetGripWidth_Request.
typedef struct piper_msgs__srv__SetGripWidth_Request__Sequence
{
  piper_msgs__srv__SetGripWidth_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__SetGripWidth_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetGripWidth in the package piper_msgs.
typedef struct piper_msgs__srv__SetGripWidth_Response
{
  bool success;
} piper_msgs__srv__SetGripWidth_Response;

// Struct for a sequence of piper_msgs__srv__SetGripWidth_Response.
typedef struct piper_msgs__srv__SetGripWidth_Response__Sequence
{
  piper_msgs__srv__SetGripWidth_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__SetGripWidth_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PIPER_MSGS__SRV__DETAIL__SET_GRIP_WIDTH__STRUCT_H_
