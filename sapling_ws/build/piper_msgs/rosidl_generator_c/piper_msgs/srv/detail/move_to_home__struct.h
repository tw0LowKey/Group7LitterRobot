// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from piper_msgs:srv/MoveToHome.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__MOVE_TO_HOME__STRUCT_H_
#define PIPER_MSGS__SRV__DETAIL__MOVE_TO_HOME__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/MoveToHome in the package piper_msgs.
typedef struct piper_msgs__srv__MoveToHome_Request
{
  uint8_t structure_needs_at_least_one_member;
} piper_msgs__srv__MoveToHome_Request;

// Struct for a sequence of piper_msgs__srv__MoveToHome_Request.
typedef struct piper_msgs__srv__MoveToHome_Request__Sequence
{
  piper_msgs__srv__MoveToHome_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__MoveToHome_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MoveToHome in the package piper_msgs.
typedef struct piper_msgs__srv__MoveToHome_Response
{
  bool success;
} piper_msgs__srv__MoveToHome_Response;

// Struct for a sequence of piper_msgs__srv__MoveToHome_Response.
typedef struct piper_msgs__srv__MoveToHome_Response__Sequence
{
  piper_msgs__srv__MoveToHome_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} piper_msgs__srv__MoveToHome_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PIPER_MSGS__SRV__DETAIL__MOVE_TO_HOME__STRUCT_H_
