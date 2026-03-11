// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from piper_msgs:srv/MoveToPose.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__MOVE_TO_POSE__FUNCTIONS_H_
#define PIPER_MSGS__SRV__DETAIL__MOVE_TO_POSE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "piper_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "piper_msgs/srv/detail/move_to_pose__struct.h"

/// Initialize srv/MoveToPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * piper_msgs__srv__MoveToPose_Request
 * )) before or use
 * piper_msgs__srv__MoveToPose_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Request__init(piper_msgs__srv__MoveToPose_Request * msg);

/// Finalize srv/MoveToPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
void
piper_msgs__srv__MoveToPose_Request__fini(piper_msgs__srv__MoveToPose_Request * msg);

/// Create srv/MoveToPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * piper_msgs__srv__MoveToPose_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
piper_msgs__srv__MoveToPose_Request *
piper_msgs__srv__MoveToPose_Request__create();

/// Destroy srv/MoveToPose message.
/**
 * It calls
 * piper_msgs__srv__MoveToPose_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
void
piper_msgs__srv__MoveToPose_Request__destroy(piper_msgs__srv__MoveToPose_Request * msg);

/// Check for srv/MoveToPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Request__are_equal(const piper_msgs__srv__MoveToPose_Request * lhs, const piper_msgs__srv__MoveToPose_Request * rhs);

/// Copy a srv/MoveToPose message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Request__copy(
  const piper_msgs__srv__MoveToPose_Request * input,
  piper_msgs__srv__MoveToPose_Request * output);

/// Initialize array of srv/MoveToPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * piper_msgs__srv__MoveToPose_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Request__Sequence__init(piper_msgs__srv__MoveToPose_Request__Sequence * array, size_t size);

/// Finalize array of srv/MoveToPose messages.
/**
 * It calls
 * piper_msgs__srv__MoveToPose_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
void
piper_msgs__srv__MoveToPose_Request__Sequence__fini(piper_msgs__srv__MoveToPose_Request__Sequence * array);

/// Create array of srv/MoveToPose messages.
/**
 * It allocates the memory for the array and calls
 * piper_msgs__srv__MoveToPose_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
piper_msgs__srv__MoveToPose_Request__Sequence *
piper_msgs__srv__MoveToPose_Request__Sequence__create(size_t size);

/// Destroy array of srv/MoveToPose messages.
/**
 * It calls
 * piper_msgs__srv__MoveToPose_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
void
piper_msgs__srv__MoveToPose_Request__Sequence__destroy(piper_msgs__srv__MoveToPose_Request__Sequence * array);

/// Check for srv/MoveToPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Request__Sequence__are_equal(const piper_msgs__srv__MoveToPose_Request__Sequence * lhs, const piper_msgs__srv__MoveToPose_Request__Sequence * rhs);

/// Copy an array of srv/MoveToPose messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Request__Sequence__copy(
  const piper_msgs__srv__MoveToPose_Request__Sequence * input,
  piper_msgs__srv__MoveToPose_Request__Sequence * output);

/// Initialize srv/MoveToPose message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * piper_msgs__srv__MoveToPose_Response
 * )) before or use
 * piper_msgs__srv__MoveToPose_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Response__init(piper_msgs__srv__MoveToPose_Response * msg);

/// Finalize srv/MoveToPose message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
void
piper_msgs__srv__MoveToPose_Response__fini(piper_msgs__srv__MoveToPose_Response * msg);

/// Create srv/MoveToPose message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * piper_msgs__srv__MoveToPose_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
piper_msgs__srv__MoveToPose_Response *
piper_msgs__srv__MoveToPose_Response__create();

/// Destroy srv/MoveToPose message.
/**
 * It calls
 * piper_msgs__srv__MoveToPose_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
void
piper_msgs__srv__MoveToPose_Response__destroy(piper_msgs__srv__MoveToPose_Response * msg);

/// Check for srv/MoveToPose message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Response__are_equal(const piper_msgs__srv__MoveToPose_Response * lhs, const piper_msgs__srv__MoveToPose_Response * rhs);

/// Copy a srv/MoveToPose message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Response__copy(
  const piper_msgs__srv__MoveToPose_Response * input,
  piper_msgs__srv__MoveToPose_Response * output);

/// Initialize array of srv/MoveToPose messages.
/**
 * It allocates the memory for the number of elements and calls
 * piper_msgs__srv__MoveToPose_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Response__Sequence__init(piper_msgs__srv__MoveToPose_Response__Sequence * array, size_t size);

/// Finalize array of srv/MoveToPose messages.
/**
 * It calls
 * piper_msgs__srv__MoveToPose_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
void
piper_msgs__srv__MoveToPose_Response__Sequence__fini(piper_msgs__srv__MoveToPose_Response__Sequence * array);

/// Create array of srv/MoveToPose messages.
/**
 * It allocates the memory for the array and calls
 * piper_msgs__srv__MoveToPose_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
piper_msgs__srv__MoveToPose_Response__Sequence *
piper_msgs__srv__MoveToPose_Response__Sequence__create(size_t size);

/// Destroy array of srv/MoveToPose messages.
/**
 * It calls
 * piper_msgs__srv__MoveToPose_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
void
piper_msgs__srv__MoveToPose_Response__Sequence__destroy(piper_msgs__srv__MoveToPose_Response__Sequence * array);

/// Check for srv/MoveToPose message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Response__Sequence__are_equal(const piper_msgs__srv__MoveToPose_Response__Sequence * lhs, const piper_msgs__srv__MoveToPose_Response__Sequence * rhs);

/// Copy an array of srv/MoveToPose messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_piper_msgs
bool
piper_msgs__srv__MoveToPose_Response__Sequence__copy(
  const piper_msgs__srv__MoveToPose_Response__Sequence * input,
  piper_msgs__srv__MoveToPose_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // PIPER_MSGS__SRV__DETAIL__MOVE_TO_POSE__FUNCTIONS_H_
