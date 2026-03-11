// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from piper_msgs:msg/PiperStatusMsg.idl
// generated code does not contain a copyright notice
#include "piper_msgs/msg/detail/piper_status_msg__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "piper_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "piper_msgs/msg/detail/piper_status_msg__struct.h"
#include "piper_msgs/msg/detail/piper_status_msg__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _PiperStatusMsg__ros_msg_type = piper_msgs__msg__PiperStatusMsg;

static bool _PiperStatusMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _PiperStatusMsg__ros_msg_type * ros_message = static_cast<const _PiperStatusMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: ctrl_mode
  {
    cdr << ros_message->ctrl_mode;
  }

  // Field name: arm_status
  {
    cdr << ros_message->arm_status;
  }

  // Field name: mode_feedback
  {
    cdr << ros_message->mode_feedback;
  }

  // Field name: teach_status
  {
    cdr << ros_message->teach_status;
  }

  // Field name: motion_status
  {
    cdr << ros_message->motion_status;
  }

  // Field name: trajectory_num
  {
    cdr << ros_message->trajectory_num;
  }

  // Field name: err_code
  {
    cdr << ros_message->err_code;
  }

  // Field name: joint_1_angle_limit
  {
    cdr << (ros_message->joint_1_angle_limit ? true : false);
  }

  // Field name: joint_2_angle_limit
  {
    cdr << (ros_message->joint_2_angle_limit ? true : false);
  }

  // Field name: joint_3_angle_limit
  {
    cdr << (ros_message->joint_3_angle_limit ? true : false);
  }

  // Field name: joint_4_angle_limit
  {
    cdr << (ros_message->joint_4_angle_limit ? true : false);
  }

  // Field name: joint_5_angle_limit
  {
    cdr << (ros_message->joint_5_angle_limit ? true : false);
  }

  // Field name: joint_6_angle_limit
  {
    cdr << (ros_message->joint_6_angle_limit ? true : false);
  }

  // Field name: communication_status_joint_1
  {
    cdr << (ros_message->communication_status_joint_1 ? true : false);
  }

  // Field name: communication_status_joint_2
  {
    cdr << (ros_message->communication_status_joint_2 ? true : false);
  }

  // Field name: communication_status_joint_3
  {
    cdr << (ros_message->communication_status_joint_3 ? true : false);
  }

  // Field name: communication_status_joint_4
  {
    cdr << (ros_message->communication_status_joint_4 ? true : false);
  }

  // Field name: communication_status_joint_5
  {
    cdr << (ros_message->communication_status_joint_5 ? true : false);
  }

  // Field name: communication_status_joint_6
  {
    cdr << (ros_message->communication_status_joint_6 ? true : false);
  }

  return true;
}

static bool _PiperStatusMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _PiperStatusMsg__ros_msg_type * ros_message = static_cast<_PiperStatusMsg__ros_msg_type *>(untyped_ros_message);
  // Field name: ctrl_mode
  {
    cdr >> ros_message->ctrl_mode;
  }

  // Field name: arm_status
  {
    cdr >> ros_message->arm_status;
  }

  // Field name: mode_feedback
  {
    cdr >> ros_message->mode_feedback;
  }

  // Field name: teach_status
  {
    cdr >> ros_message->teach_status;
  }

  // Field name: motion_status
  {
    cdr >> ros_message->motion_status;
  }

  // Field name: trajectory_num
  {
    cdr >> ros_message->trajectory_num;
  }

  // Field name: err_code
  {
    cdr >> ros_message->err_code;
  }

  // Field name: joint_1_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->joint_1_angle_limit = tmp ? true : false;
  }

  // Field name: joint_2_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->joint_2_angle_limit = tmp ? true : false;
  }

  // Field name: joint_3_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->joint_3_angle_limit = tmp ? true : false;
  }

  // Field name: joint_4_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->joint_4_angle_limit = tmp ? true : false;
  }

  // Field name: joint_5_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->joint_5_angle_limit = tmp ? true : false;
  }

  // Field name: joint_6_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->joint_6_angle_limit = tmp ? true : false;
  }

  // Field name: communication_status_joint_1
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->communication_status_joint_1 = tmp ? true : false;
  }

  // Field name: communication_status_joint_2
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->communication_status_joint_2 = tmp ? true : false;
  }

  // Field name: communication_status_joint_3
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->communication_status_joint_3 = tmp ? true : false;
  }

  // Field name: communication_status_joint_4
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->communication_status_joint_4 = tmp ? true : false;
  }

  // Field name: communication_status_joint_5
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->communication_status_joint_5 = tmp ? true : false;
  }

  // Field name: communication_status_joint_6
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->communication_status_joint_6 = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_piper_msgs
size_t get_serialized_size_piper_msgs__msg__PiperStatusMsg(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _PiperStatusMsg__ros_msg_type * ros_message = static_cast<const _PiperStatusMsg__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name ctrl_mode
  {
    size_t item_size = sizeof(ros_message->ctrl_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name arm_status
  {
    size_t item_size = sizeof(ros_message->arm_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name mode_feedback
  {
    size_t item_size = sizeof(ros_message->mode_feedback);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name teach_status
  {
    size_t item_size = sizeof(ros_message->teach_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motion_status
  {
    size_t item_size = sizeof(ros_message->motion_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name trajectory_num
  {
    size_t item_size = sizeof(ros_message->trajectory_num);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name err_code
  {
    size_t item_size = sizeof(ros_message->err_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_1_angle_limit
  {
    size_t item_size = sizeof(ros_message->joint_1_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_2_angle_limit
  {
    size_t item_size = sizeof(ros_message->joint_2_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_3_angle_limit
  {
    size_t item_size = sizeof(ros_message->joint_3_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_4_angle_limit
  {
    size_t item_size = sizeof(ros_message->joint_4_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_5_angle_limit
  {
    size_t item_size = sizeof(ros_message->joint_5_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name joint_6_angle_limit
  {
    size_t item_size = sizeof(ros_message->joint_6_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name communication_status_joint_1
  {
    size_t item_size = sizeof(ros_message->communication_status_joint_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name communication_status_joint_2
  {
    size_t item_size = sizeof(ros_message->communication_status_joint_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name communication_status_joint_3
  {
    size_t item_size = sizeof(ros_message->communication_status_joint_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name communication_status_joint_4
  {
    size_t item_size = sizeof(ros_message->communication_status_joint_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name communication_status_joint_5
  {
    size_t item_size = sizeof(ros_message->communication_status_joint_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name communication_status_joint_6
  {
    size_t item_size = sizeof(ros_message->communication_status_joint_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _PiperStatusMsg__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_piper_msgs__msg__PiperStatusMsg(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_piper_msgs
size_t max_serialized_size_piper_msgs__msg__PiperStatusMsg(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: ctrl_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: arm_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: mode_feedback
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: teach_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: motion_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: trajectory_num
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: err_code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: joint_1_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: joint_2_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: joint_3_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: joint_4_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: joint_5_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: joint_6_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: communication_status_joint_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: communication_status_joint_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: communication_status_joint_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: communication_status_joint_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: communication_status_joint_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: communication_status_joint_6
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = piper_msgs__msg__PiperStatusMsg;
    is_plain =
      (
      offsetof(DataType, communication_status_joint_6) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _PiperStatusMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_piper_msgs__msg__PiperStatusMsg(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_PiperStatusMsg = {
  "piper_msgs::msg",
  "PiperStatusMsg",
  _PiperStatusMsg__cdr_serialize,
  _PiperStatusMsg__cdr_deserialize,
  _PiperStatusMsg__get_serialized_size,
  _PiperStatusMsg__max_serialized_size
};

static rosidl_message_type_support_t _PiperStatusMsg__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_PiperStatusMsg,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, piper_msgs, msg, PiperStatusMsg)() {
  return &_PiperStatusMsg__type_support;
}

#if defined(__cplusplus)
}
#endif
