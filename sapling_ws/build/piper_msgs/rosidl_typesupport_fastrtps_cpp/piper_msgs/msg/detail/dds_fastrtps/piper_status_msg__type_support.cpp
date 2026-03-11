// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from piper_msgs:msg/PiperStatusMsg.idl
// generated code does not contain a copyright notice
#include "piper_msgs/msg/detail/piper_status_msg__rosidl_typesupport_fastrtps_cpp.hpp"
#include "piper_msgs/msg/detail/piper_status_msg__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace piper_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_piper_msgs
cdr_serialize(
  const piper_msgs::msg::PiperStatusMsg & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: ctrl_mode
  cdr << ros_message.ctrl_mode;
  // Member: arm_status
  cdr << ros_message.arm_status;
  // Member: mode_feedback
  cdr << ros_message.mode_feedback;
  // Member: teach_status
  cdr << ros_message.teach_status;
  // Member: motion_status
  cdr << ros_message.motion_status;
  // Member: trajectory_num
  cdr << ros_message.trajectory_num;
  // Member: err_code
  cdr << ros_message.err_code;
  // Member: joint_1_angle_limit
  cdr << (ros_message.joint_1_angle_limit ? true : false);
  // Member: joint_2_angle_limit
  cdr << (ros_message.joint_2_angle_limit ? true : false);
  // Member: joint_3_angle_limit
  cdr << (ros_message.joint_3_angle_limit ? true : false);
  // Member: joint_4_angle_limit
  cdr << (ros_message.joint_4_angle_limit ? true : false);
  // Member: joint_5_angle_limit
  cdr << (ros_message.joint_5_angle_limit ? true : false);
  // Member: joint_6_angle_limit
  cdr << (ros_message.joint_6_angle_limit ? true : false);
  // Member: communication_status_joint_1
  cdr << (ros_message.communication_status_joint_1 ? true : false);
  // Member: communication_status_joint_2
  cdr << (ros_message.communication_status_joint_2 ? true : false);
  // Member: communication_status_joint_3
  cdr << (ros_message.communication_status_joint_3 ? true : false);
  // Member: communication_status_joint_4
  cdr << (ros_message.communication_status_joint_4 ? true : false);
  // Member: communication_status_joint_5
  cdr << (ros_message.communication_status_joint_5 ? true : false);
  // Member: communication_status_joint_6
  cdr << (ros_message.communication_status_joint_6 ? true : false);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_piper_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  piper_msgs::msg::PiperStatusMsg & ros_message)
{
  // Member: ctrl_mode
  cdr >> ros_message.ctrl_mode;

  // Member: arm_status
  cdr >> ros_message.arm_status;

  // Member: mode_feedback
  cdr >> ros_message.mode_feedback;

  // Member: teach_status
  cdr >> ros_message.teach_status;

  // Member: motion_status
  cdr >> ros_message.motion_status;

  // Member: trajectory_num
  cdr >> ros_message.trajectory_num;

  // Member: err_code
  cdr >> ros_message.err_code;

  // Member: joint_1_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.joint_1_angle_limit = tmp ? true : false;
  }

  // Member: joint_2_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.joint_2_angle_limit = tmp ? true : false;
  }

  // Member: joint_3_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.joint_3_angle_limit = tmp ? true : false;
  }

  // Member: joint_4_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.joint_4_angle_limit = tmp ? true : false;
  }

  // Member: joint_5_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.joint_5_angle_limit = tmp ? true : false;
  }

  // Member: joint_6_angle_limit
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.joint_6_angle_limit = tmp ? true : false;
  }

  // Member: communication_status_joint_1
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.communication_status_joint_1 = tmp ? true : false;
  }

  // Member: communication_status_joint_2
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.communication_status_joint_2 = tmp ? true : false;
  }

  // Member: communication_status_joint_3
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.communication_status_joint_3 = tmp ? true : false;
  }

  // Member: communication_status_joint_4
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.communication_status_joint_4 = tmp ? true : false;
  }

  // Member: communication_status_joint_5
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.communication_status_joint_5 = tmp ? true : false;
  }

  // Member: communication_status_joint_6
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.communication_status_joint_6 = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_piper_msgs
get_serialized_size(
  const piper_msgs::msg::PiperStatusMsg & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: ctrl_mode
  {
    size_t item_size = sizeof(ros_message.ctrl_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: arm_status
  {
    size_t item_size = sizeof(ros_message.arm_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mode_feedback
  {
    size_t item_size = sizeof(ros_message.mode_feedback);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: teach_status
  {
    size_t item_size = sizeof(ros_message.teach_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: motion_status
  {
    size_t item_size = sizeof(ros_message.motion_status);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: trajectory_num
  {
    size_t item_size = sizeof(ros_message.trajectory_num);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: err_code
  {
    size_t item_size = sizeof(ros_message.err_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joint_1_angle_limit
  {
    size_t item_size = sizeof(ros_message.joint_1_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joint_2_angle_limit
  {
    size_t item_size = sizeof(ros_message.joint_2_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joint_3_angle_limit
  {
    size_t item_size = sizeof(ros_message.joint_3_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joint_4_angle_limit
  {
    size_t item_size = sizeof(ros_message.joint_4_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joint_5_angle_limit
  {
    size_t item_size = sizeof(ros_message.joint_5_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: joint_6_angle_limit
  {
    size_t item_size = sizeof(ros_message.joint_6_angle_limit);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: communication_status_joint_1
  {
    size_t item_size = sizeof(ros_message.communication_status_joint_1);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: communication_status_joint_2
  {
    size_t item_size = sizeof(ros_message.communication_status_joint_2);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: communication_status_joint_3
  {
    size_t item_size = sizeof(ros_message.communication_status_joint_3);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: communication_status_joint_4
  {
    size_t item_size = sizeof(ros_message.communication_status_joint_4);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: communication_status_joint_5
  {
    size_t item_size = sizeof(ros_message.communication_status_joint_5);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: communication_status_joint_6
  {
    size_t item_size = sizeof(ros_message.communication_status_joint_6);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_piper_msgs
max_serialized_size_PiperStatusMsg(
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


  // Member: ctrl_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: arm_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: mode_feedback
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: teach_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: motion_status
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: trajectory_num
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: err_code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: joint_1_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: joint_2_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: joint_3_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: joint_4_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: joint_5_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: joint_6_angle_limit
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: communication_status_joint_1
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: communication_status_joint_2
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: communication_status_joint_3
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: communication_status_joint_4
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: communication_status_joint_5
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: communication_status_joint_6
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
    using DataType = piper_msgs::msg::PiperStatusMsg;
    is_plain =
      (
      offsetof(DataType, communication_status_joint_6) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PiperStatusMsg__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const piper_msgs::msg::PiperStatusMsg *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PiperStatusMsg__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<piper_msgs::msg::PiperStatusMsg *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PiperStatusMsg__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const piper_msgs::msg::PiperStatusMsg *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PiperStatusMsg__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PiperStatusMsg(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PiperStatusMsg__callbacks = {
  "piper_msgs::msg",
  "PiperStatusMsg",
  _PiperStatusMsg__cdr_serialize,
  _PiperStatusMsg__cdr_deserialize,
  _PiperStatusMsg__get_serialized_size,
  _PiperStatusMsg__max_serialized_size
};

static rosidl_message_type_support_t _PiperStatusMsg__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PiperStatusMsg__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace piper_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_piper_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<piper_msgs::msg::PiperStatusMsg>()
{
  return &piper_msgs::msg::typesupport_fastrtps_cpp::_PiperStatusMsg__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, piper_msgs, msg, PiperStatusMsg)() {
  return &piper_msgs::msg::typesupport_fastrtps_cpp::_PiperStatusMsg__handle;
}

#ifdef __cplusplus
}
#endif
