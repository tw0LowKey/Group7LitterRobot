// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from piper_msgs:srv/PickPlaceRequest.idl
// generated code does not contain a copyright notice

#ifndef PIPER_MSGS__SRV__DETAIL__PICK_PLACE_REQUEST__STRUCT_HPP_
#define PIPER_MSGS__SRV__DETAIL__PICK_PLACE_REQUEST__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__piper_msgs__srv__PickPlaceRequest_Request __attribute__((deprecated))
#else
# define DEPRECATED__piper_msgs__srv__PickPlaceRequest_Request __declspec(deprecated)
#endif

namespace piper_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PickPlaceRequest_Request_
{
  using Type = PickPlaceRequest_Request_<ContainerAllocator>;

  explicit PickPlaceRequest_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    (void)_init;
  }

  explicit PickPlaceRequest_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__piper_msgs__srv__PickPlaceRequest_Request
    std::shared_ptr<piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__piper_msgs__srv__PickPlaceRequest_Request
    std::shared_ptr<piper_msgs::srv::PickPlaceRequest_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPlaceRequest_Request_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPlaceRequest_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPlaceRequest_Request_

// alias to use template instance with default allocator
using PickPlaceRequest_Request =
  piper_msgs::srv::PickPlaceRequest_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace piper_msgs


#ifndef _WIN32
# define DEPRECATED__piper_msgs__srv__PickPlaceRequest_Response __attribute__((deprecated))
#else
# define DEPRECATED__piper_msgs__srv__PickPlaceRequest_Response __declspec(deprecated)
#endif

namespace piper_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PickPlaceRequest_Response_
{
  using Type = PickPlaceRequest_Response_<ContainerAllocator>;

  explicit PickPlaceRequest_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit PickPlaceRequest_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__piper_msgs__srv__PickPlaceRequest_Response
    std::shared_ptr<piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__piper_msgs__srv__PickPlaceRequest_Response
    std::shared_ptr<piper_msgs::srv::PickPlaceRequest_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPlaceRequest_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPlaceRequest_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPlaceRequest_Response_

// alias to use template instance with default allocator
using PickPlaceRequest_Response =
  piper_msgs::srv::PickPlaceRequest_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace piper_msgs

namespace piper_msgs
{

namespace srv
{

struct PickPlaceRequest
{
  using Request = piper_msgs::srv::PickPlaceRequest_Request;
  using Response = piper_msgs::srv::PickPlaceRequest_Response;
};

}  // namespace srv

}  // namespace piper_msgs

#endif  // PIPER_MSGS__SRV__DETAIL__PICK_PLACE_REQUEST__STRUCT_HPP_
