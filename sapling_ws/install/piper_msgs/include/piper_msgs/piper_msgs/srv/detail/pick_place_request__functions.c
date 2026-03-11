// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from piper_msgs:srv/PickPlaceRequest.idl
// generated code does not contain a copyright notice
#include "piper_msgs/srv/detail/pick_place_request__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
piper_msgs__srv__PickPlaceRequest_Request__init(piper_msgs__srv__PickPlaceRequest_Request * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    piper_msgs__srv__PickPlaceRequest_Request__fini(msg);
    return false;
  }
  return true;
}

void
piper_msgs__srv__PickPlaceRequest_Request__fini(piper_msgs__srv__PickPlaceRequest_Request * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
}

bool
piper_msgs__srv__PickPlaceRequest_Request__are_equal(const piper_msgs__srv__PickPlaceRequest_Request * lhs, const piper_msgs__srv__PickPlaceRequest_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
piper_msgs__srv__PickPlaceRequest_Request__copy(
  const piper_msgs__srv__PickPlaceRequest_Request * input,
  piper_msgs__srv__PickPlaceRequest_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

piper_msgs__srv__PickPlaceRequest_Request *
piper_msgs__srv__PickPlaceRequest_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  piper_msgs__srv__PickPlaceRequest_Request * msg = (piper_msgs__srv__PickPlaceRequest_Request *)allocator.allocate(sizeof(piper_msgs__srv__PickPlaceRequest_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(piper_msgs__srv__PickPlaceRequest_Request));
  bool success = piper_msgs__srv__PickPlaceRequest_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
piper_msgs__srv__PickPlaceRequest_Request__destroy(piper_msgs__srv__PickPlaceRequest_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    piper_msgs__srv__PickPlaceRequest_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
piper_msgs__srv__PickPlaceRequest_Request__Sequence__init(piper_msgs__srv__PickPlaceRequest_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  piper_msgs__srv__PickPlaceRequest_Request * data = NULL;

  if (size) {
    data = (piper_msgs__srv__PickPlaceRequest_Request *)allocator.zero_allocate(size, sizeof(piper_msgs__srv__PickPlaceRequest_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = piper_msgs__srv__PickPlaceRequest_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        piper_msgs__srv__PickPlaceRequest_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
piper_msgs__srv__PickPlaceRequest_Request__Sequence__fini(piper_msgs__srv__PickPlaceRequest_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      piper_msgs__srv__PickPlaceRequest_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

piper_msgs__srv__PickPlaceRequest_Request__Sequence *
piper_msgs__srv__PickPlaceRequest_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  piper_msgs__srv__PickPlaceRequest_Request__Sequence * array = (piper_msgs__srv__PickPlaceRequest_Request__Sequence *)allocator.allocate(sizeof(piper_msgs__srv__PickPlaceRequest_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = piper_msgs__srv__PickPlaceRequest_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
piper_msgs__srv__PickPlaceRequest_Request__Sequence__destroy(piper_msgs__srv__PickPlaceRequest_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    piper_msgs__srv__PickPlaceRequest_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
piper_msgs__srv__PickPlaceRequest_Request__Sequence__are_equal(const piper_msgs__srv__PickPlaceRequest_Request__Sequence * lhs, const piper_msgs__srv__PickPlaceRequest_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!piper_msgs__srv__PickPlaceRequest_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
piper_msgs__srv__PickPlaceRequest_Request__Sequence__copy(
  const piper_msgs__srv__PickPlaceRequest_Request__Sequence * input,
  piper_msgs__srv__PickPlaceRequest_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(piper_msgs__srv__PickPlaceRequest_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    piper_msgs__srv__PickPlaceRequest_Request * data =
      (piper_msgs__srv__PickPlaceRequest_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!piper_msgs__srv__PickPlaceRequest_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          piper_msgs__srv__PickPlaceRequest_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!piper_msgs__srv__PickPlaceRequest_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
piper_msgs__srv__PickPlaceRequest_Response__init(piper_msgs__srv__PickPlaceRequest_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
piper_msgs__srv__PickPlaceRequest_Response__fini(piper_msgs__srv__PickPlaceRequest_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
piper_msgs__srv__PickPlaceRequest_Response__are_equal(const piper_msgs__srv__PickPlaceRequest_Response * lhs, const piper_msgs__srv__PickPlaceRequest_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
piper_msgs__srv__PickPlaceRequest_Response__copy(
  const piper_msgs__srv__PickPlaceRequest_Response * input,
  piper_msgs__srv__PickPlaceRequest_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

piper_msgs__srv__PickPlaceRequest_Response *
piper_msgs__srv__PickPlaceRequest_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  piper_msgs__srv__PickPlaceRequest_Response * msg = (piper_msgs__srv__PickPlaceRequest_Response *)allocator.allocate(sizeof(piper_msgs__srv__PickPlaceRequest_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(piper_msgs__srv__PickPlaceRequest_Response));
  bool success = piper_msgs__srv__PickPlaceRequest_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
piper_msgs__srv__PickPlaceRequest_Response__destroy(piper_msgs__srv__PickPlaceRequest_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    piper_msgs__srv__PickPlaceRequest_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
piper_msgs__srv__PickPlaceRequest_Response__Sequence__init(piper_msgs__srv__PickPlaceRequest_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  piper_msgs__srv__PickPlaceRequest_Response * data = NULL;

  if (size) {
    data = (piper_msgs__srv__PickPlaceRequest_Response *)allocator.zero_allocate(size, sizeof(piper_msgs__srv__PickPlaceRequest_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = piper_msgs__srv__PickPlaceRequest_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        piper_msgs__srv__PickPlaceRequest_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
piper_msgs__srv__PickPlaceRequest_Response__Sequence__fini(piper_msgs__srv__PickPlaceRequest_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      piper_msgs__srv__PickPlaceRequest_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

piper_msgs__srv__PickPlaceRequest_Response__Sequence *
piper_msgs__srv__PickPlaceRequest_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  piper_msgs__srv__PickPlaceRequest_Response__Sequence * array = (piper_msgs__srv__PickPlaceRequest_Response__Sequence *)allocator.allocate(sizeof(piper_msgs__srv__PickPlaceRequest_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = piper_msgs__srv__PickPlaceRequest_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
piper_msgs__srv__PickPlaceRequest_Response__Sequence__destroy(piper_msgs__srv__PickPlaceRequest_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    piper_msgs__srv__PickPlaceRequest_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
piper_msgs__srv__PickPlaceRequest_Response__Sequence__are_equal(const piper_msgs__srv__PickPlaceRequest_Response__Sequence * lhs, const piper_msgs__srv__PickPlaceRequest_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!piper_msgs__srv__PickPlaceRequest_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
piper_msgs__srv__PickPlaceRequest_Response__Sequence__copy(
  const piper_msgs__srv__PickPlaceRequest_Response__Sequence * input,
  piper_msgs__srv__PickPlaceRequest_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(piper_msgs__srv__PickPlaceRequest_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    piper_msgs__srv__PickPlaceRequest_Response * data =
      (piper_msgs__srv__PickPlaceRequest_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!piper_msgs__srv__PickPlaceRequest_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          piper_msgs__srv__PickPlaceRequest_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!piper_msgs__srv__PickPlaceRequest_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
