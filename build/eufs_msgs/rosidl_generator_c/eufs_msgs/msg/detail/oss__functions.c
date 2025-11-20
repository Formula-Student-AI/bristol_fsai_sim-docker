// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eufs_msgs:msg/OSS.idl
// generated code does not contain a copyright notice
#include "eufs_msgs/msg/detail/oss__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
eufs_msgs__msg__OSS__init(eufs_msgs__msg__OSS * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    eufs_msgs__msg__OSS__fini(msg);
    return false;
  }
  // velocity
  // v_x
  // v_y
  // yaw_rate
  // distance
  return true;
}

void
eufs_msgs__msg__OSS__fini(eufs_msgs__msg__OSS * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // velocity
  // v_x
  // v_y
  // yaw_rate
  // distance
}

bool
eufs_msgs__msg__OSS__are_equal(const eufs_msgs__msg__OSS * lhs, const eufs_msgs__msg__OSS * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // velocity
  if (lhs->velocity != rhs->velocity) {
    return false;
  }
  // v_x
  if (lhs->v_x != rhs->v_x) {
    return false;
  }
  // v_y
  if (lhs->v_y != rhs->v_y) {
    return false;
  }
  // yaw_rate
  if (lhs->yaw_rate != rhs->yaw_rate) {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  return true;
}

bool
eufs_msgs__msg__OSS__copy(
  const eufs_msgs__msg__OSS * input,
  eufs_msgs__msg__OSS * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // velocity
  output->velocity = input->velocity;
  // v_x
  output->v_x = input->v_x;
  // v_y
  output->v_y = input->v_y;
  // yaw_rate
  output->yaw_rate = input->yaw_rate;
  // distance
  output->distance = input->distance;
  return true;
}

eufs_msgs__msg__OSS *
eufs_msgs__msg__OSS__create()
{
  eufs_msgs__msg__OSS * msg = (eufs_msgs__msg__OSS *)malloc(sizeof(eufs_msgs__msg__OSS));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eufs_msgs__msg__OSS));
  bool success = eufs_msgs__msg__OSS__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
eufs_msgs__msg__OSS__destroy(eufs_msgs__msg__OSS * msg)
{
  if (msg) {
    eufs_msgs__msg__OSS__fini(msg);
  }
  free(msg);
}


bool
eufs_msgs__msg__OSS__Sequence__init(eufs_msgs__msg__OSS__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  eufs_msgs__msg__OSS * data = NULL;
  if (size) {
    data = (eufs_msgs__msg__OSS *)calloc(size, sizeof(eufs_msgs__msg__OSS));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eufs_msgs__msg__OSS__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eufs_msgs__msg__OSS__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
eufs_msgs__msg__OSS__Sequence__fini(eufs_msgs__msg__OSS__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      eufs_msgs__msg__OSS__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

eufs_msgs__msg__OSS__Sequence *
eufs_msgs__msg__OSS__Sequence__create(size_t size)
{
  eufs_msgs__msg__OSS__Sequence * array = (eufs_msgs__msg__OSS__Sequence *)malloc(sizeof(eufs_msgs__msg__OSS__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = eufs_msgs__msg__OSS__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
eufs_msgs__msg__OSS__Sequence__destroy(eufs_msgs__msg__OSS__Sequence * array)
{
  if (array) {
    eufs_msgs__msg__OSS__Sequence__fini(array);
  }
  free(array);
}

bool
eufs_msgs__msg__OSS__Sequence__are_equal(const eufs_msgs__msg__OSS__Sequence * lhs, const eufs_msgs__msg__OSS__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eufs_msgs__msg__OSS__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__msg__OSS__Sequence__copy(
  const eufs_msgs__msg__OSS__Sequence * input,
  eufs_msgs__msg__OSS__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eufs_msgs__msg__OSS);
    eufs_msgs__msg__OSS * data =
      (eufs_msgs__msg__OSS *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eufs_msgs__msg__OSS__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          eufs_msgs__msg__OSS__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!eufs_msgs__msg__OSS__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
