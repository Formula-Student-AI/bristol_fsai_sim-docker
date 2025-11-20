// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eufs_msgs:srv/GetMap.idl
// generated code does not contain a copyright notice
#include "eufs_msgs/srv/detail/get_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
eufs_msgs__srv__GetMap_Request__init(eufs_msgs__srv__GetMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
eufs_msgs__srv__GetMap_Request__fini(eufs_msgs__srv__GetMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
eufs_msgs__srv__GetMap_Request__are_equal(const eufs_msgs__srv__GetMap_Request * lhs, const eufs_msgs__srv__GetMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
eufs_msgs__srv__GetMap_Request__copy(
  const eufs_msgs__srv__GetMap_Request * input,
  eufs_msgs__srv__GetMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

eufs_msgs__srv__GetMap_Request *
eufs_msgs__srv__GetMap_Request__create()
{
  eufs_msgs__srv__GetMap_Request * msg = (eufs_msgs__srv__GetMap_Request *)malloc(sizeof(eufs_msgs__srv__GetMap_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eufs_msgs__srv__GetMap_Request));
  bool success = eufs_msgs__srv__GetMap_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
eufs_msgs__srv__GetMap_Request__destroy(eufs_msgs__srv__GetMap_Request * msg)
{
  if (msg) {
    eufs_msgs__srv__GetMap_Request__fini(msg);
  }
  free(msg);
}


bool
eufs_msgs__srv__GetMap_Request__Sequence__init(eufs_msgs__srv__GetMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  eufs_msgs__srv__GetMap_Request * data = NULL;
  if (size) {
    data = (eufs_msgs__srv__GetMap_Request *)calloc(size, sizeof(eufs_msgs__srv__GetMap_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eufs_msgs__srv__GetMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eufs_msgs__srv__GetMap_Request__fini(&data[i - 1]);
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
eufs_msgs__srv__GetMap_Request__Sequence__fini(eufs_msgs__srv__GetMap_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      eufs_msgs__srv__GetMap_Request__fini(&array->data[i]);
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

eufs_msgs__srv__GetMap_Request__Sequence *
eufs_msgs__srv__GetMap_Request__Sequence__create(size_t size)
{
  eufs_msgs__srv__GetMap_Request__Sequence * array = (eufs_msgs__srv__GetMap_Request__Sequence *)malloc(sizeof(eufs_msgs__srv__GetMap_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = eufs_msgs__srv__GetMap_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
eufs_msgs__srv__GetMap_Request__Sequence__destroy(eufs_msgs__srv__GetMap_Request__Sequence * array)
{
  if (array) {
    eufs_msgs__srv__GetMap_Request__Sequence__fini(array);
  }
  free(array);
}

bool
eufs_msgs__srv__GetMap_Request__Sequence__are_equal(const eufs_msgs__srv__GetMap_Request__Sequence * lhs, const eufs_msgs__srv__GetMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eufs_msgs__srv__GetMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__srv__GetMap_Request__Sequence__copy(
  const eufs_msgs__srv__GetMap_Request__Sequence * input,
  eufs_msgs__srv__GetMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eufs_msgs__srv__GetMap_Request);
    eufs_msgs__srv__GetMap_Request * data =
      (eufs_msgs__srv__GetMap_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eufs_msgs__srv__GetMap_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          eufs_msgs__srv__GetMap_Request__fini(&data[i]);
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
    if (!eufs_msgs__srv__GetMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `map_path`
#include "rosidl_runtime_c/string_functions.h"

bool
eufs_msgs__srv__GetMap_Response__init(eufs_msgs__srv__GetMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // map_path
  if (!rosidl_runtime_c__String__Sequence__init(&msg->map_path, 0)) {
    eufs_msgs__srv__GetMap_Response__fini(msg);
    return false;
  }
  return true;
}

void
eufs_msgs__srv__GetMap_Response__fini(eufs_msgs__srv__GetMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // map_path
  rosidl_runtime_c__String__Sequence__fini(&msg->map_path);
}

bool
eufs_msgs__srv__GetMap_Response__are_equal(const eufs_msgs__srv__GetMap_Response * lhs, const eufs_msgs__srv__GetMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // map_path
  if (!rosidl_runtime_c__String__Sequence__are_equal(
      &(lhs->map_path), &(rhs->map_path)))
  {
    return false;
  }
  return true;
}

bool
eufs_msgs__srv__GetMap_Response__copy(
  const eufs_msgs__srv__GetMap_Response * input,
  eufs_msgs__srv__GetMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // map_path
  if (!rosidl_runtime_c__String__Sequence__copy(
      &(input->map_path), &(output->map_path)))
  {
    return false;
  }
  return true;
}

eufs_msgs__srv__GetMap_Response *
eufs_msgs__srv__GetMap_Response__create()
{
  eufs_msgs__srv__GetMap_Response * msg = (eufs_msgs__srv__GetMap_Response *)malloc(sizeof(eufs_msgs__srv__GetMap_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eufs_msgs__srv__GetMap_Response));
  bool success = eufs_msgs__srv__GetMap_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
eufs_msgs__srv__GetMap_Response__destroy(eufs_msgs__srv__GetMap_Response * msg)
{
  if (msg) {
    eufs_msgs__srv__GetMap_Response__fini(msg);
  }
  free(msg);
}


bool
eufs_msgs__srv__GetMap_Response__Sequence__init(eufs_msgs__srv__GetMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  eufs_msgs__srv__GetMap_Response * data = NULL;
  if (size) {
    data = (eufs_msgs__srv__GetMap_Response *)calloc(size, sizeof(eufs_msgs__srv__GetMap_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eufs_msgs__srv__GetMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eufs_msgs__srv__GetMap_Response__fini(&data[i - 1]);
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
eufs_msgs__srv__GetMap_Response__Sequence__fini(eufs_msgs__srv__GetMap_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      eufs_msgs__srv__GetMap_Response__fini(&array->data[i]);
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

eufs_msgs__srv__GetMap_Response__Sequence *
eufs_msgs__srv__GetMap_Response__Sequence__create(size_t size)
{
  eufs_msgs__srv__GetMap_Response__Sequence * array = (eufs_msgs__srv__GetMap_Response__Sequence *)malloc(sizeof(eufs_msgs__srv__GetMap_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = eufs_msgs__srv__GetMap_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
eufs_msgs__srv__GetMap_Response__Sequence__destroy(eufs_msgs__srv__GetMap_Response__Sequence * array)
{
  if (array) {
    eufs_msgs__srv__GetMap_Response__Sequence__fini(array);
  }
  free(array);
}

bool
eufs_msgs__srv__GetMap_Response__Sequence__are_equal(const eufs_msgs__srv__GetMap_Response__Sequence * lhs, const eufs_msgs__srv__GetMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eufs_msgs__srv__GetMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__srv__GetMap_Response__Sequence__copy(
  const eufs_msgs__srv__GetMap_Response__Sequence * input,
  eufs_msgs__srv__GetMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eufs_msgs__srv__GetMap_Response);
    eufs_msgs__srv__GetMap_Response * data =
      (eufs_msgs__srv__GetMap_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eufs_msgs__srv__GetMap_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          eufs_msgs__srv__GetMap_Response__fini(&data[i]);
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
    if (!eufs_msgs__srv__GetMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
