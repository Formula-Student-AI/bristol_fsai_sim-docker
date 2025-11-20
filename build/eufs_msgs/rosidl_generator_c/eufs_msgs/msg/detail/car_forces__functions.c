// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eufs_msgs:msg/CarForces.idl
// generated code does not contain a copyright notice
#include "eufs_msgs/msg/detail/car_forces__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


bool
eufs_msgs__msg__CarForces__init(eufs_msgs__msg__CarForces * msg)
{
  if (!msg) {
    return false;
  }
  // front_left_lateral
  // front_left_longitudinal
  // front_left_vertical
  // front_right_lateral
  // front_right_longitudinal
  // front_right_vertical
  // rear_left_lateral
  // rear_left_longitudinal
  // rear_left_vertical
  // rear_right_lateral
  // rear_right_longitudinal
  // rear_right_vertical
  return true;
}

void
eufs_msgs__msg__CarForces__fini(eufs_msgs__msg__CarForces * msg)
{
  if (!msg) {
    return;
  }
  // front_left_lateral
  // front_left_longitudinal
  // front_left_vertical
  // front_right_lateral
  // front_right_longitudinal
  // front_right_vertical
  // rear_left_lateral
  // rear_left_longitudinal
  // rear_left_vertical
  // rear_right_lateral
  // rear_right_longitudinal
  // rear_right_vertical
}

bool
eufs_msgs__msg__CarForces__are_equal(const eufs_msgs__msg__CarForces * lhs, const eufs_msgs__msg__CarForces * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // front_left_lateral
  if (lhs->front_left_lateral != rhs->front_left_lateral) {
    return false;
  }
  // front_left_longitudinal
  if (lhs->front_left_longitudinal != rhs->front_left_longitudinal) {
    return false;
  }
  // front_left_vertical
  if (lhs->front_left_vertical != rhs->front_left_vertical) {
    return false;
  }
  // front_right_lateral
  if (lhs->front_right_lateral != rhs->front_right_lateral) {
    return false;
  }
  // front_right_longitudinal
  if (lhs->front_right_longitudinal != rhs->front_right_longitudinal) {
    return false;
  }
  // front_right_vertical
  if (lhs->front_right_vertical != rhs->front_right_vertical) {
    return false;
  }
  // rear_left_lateral
  if (lhs->rear_left_lateral != rhs->rear_left_lateral) {
    return false;
  }
  // rear_left_longitudinal
  if (lhs->rear_left_longitudinal != rhs->rear_left_longitudinal) {
    return false;
  }
  // rear_left_vertical
  if (lhs->rear_left_vertical != rhs->rear_left_vertical) {
    return false;
  }
  // rear_right_lateral
  if (lhs->rear_right_lateral != rhs->rear_right_lateral) {
    return false;
  }
  // rear_right_longitudinal
  if (lhs->rear_right_longitudinal != rhs->rear_right_longitudinal) {
    return false;
  }
  // rear_right_vertical
  if (lhs->rear_right_vertical != rhs->rear_right_vertical) {
    return false;
  }
  return true;
}

bool
eufs_msgs__msg__CarForces__copy(
  const eufs_msgs__msg__CarForces * input,
  eufs_msgs__msg__CarForces * output)
{
  if (!input || !output) {
    return false;
  }
  // front_left_lateral
  output->front_left_lateral = input->front_left_lateral;
  // front_left_longitudinal
  output->front_left_longitudinal = input->front_left_longitudinal;
  // front_left_vertical
  output->front_left_vertical = input->front_left_vertical;
  // front_right_lateral
  output->front_right_lateral = input->front_right_lateral;
  // front_right_longitudinal
  output->front_right_longitudinal = input->front_right_longitudinal;
  // front_right_vertical
  output->front_right_vertical = input->front_right_vertical;
  // rear_left_lateral
  output->rear_left_lateral = input->rear_left_lateral;
  // rear_left_longitudinal
  output->rear_left_longitudinal = input->rear_left_longitudinal;
  // rear_left_vertical
  output->rear_left_vertical = input->rear_left_vertical;
  // rear_right_lateral
  output->rear_right_lateral = input->rear_right_lateral;
  // rear_right_longitudinal
  output->rear_right_longitudinal = input->rear_right_longitudinal;
  // rear_right_vertical
  output->rear_right_vertical = input->rear_right_vertical;
  return true;
}

eufs_msgs__msg__CarForces *
eufs_msgs__msg__CarForces__create()
{
  eufs_msgs__msg__CarForces * msg = (eufs_msgs__msg__CarForces *)malloc(sizeof(eufs_msgs__msg__CarForces));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eufs_msgs__msg__CarForces));
  bool success = eufs_msgs__msg__CarForces__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
eufs_msgs__msg__CarForces__destroy(eufs_msgs__msg__CarForces * msg)
{
  if (msg) {
    eufs_msgs__msg__CarForces__fini(msg);
  }
  free(msg);
}


bool
eufs_msgs__msg__CarForces__Sequence__init(eufs_msgs__msg__CarForces__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  eufs_msgs__msg__CarForces * data = NULL;
  if (size) {
    data = (eufs_msgs__msg__CarForces *)calloc(size, sizeof(eufs_msgs__msg__CarForces));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eufs_msgs__msg__CarForces__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eufs_msgs__msg__CarForces__fini(&data[i - 1]);
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
eufs_msgs__msg__CarForces__Sequence__fini(eufs_msgs__msg__CarForces__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      eufs_msgs__msg__CarForces__fini(&array->data[i]);
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

eufs_msgs__msg__CarForces__Sequence *
eufs_msgs__msg__CarForces__Sequence__create(size_t size)
{
  eufs_msgs__msg__CarForces__Sequence * array = (eufs_msgs__msg__CarForces__Sequence *)malloc(sizeof(eufs_msgs__msg__CarForces__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = eufs_msgs__msg__CarForces__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
eufs_msgs__msg__CarForces__Sequence__destroy(eufs_msgs__msg__CarForces__Sequence * array)
{
  if (array) {
    eufs_msgs__msg__CarForces__Sequence__fini(array);
  }
  free(array);
}

bool
eufs_msgs__msg__CarForces__Sequence__are_equal(const eufs_msgs__msg__CarForces__Sequence * lhs, const eufs_msgs__msg__CarForces__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eufs_msgs__msg__CarForces__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__msg__CarForces__Sequence__copy(
  const eufs_msgs__msg__CarForces__Sequence * input,
  eufs_msgs__msg__CarForces__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eufs_msgs__msg__CarForces);
    eufs_msgs__msg__CarForces * data =
      (eufs_msgs__msg__CarForces *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eufs_msgs__msg__CarForces__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          eufs_msgs__msg__CarForces__fini(&data[i]);
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
    if (!eufs_msgs__msg__CarForces__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
