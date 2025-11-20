// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eufs_msgs:msg/ConeAssociationArrayArrayStamped.idl
// generated code does not contain a copyright notice
#include "eufs_msgs/msg/detail/cone_association_array_array_stamped__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `associations`
#include "eufs_msgs/msg/detail/cone_association_array__functions.h"

bool
eufs_msgs__msg__ConeAssociationArrayArrayStamped__init(eufs_msgs__msg__ConeAssociationArrayArrayStamped * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    eufs_msgs__msg__ConeAssociationArrayArrayStamped__fini(msg);
    return false;
  }
  // associations
  if (!eufs_msgs__msg__ConeAssociationArray__Sequence__init(&msg->associations, 0)) {
    eufs_msgs__msg__ConeAssociationArrayArrayStamped__fini(msg);
    return false;
  }
  return true;
}

void
eufs_msgs__msg__ConeAssociationArrayArrayStamped__fini(eufs_msgs__msg__ConeAssociationArrayArrayStamped * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // associations
  eufs_msgs__msg__ConeAssociationArray__Sequence__fini(&msg->associations);
}

bool
eufs_msgs__msg__ConeAssociationArrayArrayStamped__are_equal(const eufs_msgs__msg__ConeAssociationArrayArrayStamped * lhs, const eufs_msgs__msg__ConeAssociationArrayArrayStamped * rhs)
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
  // associations
  if (!eufs_msgs__msg__ConeAssociationArray__Sequence__are_equal(
      &(lhs->associations), &(rhs->associations)))
  {
    return false;
  }
  return true;
}

bool
eufs_msgs__msg__ConeAssociationArrayArrayStamped__copy(
  const eufs_msgs__msg__ConeAssociationArrayArrayStamped * input,
  eufs_msgs__msg__ConeAssociationArrayArrayStamped * output)
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
  // associations
  if (!eufs_msgs__msg__ConeAssociationArray__Sequence__copy(
      &(input->associations), &(output->associations)))
  {
    return false;
  }
  return true;
}

eufs_msgs__msg__ConeAssociationArrayArrayStamped *
eufs_msgs__msg__ConeAssociationArrayArrayStamped__create()
{
  eufs_msgs__msg__ConeAssociationArrayArrayStamped * msg = (eufs_msgs__msg__ConeAssociationArrayArrayStamped *)malloc(sizeof(eufs_msgs__msg__ConeAssociationArrayArrayStamped));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eufs_msgs__msg__ConeAssociationArrayArrayStamped));
  bool success = eufs_msgs__msg__ConeAssociationArrayArrayStamped__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
eufs_msgs__msg__ConeAssociationArrayArrayStamped__destroy(eufs_msgs__msg__ConeAssociationArrayArrayStamped * msg)
{
  if (msg) {
    eufs_msgs__msg__ConeAssociationArrayArrayStamped__fini(msg);
  }
  free(msg);
}


bool
eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence__init(eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  eufs_msgs__msg__ConeAssociationArrayArrayStamped * data = NULL;
  if (size) {
    data = (eufs_msgs__msg__ConeAssociationArrayArrayStamped *)calloc(size, sizeof(eufs_msgs__msg__ConeAssociationArrayArrayStamped));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eufs_msgs__msg__ConeAssociationArrayArrayStamped__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eufs_msgs__msg__ConeAssociationArrayArrayStamped__fini(&data[i - 1]);
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
eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence__fini(eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      eufs_msgs__msg__ConeAssociationArrayArrayStamped__fini(&array->data[i]);
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

eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence *
eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence__create(size_t size)
{
  eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence * array = (eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence *)malloc(sizeof(eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence__destroy(eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence * array)
{
  if (array) {
    eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence__fini(array);
  }
  free(array);
}

bool
eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence__are_equal(const eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence * lhs, const eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eufs_msgs__msg__ConeAssociationArrayArrayStamped__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence__copy(
  const eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence * input,
  eufs_msgs__msg__ConeAssociationArrayArrayStamped__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eufs_msgs__msg__ConeAssociationArrayArrayStamped);
    eufs_msgs__msg__ConeAssociationArrayArrayStamped * data =
      (eufs_msgs__msg__ConeAssociationArrayArrayStamped *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eufs_msgs__msg__ConeAssociationArrayArrayStamped__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          eufs_msgs__msg__ConeAssociationArrayArrayStamped__fini(&data[i]);
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
    if (!eufs_msgs__msg__ConeAssociationArrayArrayStamped__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
