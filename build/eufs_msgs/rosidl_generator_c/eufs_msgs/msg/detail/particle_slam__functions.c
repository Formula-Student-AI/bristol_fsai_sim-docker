// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from eufs_msgs:msg/ParticleSLAM.idl
// generated code does not contain a copyright notice
#include "eufs_msgs/msg/detail/particle_slam__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>


// Include directives for member types
// Member `particles`
#include "eufs_msgs/msg/detail/particle__functions.h"

bool
eufs_msgs__msg__ParticleSLAM__init(eufs_msgs__msg__ParticleSLAM * msg)
{
  if (!msg) {
    return false;
  }
  // best_particle_idx
  // particles
  if (!eufs_msgs__msg__Particle__Sequence__init(&msg->particles, 0)) {
    eufs_msgs__msg__ParticleSLAM__fini(msg);
    return false;
  }
  return true;
}

void
eufs_msgs__msg__ParticleSLAM__fini(eufs_msgs__msg__ParticleSLAM * msg)
{
  if (!msg) {
    return;
  }
  // best_particle_idx
  // particles
  eufs_msgs__msg__Particle__Sequence__fini(&msg->particles);
}

bool
eufs_msgs__msg__ParticleSLAM__are_equal(const eufs_msgs__msg__ParticleSLAM * lhs, const eufs_msgs__msg__ParticleSLAM * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // best_particle_idx
  if (lhs->best_particle_idx != rhs->best_particle_idx) {
    return false;
  }
  // particles
  if (!eufs_msgs__msg__Particle__Sequence__are_equal(
      &(lhs->particles), &(rhs->particles)))
  {
    return false;
  }
  return true;
}

bool
eufs_msgs__msg__ParticleSLAM__copy(
  const eufs_msgs__msg__ParticleSLAM * input,
  eufs_msgs__msg__ParticleSLAM * output)
{
  if (!input || !output) {
    return false;
  }
  // best_particle_idx
  output->best_particle_idx = input->best_particle_idx;
  // particles
  if (!eufs_msgs__msg__Particle__Sequence__copy(
      &(input->particles), &(output->particles)))
  {
    return false;
  }
  return true;
}

eufs_msgs__msg__ParticleSLAM *
eufs_msgs__msg__ParticleSLAM__create()
{
  eufs_msgs__msg__ParticleSLAM * msg = (eufs_msgs__msg__ParticleSLAM *)malloc(sizeof(eufs_msgs__msg__ParticleSLAM));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(eufs_msgs__msg__ParticleSLAM));
  bool success = eufs_msgs__msg__ParticleSLAM__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
eufs_msgs__msg__ParticleSLAM__destroy(eufs_msgs__msg__ParticleSLAM * msg)
{
  if (msg) {
    eufs_msgs__msg__ParticleSLAM__fini(msg);
  }
  free(msg);
}


bool
eufs_msgs__msg__ParticleSLAM__Sequence__init(eufs_msgs__msg__ParticleSLAM__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  eufs_msgs__msg__ParticleSLAM * data = NULL;
  if (size) {
    data = (eufs_msgs__msg__ParticleSLAM *)calloc(size, sizeof(eufs_msgs__msg__ParticleSLAM));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = eufs_msgs__msg__ParticleSLAM__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        eufs_msgs__msg__ParticleSLAM__fini(&data[i - 1]);
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
eufs_msgs__msg__ParticleSLAM__Sequence__fini(eufs_msgs__msg__ParticleSLAM__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      eufs_msgs__msg__ParticleSLAM__fini(&array->data[i]);
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

eufs_msgs__msg__ParticleSLAM__Sequence *
eufs_msgs__msg__ParticleSLAM__Sequence__create(size_t size)
{
  eufs_msgs__msg__ParticleSLAM__Sequence * array = (eufs_msgs__msg__ParticleSLAM__Sequence *)malloc(sizeof(eufs_msgs__msg__ParticleSLAM__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = eufs_msgs__msg__ParticleSLAM__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
eufs_msgs__msg__ParticleSLAM__Sequence__destroy(eufs_msgs__msg__ParticleSLAM__Sequence * array)
{
  if (array) {
    eufs_msgs__msg__ParticleSLAM__Sequence__fini(array);
  }
  free(array);
}

bool
eufs_msgs__msg__ParticleSLAM__Sequence__are_equal(const eufs_msgs__msg__ParticleSLAM__Sequence * lhs, const eufs_msgs__msg__ParticleSLAM__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!eufs_msgs__msg__ParticleSLAM__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
eufs_msgs__msg__ParticleSLAM__Sequence__copy(
  const eufs_msgs__msg__ParticleSLAM__Sequence * input,
  eufs_msgs__msg__ParticleSLAM__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(eufs_msgs__msg__ParticleSLAM);
    eufs_msgs__msg__ParticleSLAM * data =
      (eufs_msgs__msg__ParticleSLAM *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!eufs_msgs__msg__ParticleSLAM__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          eufs_msgs__msg__ParticleSLAM__fini(&data[i]);
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
    if (!eufs_msgs__msg__ParticleSLAM__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
