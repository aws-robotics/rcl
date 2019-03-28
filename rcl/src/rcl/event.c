// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifdef __cplusplus
extern "C"
{
#endif

#include "rcl/event.h"

#include <stdio.h>

#include "rcl/error_handling.h"
#include "rcl/expand_topic_name.h"
#include "rcl/remap.h"
#include "rcutils/logging_macros.h"
#include "rmw/error_handling.h"
#include "rmw/validate_full_topic_name.h"

#include "./common.h"
#include "./publisher_impl.h"
#include "./subscription_impl.h"


typedef struct rcl_event_impl_t
{
  rmw_event_t * rmw_handle;
} rcl_event_impl_t;


rcl_event_t
rcl_get_zero_initialized_event()
{
  static rcl_event_t null_event = {0};
  return null_event;
}

rcl_ret_t
rcl_publisher_event_init(
  rcl_event_t * event,
  const rcl_publisher_t * publisher,
  const rcl_publisher_event_type_t event_type,
  const rcl_allocator_t * allocator)
{
  rcl_ret_t ret = RCL_RET_OK;

  // Check allocator first, so allocator can be used with errors.
  RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

  // Allocate space for the implementation struct.
  event->impl = (rcl_event_impl_t *) allocator->allocate(
    sizeof(rcl_event_impl_t), allocator->state);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    event->impl, "allocating memory failed", ret = RCL_RET_BAD_ALLOC; return ret);

  rmw_event_type_t rmw_event_type = RMW_EVENT_INVALID;
  switch (event_type) {
    case RCL_PUBLISHER_OFFERED_DEADLINE_MISSED:
      rmw_event_type = RMW_EVENT_OFFERED_DEADLINE_MISSED;
      break;
    case RCL_PUBLISHER_LIVELINESS_LOST:
      rmw_event_type = RMW_EVENT_LIVELINESS_LOST;
      break;
    default:
      return RCL_RET_INVALID_ARGUMENT;
  }

  event->impl->rmw_handle = rmw_create_publisher_event(publisher->impl->rmw_handle, rmw_event_type);

  return ret;
}

rcl_ret_t
rcl_subscription_event_init(
  rcl_event_t * event,
  const rcl_subscription_t * subscription,
  const rcl_subscription_event_type_t event_type,
  const rcl_allocator_t * allocator)
{
  rcl_ret_t ret = RCL_RET_OK;

  // Check allocator first, so allocator can be used with errors.
  RCL_CHECK_ALLOCATOR_WITH_MSG(allocator, "invalid allocator", return RCL_RET_INVALID_ARGUMENT);

  // Allocate space for the implementation struct.
  event->impl = (rcl_event_impl_t *) allocator->allocate(
    sizeof(rcl_event_impl_t), allocator->state);
  RCL_CHECK_FOR_NULL_WITH_MSG(
    event->impl, "allocating memory failed", ret = RCL_RET_BAD_ALLOC; return ret);

  rmw_event_type_t rmw_event_type = RMW_EVENT_INVALID;
  switch (event_type) {
    case RCL_SUBSCRIPTION_REQUESTED_DEADLINE_MISSED:
      rmw_event_type = RMW_EVENT_REQUESTED_DEADLINE_MISSED;
      break;
    case RCL_SUBSCRIPTION_LIVELINESS_CHANGED:
      rmw_event_type = RMW_EVENT_LIVELINESS_CHANGED;
      break;
    default:
      return RCL_RET_INVALID_ARGUMENT;
  }

  event->impl->rmw_handle =
    rmw_create_subscription_event(subscription->impl->rmw_handle, rmw_event_type);

  return ret;
}

rcl_ret_t
rcl_take_event(
  const rcl_event_t * event,
  void * event_info)
{
  bool taken = false;
  RCL_CHECK_ARGUMENT_FOR_NULL(event, RCL_RET_INVALID_ARGUMENT);
  RCL_CHECK_ARGUMENT_FOR_NULL(event_info, RCL_RET_INVALID_ARGUMENT);
  rmw_ret_t ret = rmw_take_event(event->impl->rmw_handle, event_info, &taken);
  if (RMW_RET_OK != ret) {
    RCL_SET_ERROR_MSG(rmw_get_error_string().str);
    return rcl_convert_rmw_ret_to_rcl_ret(ret);
  }
  RCUTILS_LOG_DEBUG_NAMED(
    ROS_PACKAGE_NAME, "Event take request succeeded: %s", taken ? "true" : "false");
  if (!taken) {
    return RCL_RET_EVENT_TAKE_FAILED;
  }
  return rcl_convert_rmw_ret_to_rcl_ret(ret);
}

rcl_ret_t
rcl_event_fini(rcl_event_t * event)
{
  rmw_ret_t ret = rmw_destroy_event(event->impl->rmw_handle);
  event->impl = NULL;
  return rcl_convert_rmw_ret_to_rcl_ret(ret);
}

rmw_event_t *
rcl_event_get_rmw_handle(const rcl_event_t * event)
{
  if (NULL == event) {
    return NULL;  // error already set
  } else {
    return event->impl->rmw_handle;
  }
}

#ifdef __cplusplus
}
#endif
