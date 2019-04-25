// Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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

#include <gtest/gtest.h>

#include <string>
#include "rcl/security_directory.h"
#include "rcutils/filesystem.h"
#include "rcl/error_handling.h"

#define ROOT_NAMESPACE "/"
#define TEST_SECURITY_DIRECTORY_RESOURCES_DIR_NAME "test_security_directory"
#define TEST_NODE_NAME "dummy_node"
#define TEST_NODE_NAMESPACE ROOT_NAMESPACE TEST_SECURITY_DIRECTORY_RESOURCES_DIR_NAME

static int putenv_wrapper(const char * env_var)
{
#ifdef _WIN32
  return _putenv(env_var);
#else
  return putenv(reinterpret_cast<char *>(const_cast<char *>(env_var)));
#endif
}

static int unsetenv_wrapper(const char * var_name)
{
#ifdef _WIN32
  // On windows, putenv("VAR=") deletes VAR from environment
  std::string var(var_name);
  var += "=";
  return _putenv(var.c_str());
#else
  return unsetenv(var_name);
#endif
}

class TestGetSecureRoot : public ::testing::Test
{
protected:
  rcl_allocator_t allocator;
  char * root_path;

  void SetUp() override
  {
    // Reset rcutil error global state in case a previously
    // running test has failed.
    rcl_reset_error();

    // Always make sure the variable we set is unset at the beginning of a test
    unsetenv_wrapper(ROS_SECURITY_ROOT_DIRECTORY_VAR_NAME);
    unsetenv_wrapper(ROS_SECURITY_NODE_DIRECTORY_VAR_NAME);
    unsetenv_wrapper(ROS_SECURITY_LOOKUP_TYPE_VAR_NAME);
    allocator = rcl_get_default_allocator();
    root_path = "";
  }
};

TEST_F(TestGetSecureRoot, failureScenarios) {
  ASSERT_EQ(rcl_get_secure_root(TEST_NODE_NAME, TEST_NODE_NAMESPACE, allocator),
    (char *) NULL);

  putenv_wrapper(ROS_SECURITY_ROOT_DIRECTORY_VAR_NAME "=" TEST_RESOURCES_DIRECTORY);

  /* Security directory is set, but there's no matching directory */
  /// Wrong namespace
  ASSERT_EQ(rcl_get_secure_root(TEST_NODE_NAME, "/some_other_namespace", allocator),
    (char *) NULL);
  /// Wrong node name
  ASSERT_EQ(rcl_get_secure_root("not_" TEST_NODE_NAME, TEST_NODE_NAMESPACE, allocator),
    (char *) NULL);
}

TEST_F(TestGetSecureRoot, successScenarios) {
  putenv_wrapper(ROS_SECURITY_ROOT_DIRECTORY_VAR_NAME "=" TEST_RESOURCES_DIRECTORY);
  /* --------------------------
   * Namespace  : Custom (local)
   * Match type : Exact
   * --------------------------
   * Root: ${CMAKE_BINARY_DIR}/tests/resources
   * Namespace: /test_security_directory
   * Node: dummy_node
   */
  char * secure_root = rcl_get_secure_root(TEST_NODE_NAME, TEST_NODE_NAMESPACE, allocator);
  std::string secure_root_str(secure_root);
  ASSERT_STREQ(TEST_NODE_NAME, secure_root_str.substr(secure_root_str.size() - (sizeof(TEST_NODE_NAME) - 1)).c_str());

  /* --------------------------
   * Namespace  : Custom (local)
   * Match type : Prefix
   * --------------------------
   * Root: ${CMAKE_BINARY_DIR}/tests/resources
   * Namespace: /test_security_directory
   * Node: dummy_node_and_some_suffix_added */
  root_path = rcl_get_secure_root(TEST_NODE_NAME "_and_some_suffix_added", TEST_NODE_NAMESPACE, allocator);
  ASSERT_STRNE(root_path, secure_root);
  putenv_wrapper(ROS_SECURITY_LOOKUP_TYPE_VAR_NAME "=MATCH_PREFIX");
  root_path = rcl_get_secure_root(TEST_NODE_NAME "_and_some_suffix_added", TEST_NODE_NAMESPACE, allocator);
  ASSERT_STREQ(root_path, secure_root);
  allocator.deallocate(root_path, allocator.state);


  /* Include the namespace as part of the root security directory and test root namespace */
  char * base_lookup_dir_fqn = rcutils_join_path(TEST_RESOURCES_DIRECTORY,
      TEST_SECURITY_DIRECTORY_RESOURCES_DIR_NAME, allocator);
  std::string putenv_input = ROS_SECURITY_ROOT_DIRECTORY_VAR_NAME "=";
  putenv_input += base_lookup_dir_fqn;
  putenv_wrapper(putenv_input.c_str());
  /* --------------------------
   * Namespace  : Root
   * Match type : Exact
   * --------------------------
   * Root: ${CMAKE_BINARY_DIR}/tests/resources/test_security_directory
   * Namespace: /
   * Node: dummy_node */
  root_path = rcl_get_secure_root(TEST_NODE_NAME, ROOT_NAMESPACE, allocator);
  ASSERT_STREQ(root_path, secure_root);
  allocator.deallocate(root_path, allocator.state);
  putenv_wrapper(ROS_SECURITY_LOOKUP_TYPE_VAR_NAME "=MATCH_EXACT");
  root_path = rcl_get_secure_root(TEST_NODE_NAME, ROOT_NAMESPACE, allocator);
  ASSERT_STREQ(root_path, secure_root);
  allocator.deallocate(root_path, allocator.state);

  /* --------------------------
   * Namespace  : Root
   * Match type : Prefix
   * --------------------------
   * Root dir: ${CMAKE_BINARY_DIR}/tests/resources/test_security_directory
   * Namespace: /
   * Node: dummy_node_and_some_suffix_added */
  root_path = rcl_get_secure_root(TEST_NODE_NAME "_and_some_suffix_added", ROOT_NAMESPACE, allocator);
  ASSERT_STRNE(root_path, secure_root);
  putenv_wrapper(ROS_SECURITY_LOOKUP_TYPE_VAR_NAME "=MATCH_PREFIX");
  root_path = rcl_get_secure_root(TEST_NODE_NAME "_and_some_suffix_added", ROOT_NAMESPACE, allocator);
  allocator.deallocate(root_path, allocator.state);

  allocator.deallocate(secure_root, allocator.state);
  allocator.deallocate(base_lookup_dir_fqn, allocator.state);
}

TEST_F(TestGetSecureRoot, nodeSecurityDirectoryOverride) {
  /* Specify a valid directory */
  putenv_wrapper(ROS_SECURITY_NODE_DIRECTORY_VAR_NAME "=" TEST_RESOURCES_DIRECTORY);
  root_path = rcl_get_secure_root("name shouldn't matter", "namespace shouldn't matter", allocator);
  ASSERT_STREQ(root_path, TEST_RESOURCES_DIRECTORY);
  allocator.deallocate(root_path, allocator.state);

  /* Setting root dir has no effect */
  root_path = rcl_get_secure_root("name shouldn't matter", "namespace shouldn't matter", allocator);
  putenv_wrapper(ROS_SECURITY_ROOT_DIRECTORY_VAR_NAME "=" TEST_RESOURCES_DIRECTORY);
  ASSERT_STREQ(root_path, TEST_RESOURCES_DIRECTORY);
  allocator.deallocate(root_path, allocator.state);

  /* The override provided should exist. Providing correct node/namespace/root dir won't help
   * if the node override is invalid. */
  putenv_wrapper(
    ROS_SECURITY_NODE_DIRECTORY_VAR_NAME
    "=TheresN_oWayThi_sDirectory_Exists_hence_this_would_fail");
  ASSERT_EQ(rcl_get_secure_root(TEST_NODE_NAME, TEST_NODE_NAMESPACE, allocator),
    (char *) NULL);
}