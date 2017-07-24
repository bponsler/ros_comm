cmake_minimum_required(VERSION 3.5)
project(message_filters)

# Add support for C++11
if(NOT WIN32)
  add_definitions(-std=c++11)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ros2_console REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

set(INCLUDE_DIRS include ${ament_cmake_INCLUDE_DIRS} ${rospack_INCLUDE_DIRS}
  ${ros2_console_INCLUDE_DIRS} ${rclcpp_INCLUDE_DIRS} ${std_msgs_INCLUDE_DIRS})
include_directories(${INCLUDE_DIRS})

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS} ${rospack_LIBRARY_DIRS}
  ${ros2_console_LIBRARY_DIRS} ${rclcpp_LIBRARY_DIRS} ${std_msgs_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES} ${rospack_LIBRARIES} ${ros2_console_LIBRARIES}
  ${rclcpp_LIBRARIES} ${std_msgs_LIBRARIES})

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

find_package(Boost REQUIRED COMPONENTS signals thread)

add_library(${PROJECT_NAME} SHARED src/connection.cpp)
target_link_libraries(${PROJECT_NAME} ${LIBS} ${Boost_LIBRARIES})

if(WIN32)
  # Causes the visibility macros to use dllexport rather than dllimport,
  # which is appropriate when building the dll but not consuming it.
  target_compile_definitions(${PROJECT_NAME}
    PRIVATE "MESSAGE_FILTERS_BUILDING_DLL")
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION bin)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Ugly workaround for check_test_ran macro issue
  #add_subdirectory(test)

  # ********** Tests **********
  ament_add_gtest(${PROJECT_NAME}-msg_cache_unittest test/msg_cache_unittest.cpp)
  if(TARGET ${PROJECT_NAME}-msg_cache_unittest)
    target_link_libraries(${PROJECT_NAME}-msg_cache_unittest message_filters
      ${GTEST_LIBRARIES})
  endif()

  #ament_add_gtest(${PROJECT_NAME}-time_synchronizer_unittest
  #  test/time_synchronizer_unittest.cpp)
  #if(TARGET ${PROJECT_NAME}-time_synchronizer_unittest)
  #  target_link_libraries(${PROJECT_NAME}-time_synchronizer_unittest message_filters
  #    ${GTEST_LIBRARIES})
  #endif()

  #ament_add_gtest(${PROJECT_NAME}-test_synchronizer test/test_synchronizer.cpp)
  #if(TARGET ${PROJECT_NAME}-test_synchronizer)
  #  target_link_libraries(${PROJECT_NAME}-test_synchronizer message_filters
  #    ${GTEST_LIBRARIES})
  #endif()

  #ament_add_gtest(${PROJECT_NAME}-test_exact_time_policy
  #  test/test_exact_time_policy.cpp)
  #if(TARGET ${PROJECT_NAME}-test_exact_time_policy)
  #  target_link_libraries(${PROJECT_NAME}-test_exact_time_policy message_filters
  #    ${GTEST_LIBRARIES})
  #endif()

  #ament_add_gtest(${PROJECT_NAME}-test_approximate_time_policy
  #  test/test_approximate_time_policy.cpp)
  #if(TARGET ${PROJECT_NAME}-test_approximate_time_policy)
  #  target_link_libraries(${PROJECT_NAME}-test_approximate_time_policy
  #    message_filters ${GTEST_LIBRARIES})
  #endif()

  ament_add_gtest(${PROJECT_NAME}-test_simple test/test_simple.cpp)
  if(TARGET ${PROJECT_NAME}-test_simple)
    target_link_libraries(${PROJECT_NAME}-test_simple message_filters
      ${GTEST_LIBRARIES})
  endif()

  ament_add_gtest(${PROJECT_NAME}-test_chain test/test_chain.cpp)
  if(TARGET ${PROJECT_NAME}-test_chain)
    target_link_libraries(${PROJECT_NAME}-test_chain message_filters
      ${GTEST_LIBRARIES})
  endif()

  # Needs to be a rostest because it spins up a node, which blocks until it hears from the master (unfortunately)
  #add_rostest_gtest(${PROJECT_NAME}-time_sequencer_unittest
  #  test/time_sequencer_unittest.xml test/time_sequencer_unittest.cpp)
  #if(TARGET ${PROJECT_NAME}-time_sequencer_unittest)
  #  target_link_libraries(${PROJECT_NAME}-time_sequencer_unittest message_filters)
  #endif()

  ament_add_gtest(${PROJECT_NAME}-test_subscriber test/test_subscriber.cpp)
  if(TARGET ${PROJECT_NAME}-test_subscriber)
    target_link_libraries(${PROJECT_NAME}-test_subscriber message_filters)
  endif()

  # Unit test of the approximate synchronizer
  #ament_add_nose_test(${PROJECT_NAME}_approxsync_nose_test test/test_approxsync.py)
  #ament_add_nose_test(${PROJECT_NAME}_filters_nose_test
  #  test/test_message_filters_cache.py)
endif()

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rospack)
ament_export_dependencies(ros2_console)
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME} ${LIBS})

ament_package()
