cmake_minimum_required(VERSION 3.5)
project(xmlrpcpp)

# Add support for C++11
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 11)
endif()

find_package(ament_cmake REQUIRED)

set(INCLUDE_DIRS
  include
  ${ament_cmake_INCLUDE_DIRS}
)
include_directories(${INCLUDE_DIRS} libb64/include)

set(LIBRARY_DIRS ${ament_cmake_LIBRARY_DIRS})

link_directories(${LIBRARY_DIRS})

set(LIBS ${ament_cmake_LIBRARIES})

if(NOT WIN32)
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "-Wall;-Wextra")
endif()

# The CFG_EXTRAS is only for compatibility, to be removed in Lunar.

if(WIN32)
  add_definitions(-D_WINDOWS)
endif()

add_library(xmlrpcpp SHARED
  src/XmlRpcClient.cpp
  src/XmlRpcDispatch.cpp
  src/XmlRpcServer.cpp
  src/XmlRpcServerConnection.cpp
  src/XmlRpcServerMethod.cpp
  src/XmlRpcSocket.cpp
  src/XmlRpcSource.cpp
  src/XmlRpcUtil.cpp
  src/XmlRpcValue.cpp
  libb64/src/cdecode.c
  libb64/src/cencode.c
)

if(WIN32)
  target_link_libraries(xmlrpcpp ws2_32)
endif()

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING
  PATTERN "*.h")

install(TARGETS xmlrpcpp
  ARCHIVE
  DESTINATION lib
  LIBRARY
  DESTINATION lib
  RUNTIME
  DESTINATION bin)

ament_export_dependencies(ament_cmake)
ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(xmlrpcpp ${LIBS})

ament_package(
  CONFIG_EXTRAS "cmake/xmlrpcpp-extras.cmake.in"
)
