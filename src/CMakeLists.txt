project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

set(EXECUTABLE path_planning)

####################################################
## TO SWITCH TO THE RELEASE TARGET COMMENT NEXT LINE
####################################################

# set(MY_BUILD_GOAL Debug)

# Support for Threads both on Linux and Windows
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

set(COMMON_SRC utils.cpp jmt.cpp optim_jmt.cpp points.cpp trajectory.cpp)

####################################################
# BUILD FOR DEBUG
####################################################

if(${MY_BUILD_GOAL} MATCHES "Debug") 
  include_directories("./")
  add_definitions(-std=c++14 -O0 -g3 -Wall -Wextra -pedantic)
  set(MY_LIBRARIES Threads::Threads)
  set(sources ${COMMON_SRC} test/test.cpp)

else(${MY_BUILD_GOAL} MATCHES "Debug")
####################################################
# BUILD FOR RELEASE
####################################################

  add_definitions(-std=c++14 -O0 -Wall)
  set(sources ${COMMON_SRC} main.cpp)
  set(MY_LIBRARIES Threads::Threads z ssl uv  uWS)

endif(${MY_BUILD_GOAL} MATCHES "Debug") 

####################################################
# DEFINE EXECUTABLE
####################################################

add_executable(${EXECUTABLE} ${sources})

####################################################
# OPTIONS FOR MACOS BUILDLD
####################################################

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")

####################################################
# ADD LIBRARY FOR RELEASE TARGET
####################################################

target_link_libraries(${EXECUTABLE} ${MY_LIBRARIES})
