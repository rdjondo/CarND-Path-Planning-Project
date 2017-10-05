project(Path_Planning)

cmake_minimum_required (VERSION 3.5)

set(EXECUTABLE path_planning)

set(MY_BUILD_GOAL Debug)
# set(MY_BUILD_GOAL Release)



####################################################
# BUILD FOR DEBUG
####################################################

if(${MY_BUILD_GOAL} MATCHES "Debug") 
  include_directories("./")
  add_definitions(-std=c++11 -O1 -g -Wall)
  set(sources utils.cpp jmt.cpp optim_jmt.cpp test/test.cpp)
endif(${MY_BUILD_GOAL} MATCHES "Debug") 

####################################################
# BUILD FOR RELEASE
####################################################
if(${MY_BUILD_GOAL} MATCHES "Release")

  add_definitions(-std=c++11 -O3)
  set(sources utils.cpp jmt.cpp optim_jmt.cpp test/test.cpp)

endif(${MY_BUILD_GOAL} MATCHES "Release") 

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
if(${MY_BUILD_GOAL} MATCHES "Release")

  target_link_libraries(${EXECUTABLE} z ssl uv  uWS)

endif(${MY_BUILD_GOAL} MATCHES "Release")
