cmake_minimum_required(VERSION 2.8)
project(naocam)
find_package(qibuild)

#include_directories( ${OpenCV_INCLUDE_DIRS} )
#message(STATUS "${OpenCV_INCLUDE_DIRS}")

option(NAOCAM_IS_REMOTE
  "Module is compiled as a remote module (ON or OFF)"
  ON)

#set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")

set(srcs
  ./src/main.cpp
  ./src/naocam.h
  ./src/naocam.cpp
)

if(NAOCAM_IS_REMOTE)
  add_definitions(" -DNAOCAM_IS_REMOTE")
  qi_create_bin(naocam ${srcs})
else()
  qi_create_lib(naocam SHARED ${srcs} SUBFOLDER naoqi)
endif()

#message(STATUS "${OpenCV_LIBS}")
qi_use_lib(naocam
  ALCOMMON
  ALVISION
  ${OpenCV_LIBS})
