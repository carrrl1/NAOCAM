cmake_minimum_required(VERSION 2.8)
project(naocam)
find_package(qibuild)
#find_package(OpenCV 3.0)

#message(STATUS "${OpenCV_INCLUDE_DIRS}")
#include_directories( ${OpenCV_INCLUDE_DIRS} )

option(NAOCAM_IS_REMOTE
  "Module is compiled as a remote module (ON or OFF)"
  ON)

#set(CMAKE_CXX_FLAGS "-stdlib=libc++")

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

qi_use_lib(naocam
  OPENCV3_CALIB3D
  OPENCV3_CORE
  OPENCV3_FEATURES2D
  OPENCV3_FLANN
  OPENCV3_HIGHGUI
  OPENCV3_IMGCODECS
  OPENCV3_IMGPROC
  OPENCV3_ML
  OPENCV3_OBJDETECT
  OPENCV3_VIDEO
  OPENCV3_VIDEOIO
  ALCOMMON ALVISION)
