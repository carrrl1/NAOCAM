cmake_minimum_required(VERSION 2.8)
project(naocam)
find_package(qibuild)
#find_package(OpenCV 3.0)

#message(STATUS "${OpenCV_INCLUDE_DIRS}")
#include_directories( ${OpenCV_INCLUDE_DIRS} )

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

qi_use_lib(naocam OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC OPENCV2_OBJDETECT OPENCV2_FEATURES2D OPENCV2_CALIB3D OPENCV2_VIDEO ALCOMMON ALVISION)
#qi_use_lib(naocam OPENCV2_HIGHGUI OPENCV_IMGPROC OPENCV_OBJDETECT OPENCV_FEATURES2D OPENCV_CALIB3D OPENCV_VIDEO ALCOMMON ALVISION)
#qi_use_lib(naocam ${OpenCV_LIBS} ALCOMMON ALVISION)
