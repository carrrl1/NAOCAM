cmake_minimum_required(VERSION 2.8)
project(naocam)
find_package(qibuild)

option(NAOCAM_IS_REMOTE
  "Module is compiled as a remote module (ON or OFF)"
  ON)

set(CMAKE_CXX_FLAGS "-stdlib=libstdc++")

set(_srcs
  main.cpp
  naocam.h
  naocam.cpp
)


if(NAOCAM_IS_REMOTE)
  add_definitions(" -DNAOCAM_IS_REMOTE")
  qi_create_bin(naocam ${_srcs})
else()
  qi_create_lib(naocam SHARED ${_srcs} SUBFOLDER naoqi)
endif()


qi_use_lib(naocam OPENCV2_CORE OPENCV2_HIGHGUI OPENCV2_IMGPROC OPENCV2_OBJDETECT OPENCV2_FEATURES2D OPENCV2_CALIB3D ALCOMMON ALVISION)
