cmake_minimum_required(VERSION 3.0) 
PROJECT(TactileGraphicHelper VERSION 0.1)
#set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(BIN_NAME ${PROJECT_NAME})

#set(CMAKE_CXX_FLAGS "-std=c++11" CACHE STRING "compile flags" FORCE)
#set(CMAKE_CXX_STANDARD 11)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake_modules" ${CMAKE_MODULE_PATH})

set( NAME_SRC
    src/main.cpp
    src/SOM.cpp
    src/Lattice.cpp
    src/BackgroundGenerator.cpp
    src/FeatureInfo.cpp
    src/FingerTip.cpp
    src/FingerTracker.cpp
    src/TGH.cpp
    src/TGModel.cpp
    src/FrameGrabber.cpp
    src/robustline.cpp
   )


#Find OpenCV
find_package(OpenCV REQUIRED )
if (OpenCV_FOUND)
  message(STATUS "OpenCV Found")
  message(STATUS "OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
  message(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
  include_directories(${OpenCV_INCLUDE_DIRS})
endif()

#Copy resources
#set(BIN_NAME ${PROJECT_NAME})
IF(CMAKE_BUILD_TYPE MATCHES DEBUG)
    message(STATUS "Debug mode")
    set(TARGET_NAME ${TARGET_NAME}d)
else(CMAKE_BUILD_TYPE MATCHES DEBUG)
    message(STATUS "Release mode")
ENDIF(CMAKE_BUILD_TYPE MATCHES DEBUG)

set(HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${HEADER_DIR})
file(GLOB NAME_HEADERS "${HEADER_DIR}/*.h" "${HEADER_DIR}/*.hpp")

set(OUTPUT_FOLDER ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${OUTPUT_FOLDER})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${OUTPUT_FOLDER})
add_executable( ${BIN_NAME} ${NAME_SRC} ${NAME_HEADERS} )
#set_property(TARGET ${BIN_NAME} PROPERTY CXX_STANDARD 11)
#set_property(TARGET ${BIN_NAME} PROPERTY CXX_STANDARD_REQUIRED ON)
set_target_properties( ${BIN_NAME}
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_FOLDER}
)	
message(STATUS "Executables will be placed in ${OUTPUT_FOLDER}.")
#file(INSTALL res DESTINATION ${OUTPUT_FOLDER})

include_directories(${PROJECT_BINARY_DIR})
#include_directories(${SOM_BASE_DIR}/include)

TARGET_LINK_LIBRARIES(${BIN_NAME} opencv_core opencv_imgproc opencv_video opencv_objdetect opencv_highgui opencv_calib3d)