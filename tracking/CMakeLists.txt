project(visp-test1)
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

FIND_PACKAGE(VISP REQUIRED)
IF(VISP_FOUND)
  INCLUDE(${VISP_USE_FILE}) # will load all external libraries will include files for subsequent binaries
ENDIF(VISP_FOUND)

add_executable(visp-test1 main.cpp)
# add_executable(visp-test2 servoTry.cpp)
add_executable(calibrateInt calibrateInt.cpp)

