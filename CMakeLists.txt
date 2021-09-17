cmake_minimum_required(VERSION 3.5.1)

project(smart_robot_platform)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(Qt5_DIR "/opt/Qt5.14.2/5.14.2/gcc_64/lib/cmake/Qt5")
# Find includes in the build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
SET(CMAKE_CXX_FLAGS "-std=c++14 -O3")
# Turn on automatic invocation of the MOC, UIC & RCC
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
SET(CMAKE_BUILD_TYPE DEBUG)
##rtde
set(DIR ${CMAKE_CURRENT_SOURCE_DIR})
message(STATUS "${DIR}")
##rtde end
# There may be a way to tell up front if Qt5 is going to be found, but I haven't found
# a foolproof way to do it yet, so settle for the default error message for now.
#if(NOT CMAKE_PREFIX_PATH AND NOT Qt5Widgets_DIR)
#    message(WARNING "CMAKE_PREFIX_PATH is not defined, so find_package may not work. Set the CMAKE_PREFIX_PATH "
#            "environment variable to the install prefix of Qt 5, either on the command line as "
#            "-DCMAKE_PREFIX_PATH=\"path/to/Qt5/lib/cmake\" or with set(CMAKE_PREFIX_PATH path/to/Qt5/lib/cmake)")
#endif(NOT CMAKE_PREFIX_PATH AND NOT Qt5Widgets_DIR)

# Add a compiler flag
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Make this a GUI application on Windows
if(WIN32)
  set(CMAKE_WIN32_EXECUTABLE ON)
endif()
# include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(
    /home/ray/eigen
)
link_directories(
    /home/ray/eigen/build/Eigen
)
include_directories(
    /home/ray/dlib
)

link_directories(
    /home/ray/dlib/build/dlib
)

#find_package(ur_rtde REQUIRED PATHS "${DIR}/../../Build/ur_rtde" "${DIR}/../../build/ur_rtde")
find_package(ur_rtde REQUIRED PATHS "/home/ray/ur_rtde/build/ur_rtde")
# Find the QtWidgets library
find_package(Qt5 COMPONENTS Widgets Network Charts REQUIRED)
# Tell CMake to create the helloworld executable
add_executable(smart_robot_platform main.cpp my_robot_loop.cpp robot_assemble_env.cpp teach_repeat.cpp drag_controller.cpp admittance_controller.cpp my_robot_control.cpp my_robot_loop.cpp my_robot_move.cpp my_force_data.cpp my_force_data_one.cpp my_force_data_two.cpp my_force_sensor.cpp my_force_sensor_one.cpp my_force_sensor_two.cpp my_math.cpp mainwindow.cpp mainwindow.ui)
# Add the Qt5 Widgets for linking
target_link_libraries(smart_robot_platform Qt5::Widgets Qt5::Network Qt5::Charts ur_rtde::rtde)
target_link_libraries(smart_robot_platform dlib)
