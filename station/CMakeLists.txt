cmake_minimum_required(VERSION 2.8.3)
project(station)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(Qt4 REQUIRED)
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})
#ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

find_package(OpenCV)
#find_package(Phonon REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(qt_srcs
      src/main.cpp
      src/mainwindow.cpp
      src/painterWiget.cpp
   )
set(qt_hdrs
      src/mainwindow.h
      src/receiver.h
      src/painterWiget.h
   )
set(qt_resrcs src/Icons.qrc)

qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs} ${catkin_LIBRARIES} )
QT4_WRAP_UI(uis_h src/mainwindow.ui)
QT4_ADD_RESOURCES(qt_resrcs_RCC ${qt_resrcs})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
#uncomment if you have defined messages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

add_executable(receiver src/receiver.cpp 
    ${uis_h} ${qt_srcs} ${qt_moc_srcs} ${qt_resrcs_RCC})
target_link_libraries(receiver ${QT_LIBRARIES} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )


