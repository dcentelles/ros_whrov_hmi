cmake_minimum_required(VERSION 3.1.0)
project(whrov_hmi)
set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_COLOR_MAKEFILE   ON)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  std_srvs
  actionlib
  actionlib_msgs
  cv_bridge
  image_transport
  merbots_whrov_msgs
  tf
)
#http://doc.qt.io/qt-5/cmake-manual.html
# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)
#SET(CMAKE_AUTOUIC ON)

set(Qt5Core_DIR "~/programming/Qt/5.9.2/gcc_64/lib/cmake/Qt5Core")
set(Qt5Gui_DIR "~/programming/Qt/5.9.2/gcc_64/lib/cmake/Qt5Gui")
set(Qt5Widgets_DIR "~/programming/Qt/5.9.2/gcc_64/lib/cmake/Qt5Widgets")
set(Qt5_DIR "~/programming/Qt/5.9.2/gcc_64/lib/cmake/Qt5")
set(QT_QMAKE_EXECUTABLE "~/programming/Qt/5.9.2/gcc_64/bin/qmake")

# Find the QtWidgets library
find_package(Qt5Core REQUIRED)
find_package(Qt5Widgets REQUIRED)

generate_messages(DEPENDENCIES
    std_msgs
    actionlib_msgs
    )

add_definitions(-fPIC -std=c++14)

SET(PROJECT_INCLUDE_DIRS
    ${catkin_INCLUDE_DIRS}
    src/include
    /usr/local/qwt-6.1.4/include/
)

set(HW_HEADER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/include/whrov_hmi)
set(HW_GUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/gui)

#https://stackoverflow.com/questions/19761767/qt-5-cmake-fails-with-undefined-reference-to-vtable-on-hello-world-with-inc-sr
qt5_wrap_cpp(hmi_SRC
    ${HW_HEADER_DIR}/imageview.h
    ${HW_HEADER_DIR}/whrovmainwindow.h
    ${HW_HEADER_DIR}/qrosnode.h
    ${HW_HEADER_DIR}/attitude_indicator.h
    )
qt5_wrap_ui(hmi_UI
    ${HW_GUI_DIR}/whrovmainwindow.ui
    ${HW_GUI_DIR}/imageview.ui
    )

set(${PROJECT_NAME}_LIB ${PROJECT_NAME}_lib)

# Tell CMake to create the ${PROJECT_NAME} executable
add_library(${${PROJECT_NAME}_LIB}
        src/whrovmainwindow.cpp
        src/imageview.cpp
        src/qrosnode.cpp
        src/attitude_indicator.cpp
        ${hmi_SRC}
        ${hmi_UI}
	)

set(QT_DEPS
        Qt5::Widgets
        Qt5::Core
        /usr/local/qwt-6.1.4/lib/libqwt.so
        )
message("DETECTED QT VERSION: "${Qt5Core_VERSION})
message("DETECTED QT LIBRARIES: "${Qt5Core_INCLUDE_DIRS})
target_include_directories(${${PROJECT_NAME}_LIB} PUBLIC ${PROJECT_INCLUDE_DIRS})
target_link_libraries(${${PROJECT_NAME}_LIB}  ${QT_DEPS}  ${catkin_LIBRARIES})
add_dependencies(${${PROJECT_NAME}_LIB} merbots_whrov_msgs_generate_messages_cpp)

add_executable(${PROJECT_NAME} 
        src/main.cpp
	)

target_link_libraries(${PROJECT_NAME} ${${PROJECT_NAME}_LIB})

#add_dependencies(uniavan beginner_tutorials_generate_messages_cpp)
# Use the Widgets module from Qt 5.
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qt_prueba
    CATKIN_DEPENDS
    roscpp std_msgs
    message_runtime
    actionlib_msgs
    cv_bridge
    image_transport
    merbots_whrov_msgs
    tf
   DEPENDS system_lib
    ${QT_DEPS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)

## Declare a C++ library
# add_library(qt_prueba
#   src/${PROJECT_NAME}/qt_prueba.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(qt_prueba ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(qt_prueba_node src/qt_prueba_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(qt_prueba_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(qt_prueba_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS qt_prueba qt_prueba_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_qt_prueba.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
