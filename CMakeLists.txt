cmake_minimum_required(VERSION 2.8.3)
project(vrep_test)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rqt_gui
  rqt_gui_cpp
  message_generation
  image_transport
  geometry_msgs
  cv_bridge
  tf2
  tf2_ros
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()


 add_message_files(
   FILES
   JointAngles.msg
   IK_msg.msg
   HandJointAngles.msg
   JointVisualization.msg
 )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/Eigen3)

add_service_files(DIRECTORY srv FILES InertiaPara.srv)


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )

catkin_package(
  LIBRARIES ${PROJECT_NAME}
CATKIN_DEPENDS roscpp rqt_gui_cpp std_msgs message_runtime cv_bridge geometry_msgs image_transport
  #INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
)

set (vrep_test_SRCS
  src/ratio_layouted_frame.cpp
  src/my_plugin.cpp
  src/XR1.cpp

)

set(vrep_test_HDRS
  include/ratio_layouted_frame.h
  include/my_plugin.h
  include/XR1.h
)

set(vrep_test_UIS
  src/my_plugin.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(vrep_test_MOCS ${vrep_test_HDRS})
  qt5_wrap_ui(vrep_test_UIS_H ${vrep_test_UIS})
else()
  qt4_wrap_cpp(vrep_test_MOCS ${vrep_test_HDRS})
  qt4_wrap_ui(vrep_test_UIS_H ${vrep_test_UIS})
endif()


include_directories(
 include

 ${CMAKE_CURRENT_BINARY_DIR}/..
  ${catkin_INCLUDE_DIRS}
  #${Eigen_INCLUDE_DIRS}
)

## Declare a C++ library
 add_library(${PROJECT_NAME}
    ${vrep_test_SRCS}
    ${vrep_test_MOCS}
    ${vrep_test_UIS_H}

)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/vrep_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

