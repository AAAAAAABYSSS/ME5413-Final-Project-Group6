cmake_minimum_required(VERSION 3.0.2)
project(interactive_tools)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  rviz
  pluginlib
)

find_package(Qt5 REQUIRED COMPONENTS Widgets)

set(UI_FILES
  resource/planner_selector_panel.ui
)

set(HEADER_FILES
  include/interactive_tools/planner_selector_panel.h
)

set(SRC_FILES
  src/planner_selector_panel.cpp
)

qt5_wrap_ui(UI_HEADERS ${UI_FILES})
qt5_wrap_cpp(MOC_HEADERS ${HEADER_FILES})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

add_library(${PROJECT_NAME}
  ${SRC_FILES}
  ${UI_HEADERS}
  ${MOC_HEADERS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  Qt5::Widgets
)

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES rviz_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
