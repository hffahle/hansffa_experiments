cmake_minimum_required(VERSION 2.8.3)
project(hansffa_experiments)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -ltbb -D SFERES_XML_WRITE")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}")

set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF)

find_package(message_generation)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
  genmsg
  dynamic_reconfigure
  dyret_common
  dyret_utils
  dyret_hardware
  gazebo_ros
)

find_package(Boost REQUIRED COMPONENTS 
  program_options
  serialization
  graph
)

find_package(TBB REQUIRED)

find_package(OpenCV REQUIRED PATHS /usr/local/share/OpenCV NO_DEFAULT_PATH)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  roscpp
  gazebo_ros
  
)

link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${TBB_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

add_executable(exp1Gui
               src/exp1Gui.cpp
        )
target_link_libraries(exp1Gui ${catkin_LIBRARIES})
target_link_libraries(exp1Gui ${Boost_LIBRARIES})
target_link_libraries(exp1Gui ${TBB_LIBRARIES})
target_link_libraries(exp1Gui ${GAZEBO_LIBRARIES})
add_dependencies(exp1Gui
  ${catkin_EXPORTED_TARGETS})

add_executable(hardwareTest
               src/hardwareTest.cpp
              )
target_link_libraries(hardwareTest ${catkin_LIBRARIES})
add_dependencies(hardwareTest
  ${catkin_EXPORTED_TARGETS})

add_executable(heightmapGen
               src/heightmapGen.cpp
               )
target_link_libraries(heightmapGen ${OpenCV_LIBS})
add_dependencies(heightmapGen
  ${catkin_EXPORTED_TARGETS})

add_executable(meshGen
               src/meshGen.cpp
               )
target_link_libraries(meshGen ${OpenCV_LIBS})
add_dependencies(meshGen
  ${catkin_EXPORTED_TARGETS})





             
            

