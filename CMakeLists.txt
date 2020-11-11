project(beginner_tutorials)

## For compiling C++11 functions
## add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation genmsg)

## Set C++11 standard for cmake project
set(CMAKE_CXX_STANDARD 11)

##Declared ROS messages and services
add_service_files(DIRECTORY srv FILES UpdateString.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  genmsg
  tf
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)

  add_rostest_gtest(
    talker_test
    test/talker_test.launch
    test/main_test.cpp
    test/talker_test.cpp
  )
  target_link_libraries(
    talker_test
    ${catkin_LIBRARIES}
  )
  add_dependencies(
    talker 
    talker_test 
    ${catkin_EXPORTED_TARGETS}
    )
endif()

