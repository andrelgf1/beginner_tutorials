cmake_minimum_required(VERSION 2.8.3)
project(beginner_tutorials)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  tf
  genmsg
  
)
add_service_files(

   FILES

   changeBaseOtput.srv

 )

generate_messages(
  DEPENDENCIES
  std_msgs
)

add_compile_options(-std=c++11)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker beginner_tutorials_generate_messages_cpp)

add_executable(listener src/listener.cpp )
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener beginner_tutorials_generate_messages_cpp)

if(CATKIN_ENABLE_TESTING)

  find_package(rostest REQUIRED)

  add_rostest_gtest(

    testTalkerNode
    
    test/test.launch
    
    test/main.cpp

    test/test.cpp

  )

  target_link_libraries( testTalkerNode ${catkin_LIBRARIES})

  add_dependencies(

    testTalkerNode

    talker

    
    ${catkin_EXPORTED_TARGETS}

  )


endif()

