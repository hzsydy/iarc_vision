cmake_minimum_required(VERSION 2.8.3)
project(iarc_webcam)

find_package(OpenCV REQUIRED)  
# add the publisher example  
add_executable(my_publisher src/my_publisher.cpp)  
  
target_link_libraries(my_publisher ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})  
  
# add the subscriber example  
add_executable(my_subscriber src/my_subscriber.cpp)  
target_link_libraries(my_subscriber ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}) 
