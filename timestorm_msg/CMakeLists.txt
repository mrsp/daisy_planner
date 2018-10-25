cmake_minimum_required(VERSION 2.8.3)
project(timestorm_msg)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)
find_package(std_msgs)

add_message_files(
   FILES
   Fruit_id.msg
   Daisy.msg
   Time.msg
   Query.msg
   Robot.msg
   Action.msg
   Petal.msg
   Daisy_graph.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs 
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)
