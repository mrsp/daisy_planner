cmake_minimum_required(VERSION 2.8.3)
project(telephone_data)

# find_package(catkin REQUIRED COMPONENTS)

find_package(catkin REQUIRED COMPONENTS   
   rospy
   std_msgs
   message_generation
)


add_message_files(
  FILES
  telephony_msg.msg
)

# SET(ENV{PYTHONPATH} "src")
# INCLUDE_DIRECTORIES( "src")
## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination

# set( nodes 
#     nodes/footstep_handler.py
# )
catkin_python_setup()



generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package()
set( nodes 
    src/telephone_data/main
)



# catkin_install_python(PROGRAMS
#    ${nodes}
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

catkin_install_python(PROGRAMS
    ${nodes}
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
