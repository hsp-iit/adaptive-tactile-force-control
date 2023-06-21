cmake_minimum_required(VERSION 3.10)

project(xela_2f_force_control)

find_package(catkin REQUIRED COMPONENTS rospy xela_server std_msgs message_generation robotiq_2f_gripper_control)

catkin_python_setup()

add_message_files(
  FILES
  SetPointTarget.msg
  ControllerStatus.msg
  LoggerCommand.msg
)

add_service_files(
  FILES
  Setpoint.srv
  SetContactPosition.srv
  SetControllerGain.srv
  SetControllerStatus.srv
  SetControllerType.srv
  SetFilterCutoff.srv
  SetLoggerCommand.srv
  SetLoggerPath.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(CATKIN_DEPENDS rospy xela_server robotiq_2f_gripper_control)

install(PROGRAMS
     nodes/controller_node
     nodes/setpoint_generator_node
     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
