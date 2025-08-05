cmake_minimum_required(VERSION 3.5)
project(youbot)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(rclpy REQUIRED)

# Install Python scripts
install(PROGRAMS
  myyoubot/blinker.py
  myyoubot/RelayControl.py
  myyoubot/MessageLogger.py
  myyoubot/SimRealSelector.py
  myyoubot/OpcUaClient.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install resource files (like ament index marker)
install(FILES
  package.xml
  resource/youbot
  DESTINATION share/ament_index/resource_index/packages
)

# Install launch/config/description/worlds/etc
file(GLOB_RECURSE launch_files launch/*)
file(GLOB_RECURSE config_files config/*)
file(GLOB_RECURSE desc_files description/*)
file(GLOB_RECURSE world_files worlds/*)
file(GLOB_RECURSE view_files views/*)
file(GLOB_RECURSE myyoubot_files myyoubot/*)

install(FILES ${launch_files} DESTINATION share/${PROJECT_NAME}/launch)
install(FILES ${config_files} DESTINATION share/${PROJECT_NAME}/config)
install(FILES ${desc_files} DESTINATION share/${PROJECT_NAME}/description)
install(FILES ${world_files} DESTINATION share/${PROJECT_NAME}/worlds)
install(FILES ${view_files} DESTINATION share/${PROJECT_NAME}/views)
install(FILES ${myyoubot_files} DESTINATION share/${PROJECT_NAME}/myyoubot)

# Mesh folders
foreach(mesh_subdir sensors youbot_arm youbot_base youbot_gripper youbot_plate)
  file(GLOB_RECURSE mesh_files meshes/${mesh_subdir}/*)
  install(FILES ${mesh_files} DESTINATION share/${PROJECT_NAME}/meshes/${mesh_subdir})
endforeach()


rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SrvString.srv"
)



# Export
ament_package()

