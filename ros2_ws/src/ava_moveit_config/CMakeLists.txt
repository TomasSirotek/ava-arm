cmake_minimum_required(VERSION 3.16)
project(ava_moveit_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

install(FILES .setup_assistant
  DESTINATION share/${PROJECT_NAME}
)

set(AVA_SCRIPTS
  scripts/publish_cup_marker.py
  scripts/cup_pick_and_place.py
  scripts/arm_ik.py
  scripts/planned_path_line.py
  scripts/gazebo_grasp_attach.py
  scripts/spawn_cube.py
)
set(AVA_SCRIPT_PERMS
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
             GROUP_READ GROUP_EXECUTE
             WORLD_READ WORLD_EXECUTE
)

# share/: used by the launch files (FindPackageShare -> ExecuteProcess python3 <share>/<script>).
install(PROGRAMS ${AVA_SCRIPTS} DESTINATION share/${PROJECT_NAME} ${AVA_SCRIPT_PERMS})

# lib/<pkg>/: where `ros2 run <pkg> <script>` looks. MUST be kept in sync (incl. arm_ik.py,
# which cup_pick_and_place.py imports from its own directory). Without this, ros2 run executes
# a stale leftover copy here and ignores the current code.
install(PROGRAMS ${AVA_SCRIPTS} DESTINATION lib/${PROJECT_NAME} ${AVA_SCRIPT_PERMS})

ament_package()
