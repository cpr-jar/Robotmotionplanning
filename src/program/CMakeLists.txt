motion_yaml_dependency()
motion_compiled_component(program
  SOURCES program_import.cpp prepared_robot_program.cpp program_planning.cpp program_dxf.cpp program_simulation.cpp
  PRIVATE yaml-cpp::yaml-cpp
  DEFINITIONS MOTION_WITH_PROGRAM=1)
