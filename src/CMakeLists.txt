add_library(robot_motion_serial STATIC
  kinematics/serial_chain_kinematics.cpp
  kinematics/inverse_differential.cpp
  calibration/rigid_calibration.cpp
  io/robot_model_import.cpp
  program/program_import.cpp
  program/program_planning.cpp
  program/program_simulation.cpp)
add_library(RobotMotion::serial ALIAS robot_motion_serial)
target_include_directories(robot_motion_serial PUBLIC ${PROJECT_SOURCE_DIR}/include)
FetchContent_GetProperties(motion_xml SOURCE_DIR motion_xml_source)
FetchContent_GetProperties(motion_yaml SOURCE_DIR motion_yaml_source)
target_include_directories(robot_motion_serial SYSTEM PRIVATE ${motion_xml_source} ${motion_yaml_source}/include)
target_compile_definitions(robot_motion_serial PUBLIC MOTION_WITH_SERIAL=1)
target_link_libraries(robot_motion_serial PRIVATE motion_eigen_headers tinyxml2::tinyxml2 yaml-cpp::yaml-cpp)
motion_strict_target(robot_motion_serial)
