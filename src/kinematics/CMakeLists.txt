motion_eigen_dependency()
if("serial_kinematics" IN_LIST MOTION_SELECTED_COMPONENTS)
  motion_compiled_component(serial_kinematics
    SOURCES serial_chain_kinematics.cpp
    PRIVATE motion_eigen_headers
    DEFINITIONS MOTION_WITH_SERIAL=1)
endif()
if("differential_kinematics" IN_LIST MOTION_SELECTED_COMPONENTS)
  motion_compiled_component(differential_kinematics
    SOURCES inverse_differential.cpp
    PRIVATE motion_eigen_headers
    DEFINITIONS MOTION_WITH_DIFFERENTIAL=1)
endif()
