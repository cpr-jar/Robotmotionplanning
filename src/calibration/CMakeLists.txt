motion_eigen_dependency()
motion_compiled_component(calibration
  SOURCES rigid_calibration.cpp
  PRIVATE motion_eigen_headers)
