include(FetchContent)
FetchContent_Declare(motion_kdl SOURCE_SUBDIR no_upstream_build URL ${PROJECT_SOURCE_DIR}/third_party/source_archives/orocos-kdl-reference.tar.gz URL_HASH SHA256=07b8884ba711dd745e018ccd27e96d716257457fa2787b7412bae5c2509e4188)
FetchContent_MakeAvailable(motion_kdl)
set(kdl_source ${motion_kdl_SOURCE_DIR}/orocos_kdl/src)
set(KDL_VERSION_MAJOR 1)
set(KDL_VERSION_MINOR 5)
set(KDL_VERSION_PATCH 1)
set(KDL_VERSION 1.5.1)
configure_file(${kdl_source}/config.h.in ${CMAKE_CURRENT_BINARY_DIR}/config.h @ONLY)
# Unmodified upstream sources for the FK/J reference only; no KDL IK is used.
add_library(motion_kdl_reference STATIC
 ${kdl_source}/frames.cpp ${kdl_source}/frames_io.cpp ${kdl_source}/joint.cpp
 ${kdl_source}/segment.cpp ${kdl_source}/chain.cpp ${kdl_source}/jntarray.cpp
 ${kdl_source}/jacobian.cpp ${kdl_source}/chainfksolverpos_recursive.cpp
 ${kdl_source}/chainjnttojacsolver.cpp ${kdl_source}/utilities/utility.cxx
 ${kdl_source}/rotationalinertia.cpp ${kdl_source}/rigidbodyinertia.cpp)
target_include_directories(motion_kdl_reference SYSTEM PUBLIC ${kdl_source} ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(motion_kdl_reference PUBLIC motion_eigen_headers)
