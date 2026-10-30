if("dxf_import" IN_LIST MOTION_SELECTED_COMPONENTS)
  motion_compiled_component(dxf_import SOURCES dxf_path_import.cpp)
endif()
if("model_import" IN_LIST MOTION_SELECTED_COMPONENTS)
  motion_yaml_dependency()
  motion_xml_dependency()
  motion_compiled_component(model_import
    SOURCES robot_model_import.cpp
    PRIVATE tinyxml2::tinyxml2 yaml-cpp::yaml-cpp
    DEFINITIONS MOTION_WITH_MODEL_IMPORT=1)
endif()
