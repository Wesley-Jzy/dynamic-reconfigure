cmake_minimum_required(VERSION 3.5)

project(dynamic_reconfigure)

if(NOT CMAKE_CXX_STANDARD)
set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
# we dont use add_compile_options with pedantic in message packages
# because the Python C extensions dont comply with it
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)

find_package(python_cmake_module REQUIRED)
find_package(PythonExtra MODULE REQUIRED)


set(_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  set(PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE_DEBUG}")
endif()

ament_python_install_package(${PROJECT_NAME})

function(set_properties _targetname _build_type)
        set_target_properties(${_targetname} PROPERTIES
                    PREFIX ""
                    LIBRARY_OUTPUT_DIRECTORY${_build_type}
                    "${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}"
                    RUNTIME_OUTPUT_DIRECTORY${_build_type}
                    "${CMAKE_CURRENT_BINARY_DIR}/test_${PROJECT_NAME}"
                    OUTPUT_NAME
                    "_${_targetname}${PythonExtra_EXTENSION_SUFFIX}"
                    SUFFIX
                    "${PythonExtra_EXTENSION_EXTENSION}")
endfunction()

# Only build the library if a C typesupport exists
        get_rmw_typesupport(typesupport_impls "rmw_implementation" LANGUAGE
                    "c")
        if(typesupport_impls STREQUAL "")
        message(STATUS "Skipping rclpy because no C typesupport library was
                    found.")
return()
endif()

function(configure_python_c_extension_library _library_name)
        set_properties(${_library_name} "")
   if(WIN32)
        set_properties(${_library_name} "_DEBUG")
        set_properties(${_library_name} "_MINSIZEREL")
        set_properties(${_library_name} "_RELEASE")
        set_properties(${_library_name} "_RELWITHDEBINFO")
   endif()
        target_link_libraries(${_library_name}
                    log4cxx
                    ${PythonExtra_LIBRARIES}
                    )

        target_include_directories(${_library_name}
                    PUBLIC
                    ${PythonExtra_INCLUDE_DIRS}
                    )

        install(TARGETS ${_library_name}
                    DESTINATION
                    "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
               )
endfunction()

add_library(dynamic_reconfigure_ SHARED dynamic_reconfigure/_dynamic_reconfigure.cpp)
configure_python_c_extension_library(dynamic_reconfigure_)
ament_target_dependencies(dynamic_reconfigure_ "rcl" "rcutils" "rclcpp")


include_directories(include)
include_directories(cfg)
add_executable(test_parameters demo/test_parameters.cpp)
ament_target_dependencies(test_parameters rclcpp rcutils)
install(TARGETS test_parameters DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(FILES cfg/parameter_generator_catkin.py  DESTINATION "${PYTHON_INSTALL_DIR}")


ament_export_include_directories(include)
ament_package()
