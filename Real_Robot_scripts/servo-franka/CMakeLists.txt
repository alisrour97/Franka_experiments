
project(franka)

cmake_minimum_required(VERSION 3.0)

find_package(VISP REQUIRED)

include_directories(${VISP_INCLUDE_DIRS})

#add_executable(frankaSavePosition frankaSavePosition.cpp)
#target_link_libraries(frankaSavePosition ${VISP_LIBRARIES})

set(example_cpp
  frankaGripper.cpp
  frankaMoveToPosition.cpp
  frankaSavePosition.cpp
  test_csv.cpp
  dynamic_controller.cpp
  kinematic_controller.cpp
  kinematic_controller_2.cpp
  dynamic_controller_2.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()



