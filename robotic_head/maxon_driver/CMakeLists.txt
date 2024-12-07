cmake_minimum_required(VERSION 3.14.4)
project(maxon_driver)

add_compile_options(-std=c++17)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(maxon_interfaces REQUIRED)

###########
## Build ##
###########

## Declare a C++ library
add_library(my_library
  src/EposCommunication.cpp
)
target_include_directories(my_library PUBLIC
 "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
 "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(my_library
  rclcpp::rclcpp
  EposCmd
)



## Declare a C++ executable
add_executable(${PROJECT_NAME} src/${PROJECT_NAME}_node.cpp src/EposController.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
   "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
   "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
   )

target_link_libraries(${PROJECT_NAME}
  rclcpp::rclcpp
  my_library
  )
ament_target_dependencies(${PROJECT_NAME} maxon_interfaces std_srvs std_msgs)

#############
## Install ##
#############

install(TARGETS my_library EXPORT export_my_library
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

ament_export_targets(export_my_library)

ament_package()
