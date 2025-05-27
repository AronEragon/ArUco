cmake_minimum_required(VERSION 3.8)
project(robot_camera_processor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)


add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  image_transport
  geometry_msgs
  tf2
  tf2_ros
)


install(TARGETS
    ${PROJECT_NAME}_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()



# cmake_minimum_required(VERSION 3.8)
# project(robot_camera_processor)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)


# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
# ament_target_dependencies(${PROJECT_NAME}_node
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   OpenCV
#   image_transport
#   geometry_msgs
#   tf2
#   tf2_ros
# )

# install(TARGETS
#     ${PROJECT_NAME}_node
#     DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()



# cmake_minimum_required(VERSION 3.8)
# project(robot_camera_processor)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)
# find_package(tf2_ros REQUIRED)

# add_executable(robot_camera_processor src/robot_camera_processor.cpp)
# ament_target_dependencies(robot_camera_processor
#   rclcpp
#   sensor_msgs
#   cv_bridge
#   OpenCV
#   image_transport
#   geometry_msgs
#   tf2
#   tf2_ros
# )

# install(TARGETS
#   robot_camera_processor
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()


# cmake_minimum_required(VERSION 3.8)
# project(robot_camera_processor)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()


# find_package(ament_cmake REQUIRED)
# find_package(rclcpp REQUIRED)
# find_package(sensor_msgs REQUIRED)
# find_package(cv_bridge REQUIRED)
# find_package(OpenCV REQUIRED)
# find_package(image_transport REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(tf2 REQUIRED)

# add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

# ament_target_dependencies(${PROJECT_NAME}_node
#     rclcpp
#     sensor_msgs
#     cv_bridge
#     OpenCV
#     image_transport
#     geometry_msgs
#     tf2
# )



# install(
#   ${PROJECT_NAME}
#   DESTINATION lib/${PROJECT_NAME}
# )

# ament_package()
