cmake_minimum_required(VERSION 3.8)
project(dart_description)

if(CMAKE_COMPILER_IS_GNUCXX
   OR CMAKE_CXX_COMPILER_ID
      MATCHES
      "Clang"
)
  add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
  )
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
# uncomment the following section in order to fill in further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config
            launch
            urdf
            rviz
            meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/sensor_fov_visualizer.py
           scripts/scene_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(
  target_geometry
  PACKAGE_DIR target_geometry
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(rm2027_geometry test/test_rm2027_geometry.py)
  ament_add_pytest_test(target_geometry_api test/test_target_geometry.py)
endif()

ament_package()
