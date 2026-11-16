cmake_minimum_required(VERSION 3.8)
project(mcl_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# Garante que o executavel esteja em lib/mcl_localization/ (local esperado pelo ros2 launch)
install(CODE "
execute_process(
  COMMAND \${CMAKE_COMMAND} -E make_directory \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}
  COMMAND \${CMAKE_COMMAND} -E copy \${CMAKE_CURRENT_SOURCE_DIR}/lib/mcl_localization/mcl_node \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/mcl_node
  COMMAND \${CMAKE_COMMAND} -E chmod +x \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/mcl_node
  WORKING_DIRECTORY \${CMAKE_CURRENT_SOURCE_DIR}
)
")

ament_package()
