cmake_minimum_required(VERSION 3.8)
project(mbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# install
install(DIRECTORY config launch maps rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/bruteforce_frontier_explorer.py
                scripts/wavefront_frontier_explorer.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
