cmake_minimum_required(VERSION 3.8)
project(aura_examples)
find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/scenario_player
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
