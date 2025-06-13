cmake_minimum_required(VERSION 3.5)
project(final_project)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/red_circle_avoider.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch models worlds
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY env-hooks
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
