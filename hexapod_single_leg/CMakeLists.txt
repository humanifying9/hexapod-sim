cmake_minimum_required(VERSION 3.8)

# This CMakeLists.txt is intentionally minimal — ament_python uses setup.py
# for Python packages. We still need a stub CMakeLists so colcon can detect
# the package, but all the real configuration lives in setup.py.

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

find_package(ament_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
