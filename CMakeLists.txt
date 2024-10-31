cmake_minimum_required(VERSION 3.8)
project(elderly_activity_visualization)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python modules
ament_python_install_package(scripts)

# Install Python executables
install(PROGRAMS
  scripts/data_fetcher.py
  scripts/activity_processor.py
  scripts/activity_visualizer.py
  scripts/logo_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install package.xml and resource files
install(FILES
  package.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  resource/media
  DESTINATION share/${PROJECT_NAME}
)

# Create and install the marker file
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/resource/${PROJECT_NAME} "")
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages
)

# Set up Python entry points
set(PYTHON_SCRIPTS
  data_fetcher
  activity_processor
  activity_visualizer
)

foreach(script ${PYTHON_SCRIPTS})
  install(CODE
    "execute_process(
      COMMAND ${CMAKE_COMMAND} -E create_symlink
      ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${script}.py
      ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${script}
    )"
  )
endforeach()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(pytest_tests
    test/test_activity_processor.py
    test/test_data_fetcher.py
    test/test_visualization.py
  )
  foreach(test_file ${pytest_tests})
    ament_add_pytest_test(${test_file} ${test_file})
  endforeach()
endif()

ament_package()