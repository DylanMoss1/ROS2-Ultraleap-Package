cmake_minimum_required(VERSION 3.5)
project(ultraleap)

# --- BEGIN set cmake and c++ compiler standards ---

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# --- END set cmake and c++ compiler standards ---


# --- BEGIN set up ultraleap --- 

set(ULTRALEAP_PATH_ROOT "/usr/share/doc/ultraleap-hand-tracking-service/lib/cmake/LeapSDK")

find_package(LeapSDK
	5
	REQUIRED
	PATHS
		"${ULTRALEAP_PATH_ROOT}")

find_package(Threads REQUIRED)

get_target_property(
	LEAPC_IMPORTED_CONFIG
	LeapSDK::LeapC
	IMPORTED_CONFIGURATIONS
)

get_target_property(
	LEAPC_SHARED_LIB_PATH
	LeapSDK::LeapC
	IMPORTED_LOCATION_${LEAPC_IMPORTED_CONFIG}
)

add_library(
	libDeviceConnection
	OBJECT
	"src/device_connection.c")

target_link_libraries(
	libDeviceConnection
	PUBLIC
	LeapSDK::LeapC)

target_link_libraries(
	libDeviceConnection
	PRIVATE
	Threads::Threads)

target_include_directories(
	libDeviceConnection
	PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR})

function(add_device_connection sample_name sample_source_file)
  add_executable(${sample_name} ${sample_source_file})
  set_property(TARGET ${sample_name} PROPERTY FOLDER "src")
	target_link_libraries(${sample_name} PUBLIC libDeviceConnection)
endfunction()

add_device_connection("ultraleap_joint_info_publisher" "src/ultraleap_joint_info_publisher.cpp")

# --- END set up ultraleap


# --- BEGIN find dependencies ---

find_package(ament_cmake REQUIRED)  # <!-- try removing this for python-only? --> 
find_package(rclcpp REQUIRED)  # <!-- remove for python-only -->
find_package(ultraleap_msgs REQUIRED)

# --- END find dependencies ---


# --- BEGIN add launch file --- 

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# --- END add launch file --- 


# --- BEGIN add c++ executables --- <!-- remove for python-only -->

ament_target_dependencies(ultraleap_joint_info_publisher PUBLIC rclcpp ultraleap_msgs)  # <!-- remove std_msgs if not needed by ros2_publisher -->

install(TARGETS
  ultraleap_joint_info_publisher
	DESTINATION lib/${PROJECT_NAME}
	)

# --- END add c++ executables --- 


# --- BEGIN final ament configuration --- 

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# --- END final ament configuration --- 

ament_package()
