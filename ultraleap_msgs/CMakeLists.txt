cmake_minimum_required(VERSION 3.5)
project(ultraleap_msgs)

# --- BEGIN set cmake and c++ compiler standards ---

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- END set cmake and c++ compiler standards ---


# --- BEGIN find dependencies ---

find_package(ament_cmake REQUIRED)

# --- END find dependencies --- 


# --- BEGIN generate messages ---

find_package(rosidl_default_generators REQUIRED)

set(private_msg_files
)

set(public_msg_files
  "msg/public/DigitInfo.msg"
  "msg/public/FingerInfo.msg"
  "msg/public/JointInfo.msg"
  "msg/public/ThumbInfo.msg"
  "msg/public/UltraleapJointInfo.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${private_msg_files}
  ${public_msg_files}
  DEPENDENCIES  # <!-- add dependencies --> 
)

# --- END generate messages ---


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
