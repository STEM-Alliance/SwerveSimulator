cmake_minimum_required(VERSION 3.10)
project(wpitest)

# Set the C++ standard to C++11 or later
set(CMAKE_CXX_STANDARD 20)

# Define the executable and its source files
add_library(wpitest SHARED main.cpp)

# Add include directories if necessary
include_directories("allwpilib/wpiutil/src/main/native/include")
include_directories("allwpilib/wpiutil/src/main/native/thirdparty/fmtlib/include")
include_directories("allwpilib/wpimath/src/main/native/thirdparty/gcem/include")
include_directories("allwpilib/wpimath/src/main/native/thirdparty/eigen/include")
include_directories("allwpilib/wpimath/src/main/native/include")

# Add link directories if necessary
target_link_directories(wpitest PUBLIC "wpilib_build/lib")

# # Link the executable with the foobar library
target_link_libraries(wpitest "wpimath")

# # Specify the location of the foobar library at runtime
set_target_properties(wpitest PROPERTIES
    INSTALL_RPATH "build/lib"
)
