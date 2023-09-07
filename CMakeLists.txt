cmake_minimum_required(VERSION 3.20)


# All the Libraries are installed using Vcpkg
set(CMAKE_TOOLCHAIN_FILE "D:/vcpkg/scripts/buildsystems/vcpkg.cmake")

project(simpleicp)


set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_FLAGS "-Wall -Wconversion -Wshadow -Wpedantic -fsanitize=undefined,address")
set(CMAKE_CXX_FLAGS_RELEASE "-O2")

# Check if using MSVC and then set appropriate preprocessor definitions for Eigen's vectorization
if(MSVC)
    add_definitions(/D_SSE2_) 
    add_compile_options(/bigobj)

else()
    add_compile_options(-march=native)  # Use -march=native for non-MSVC compilers
endif()


find_package(PCL CONFIG REQUIRED)

find_package(Eigen3 CONFIG REQUIRED)

find_package(cxxopts CONFIG REQUIRED)
find_package(nanoflann CONFIG REQUIRED)
find_package(flann CONFIG REQUIRED)

add_executable(simpleicp 6D_pose_estimation_method3_part2.cpp)
target_link_libraries(simpleicp PRIVATE Eigen3::Eigen ${PCL_LIBRARIES})