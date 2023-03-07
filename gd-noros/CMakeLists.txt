cmake_minimum_required(VERSION 3.0.0)
project(gd-noros VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-fopenmp")

include(CTest)
enable_testing()

find_package(OpenCV)
find_package(Eigen3)
find_package(PCL)
find_package(Threads)

include_directories(${OpenCV_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

file(GLOB_RECURSE ck_src "chunkmap/*.cpp")
file(GLOB_RECURSE gd_src "gd/*.cpp")
add_executable(gd-noros main.cpp ${ck_src} ${gd_src})

target_link_libraries(gd-noros ${OpenCV_LIBRARIES} ${PCL_LIBRARIES} Threads::Threads)

set(CPACK_PROJECT_NAME ${PROJECT_NAME})
set(CPACK_PROJECT_VERSION ${PROJECT_VERSION})
include(CPack)
