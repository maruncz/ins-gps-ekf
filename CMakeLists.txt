cmake_minimum_required(VERSION 3.25)
project(EKF-INS)

set(CMAKE_CXX_STANDARD 17)

find_package(Boost CONFIG COMPONENTS system)
find_package (Eigen3 3.3 REQUIRED)
find_package(spdlog CONFIG REQUIRED)

file(GLOB EKF_INCLUDE
        "include/*.h")

file(GLOB EKF_SRC
        "src/*.cpp")

add_library(EKF-INS STATIC ${EKF_SRC} ${EKF_INCLUDE})
target_include_directories(EKF-INS PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(EKF-INS ${Boost_LIBRARIES} spdlog::spdlog Eigen3::Eigen)