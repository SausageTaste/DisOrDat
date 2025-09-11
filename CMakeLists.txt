cmake_minimum_required(VERSION 3.11)

set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

project(DisOrDat)


add_subdirectory(extern/sungtools)

find_package(asio CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)


add_library(DisOrDat STATIC
    ${CMAKE_SOURCE_DIR}/src/pdu_struct.cpp
)
add_library(DisOrDat::DisOrDat ALIAS DisOrDat)
target_include_directories(DisOrDat PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)
target_link_libraries(DisOrDat PUBLIC
    glm::glm
    spdlog::spdlog
    sungtools::sungtools_basic
)


add_subdirectory(app)
