cmake_minimum_required(VERSION 3.11)

set(CMAKE_TOOLCHAIN_FILE "$ENV{VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake")

project(DisOrDat)


find_package(asio CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)


add_subdirectory(extern/SungToolsCpp)
add_subdirectory(app)
