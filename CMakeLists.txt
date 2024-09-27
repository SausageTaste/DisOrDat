cmake_minimum_required(VERSION 3.11)

project(DisOrDat)

include(FetchContent)


FetchContent_Declare(
    fetch_spdlog
    GIT_REPOSITORY  https://github.com/gabime/spdlog
    GIT_TAG         v1.14.1
)

FetchContent_Declare(
    fetch_asio
    GIT_REPOSITORY  https://github.com/chriskohlhoff/asio
    GIT_TAG         efdc25ab99786101351a5afb39f01dfaf0781401
)


FetchContent_MakeAvailable(
    fetch_spdlog
    fetch_asio
)


add_subdirectory(extern/SungToolsCpp)

add_subdirectory(app)
