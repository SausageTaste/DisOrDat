add_executable(DisDatSim main.cpp)
target_include_directories(DisDatSim PUBLIC
    ${fetch_asio_SOURCE_DIR}/asio/include
)
target_link_libraries(DisDatSim
    DisOrDat::DisOrDat
)
target_compile_features(DisDatSim PUBLIC cxx_std_23)
