add_executable(testnlsoft arduino_test_tilting)

target_compile_features(testnlsoft PRIVATE cxx_std_17)

target_link_libraries(testnlsoft PRIVATE nlsoft)

add_test(NAME testnlsofttest COMMAND testnlsoft)
