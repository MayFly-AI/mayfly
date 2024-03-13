add_executable(dct_example dct_example.cpp)
target_link_libraries(dct_example PRIVATE shared)

add_executable(crc_example crc_example.cpp)
target_link_libraries(crc_example PRIVATE shared)
