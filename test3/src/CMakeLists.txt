include_directories(/usr/include/hello)

set(main main.cpp)
add_executable(hello ${main})

target_link_libraries(hello libhello.so)