include_directories(${PCL_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main main.cpp)
target_link_libraries(main ${PCL_LIBRARIES})

