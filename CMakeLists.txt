cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(pico_grid_spll)

pico_sdk_init()

add_library(pico_iqmath INTERFACE)

target_sources(pico_iqmath INTERFACE
     ${CMAKE_CURRENT_LIST_DIR}/iqmath/_IQNfunctions/_IQNtoF.c
     ${CMAKE_CURRENT_LIST_DIR}/iqmath/_IQNfunctions/_IQNmpy.c
     ${CMAKE_CURRENT_LIST_DIR}/iqmath/_IQNfunctions/_IQNsin_cos.c
)

target_include_directories(pico_iqmath INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/iqmath/include
    ${CMAKE_CURRENT_LIST_DIR}/dummy
)

#target_link_libraries(pico_iqmath INTERFACE pico_stdlib)

add_subdirectory(grid_spll)
