cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(grid_spll
    main.c
)

target_include_directories(grid_spll PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(grid_spll PRIVATE pico_stdlib hardware_dma hardware_adc hardware_gpio pico_iqmath)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(grid_spll)
