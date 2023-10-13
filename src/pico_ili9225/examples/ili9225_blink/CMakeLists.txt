cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(ili9225_blink
    main.c
)

target_link_libraries(ili9225_blink PRIVATE pico_ili9225)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(ili9225_blink)
