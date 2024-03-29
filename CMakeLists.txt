cmake_minimum_required(VERSION 3.13)
include("C:/SRC/pico-sdk/pico_sdk_init.cmake")
pico_sdk_init()

project(book_play)

add_executable(book_play
        book_play.c
        )

pico_enable_stdio_usb(book_play 0)
pico_enable_stdio_uart(book_play 1)

target_include_directories(book_play PUBLIC ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(
book_play
pico_stdlib
tinyusb_host
tinyusb_board
hardware_pio
hardware_dma
hardware_timer
hardware_clocks
hardware_pwm
pico_rand
)

pico_add_extra_outputs(book_play)

