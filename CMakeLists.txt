
cmake_minimum_required(VERSION 3.13)

add_compile_definitions(PICO_XOSC_STARTUP_DELAY_MULTIPLIER=128)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(my_project LANGUAGES C CXX)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()
#add_compile_options(-Werror -Wall -Wextra -Wnull-dereference)
#add_compile_options(-Wall)
add_executable(oXs)

#pico_generate_pio_header(oXs ${CMAKE_CURRENT_LIST_DIR}/src/uart_crsf_rx.pio )
pico_generate_pio_header(oXs ${CMAKE_CURRENT_LIST_DIR}/src/uart_crsf_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_gps_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_gps_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_hott_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_ibus_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_jeti_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_mpx_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_sbus_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_sport_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_srxl2_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_exbus_tx_rx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_logger_tx.pio )
pico_generate_pio_header(oXs  ${CMAKE_CURRENT_LIST_DIR}/src/uart_esc_rx.pio )


file(GLOB oXs_SRC CONFIGURE_DEPENDS "src/*.h" "src/*.cpp")

target_sources(oXs PRIVATE ${oXs_SRC}  )

target_link_libraries(oXs PRIVATE
        pico_stdlib
        pico_multicore
        hardware_pio
        hardware_i2c
        hardware_watchdog
        hardware_timer
        hardware_dma
        hardware_irq
        hardware_adc
        hardware_pwm
        hardware_spi
        )

#enable printf on usb
pico_enable_stdio_usb(oXs 1)
pico_enable_stdio_uart(oXs 0)


# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(oXs)


add_custom_command(TARGET oXs POST_BUILD
  COMMAND ${CMAKE_COMMAND}
  ARGS -E copy oXs.uf2 ${CMAKE_SOURCE_DIR}   
)
#  ARGS -E copy oXs.uf2 ${CMAKE_SOURCE_DIR} it was with -E   
# copy the uf2 file to E: drive
add_custom_command(TARGET oXs POST_BUILD COMMAND ${CMAKE_COMMAND} ARGS -E copy oXs.uf2 E: )
