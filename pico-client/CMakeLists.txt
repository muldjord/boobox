# Use 'cmake .. -DPICO_BOARD=pico_w -DWIFI_SSID="ssid" -DWIFI_PASSWORD="password" ..'
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)

project(pico-client C CXX ASM)

set(PICO_CXX_ENABLE_EXCEPTIONS 1)

set(PICO_CXX_ENABLE_RTTI 1)

pico_sdk_init()

add_executable(pico-client pico-client.c )

pico_generate_pio_header(pico-client ${CMAKE_CURRENT_LIST_DIR}/servo.pio)

pico_set_program_name(pico-client "pico-client")
pico_set_program_version(pico-client "0.1")

pico_enable_stdio_uart(pico-client 1)
pico_enable_stdio_usb(pico-client 0)

target_compile_definitions(pico-client PRIVATE
  WIFI_SSID=\"${WIFI_SSID}\"
  WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
  TCP_SERVER_IP=\"${TCP_SERVER_IP}\"
  )

target_include_directories(pico-client PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  )

target_link_libraries(pico-client
  pico_stdlib
  pico_cyw43_arch_lwip_threadsafe_background
  hardware_dma
  hardware_pio
  hardware_timer
  hardware_clocks
  hardware_pwm
  )

pico_add_extra_outputs(pico-client)
