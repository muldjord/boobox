# Use the following to set wifi info:
# $ cmake -DPICO_BOARD="pico_w" -DWIFI_SSID="SSID" -DWIFI_PASSWORD="PASS" -DTCP_SERVER_IP="SERVERIP" -DTCP_SERVER_PORT="SERVERPORT" ..

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
project(pico_murray C CXX ASM)
pico_sdk_init()

add_executable(pico_murray i2s.c)

pico_generate_pio_header(pico_murray ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

target_sources(pico_murray PRIVATE pico_murray.c)

pico_set_program_name(pico_murray "pico_murray")
pico_set_program_version(pico_murray "1.0.0")

# pico_enable_stdio_usb(pico_murray 1)

target_compile_definitions(pico_murray PRIVATE
  WIFI_SSID=\"${WIFI_SSID}\"
  WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
  TCP_SERVER_IP=\"${TCP_SERVER_IP}\"
  TCP_SERVER_PORT=\"${TCP_SERVER_PORT}\"
  )

target_include_directories(pico_murray PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  )

target_link_libraries(pico_murray
  pico_stdlib
  pico_cyw43_arch_lwip_poll
  hardware_dma
  hardware_pio
  hardware_pwm
  hardware_clocks
  )

pico_add_extra_outputs(pico_murray)
