cmake_minimum_required(VERSION 3.13)

# Use 'cmake .. -DPICO_BOARD="pico_w" -DWIFI_SSID="YOUR_WIFI_SSID" -DWIFI_PASSWORD="YOUR_WIFI_PASSWORD" -DTCP_SERVER_IP="SERVERIP" -DTCP_SERVER_PORT="SERVERPORT" ..'

include(pico_sdk_import.cmake)

project(client_v2 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

#set(PICO_CXX_ENABLE_EXCEPTIONS 1)
#set(PICO_CXX_ENABLE_RTTI 1)

pico_sdk_init()

add_executable(client_v2 client_v2.c)

pico_set_program_name(client_v2 "client_v2")
pico_set_program_version(client_v2 "0.1")

# pico_enable_stdio_uart(client_v2 1)
pico_enable_stdio_usb(client_v2 1)

target_compile_definitions(client_v2 PRIVATE
  WIFI_SSID=\"${WIFI_SSID}\"
  WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
  TCP_SERVER_IP=\"${TCP_SERVER_IP}\"
  TCP_SERVER_PORT=\"${TCP_SERVER_PORT}\"
  )

target_include_directories(client_v2 PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}
  )

target_link_libraries(client_v2
  pico_cyw43_arch_lwip_poll
  pico_stdlib
  )

target_compile_definitions(client_v2 PRIVATE
  )

pico_add_extra_outputs(client_v2)
