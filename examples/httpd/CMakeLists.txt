cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(pico_rmii_ethernet_httpd
    main.c
)

target_link_libraries(pico_rmii_ethernet_httpd pico_stdlib pico_multicore pico_rmii_ethernet)

# Select console output ports
pico_enable_stdio_usb(pico_rmii_ethernet_httpd 1)
pico_enable_stdio_uart(pico_rmii_ethernet_httpd 1)

# Use J7 as serial port 
target_compile_definitions(pico_rmii_ethernet_httpd PRIVATE
  PICO_DEFAULT_UART=0
  PICO_DEFAULT_UART_TX_PIN=16
  PICO_DEFAULT_UART_RX_PIN=17
)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_rmii_ethernet_httpd)

# Enable SRAM only executable
#pico_set_binary_type(pico_rmii_ethernet_httpd no_flash)
#pico_set_binary_type(pico_rmii_ethernet_httpd copy_to_ram)

