# Pico RMII Ethernet library, Neon Chrome Edition (NCE)

## Introduction

This is an update to the existing pico-rmii-ethernet library by Sandeep Mistry.
Please see README_orig.md for a description of that library. Note that the
RMII board modifications to use an external 50 MHz clock are still necessary.

## Improvements present in this library:
1. Achieves 94.9 Mbit/sec when Pico is overclocked to 300 MHz, as measured
by iperf.
2. If the DMA CRC engine is used, 94.9 Mbit/sec rate is achived wtih 150 MHz system clock.
3. Inter packet gaps correctly inserted into transmit stream.
4. Greater flexiblity in choosing system clocks: 150, 200, 250, 300.
Higher system clock frequencies should work; untested.
5. iperf available in the examples directory

## Discussion

This library uses DMA driven ring buffers for both transmit and receive. The
transmit side is entirely DMA driven, while the receive side uses a per-packet
interrupt to finalize a received packet. Additionally, the DMA sniffer subsystem
may be enabled to off-load CRC calculations from the CPU. Performance does vary
with system clock speed, and the memory region the executable is placed:

| Sys Clk | Iperf CPU/SRAM | Iperf CPU/flash | Iperf DMA/SRAM | Iperf DMA/flash |
| :---:   |     :---:      |     :---:       |   :---:        |     :---:       |
| MHz     | Mbit/sec       | Mbit/sec        |  Mbit/sec      | Mbit/sec        |
| 150     | 11.6           |  2.9            | 94.9           | 70.1            |
| 200     | 65.4           | 31.4            | 94.9           | 94.9            |
| 250     | 83.4           | 69.3            | 94.9           | 94.9            |
| 300     | 94.9           | 85.9            | 94.9           | 94.9            |

The MDIO interface was changed from polled to interrupt driven, thus freeing
up more processor time for packet processing.

The RMII Rx/Tx PIO subsystems are clocked at 100 MHz via dividing down the system
clock. This should allow the same RMII performance for either CPU or DMA
driven CRC configurations, so it is unclear why there is such a large difference
between the two scenarios at 150 MHz.

## Resource Utilization

The library uses:

1. Five DMA channels: 2 receive, 3 transmit. Two channels are used per Tx/Rx for
ring buffer management, and the third Tx channel assists in generating the
Ethernet Inter Packet Gap.
2. Optionally, the DMA "sniffer" logic may be enabled. There is only one sniffer.
2. Two interrupts: 1 shared for MDIO, and 1 exclusive for the end-of-packet
processing.
3. Two 4KB aligned memory regions for Tx/Rx data, 32/64 long word pointer
buffers, and a 256 long word CRC table (if CPU CRC is enabled). 
4. One PWM timer used as MD clock
5. One DMA timer used to assist with IPG generation
6. 12 PIO instructions for Tx, 6 for Rx, total 18

At 300 MHz, almost all of core 1 is used when CPU CRC generation is used.
It is possible to use about 6 usec per packet poll, verified by placing a
sleep_us(6) call in netif_rmii_ethernet_loop() and running iperf. With DMA CRC
generation, a sleep_us(10) may be used with minimal impact. If a 90 Mb/s
iperf rate is acceptable, then sleep_us interval may be increased to 30.
Core 0, of course, remains available for user applications.

## Customization

Edit main.c in examples/{http, lwiperf} to set the pins used for the RMII
interface, and set the target_clk variable to the desired system clock, one
of: 150/200/250/300 MHz. Higher frequencies of 400 or 450 may work, but
are untested. These frequencies ensure that a symmetric 50 MHz RMII is generated.

DMA CRC calculation is enabled in src/rmii_ethernet.c via #define USE_DMA_CRC.
Commenting out this define will enable CPU driven CRC calculations.

## Limitations

** For best performance, must be run directly from RP2040 SRAM **, using:
```
        pico_set_binary_type(pico_rmii_ethernet_lwiperf no_flash)
```
and loading via a debugger or
```
        pico_set_binary_type(pico_rmii_ethernet_lwiperf copy_to_ram)
```
in the CMakeLists.

## Acknowledgements

Sandeep Mistry - who determined that using an external clock for the RMII
board would enable much better performance.

## History
1. Original version
2. Added DMA driven CRC, removed 100 MHz system clock configuration.




