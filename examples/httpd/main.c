/*
 * Copyright (c) 2024 Rob Scott, portions copyrighted as below:
 *
 * Copyright (c) 2021 Sandeep Mistry
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/regs/clocks.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

#include "hardware/clocks.h"
#include "hardware/pll.h"

// For setting 1.8v threshold
#include "hardware/vreg.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/pads_bank0.h"

#include "lwip/dhcp.h"
#include "lwip/init.h"

#include "lwip/apps/httpd.h"
#include "rmii_ethernet/netif.h"
#include "lan8720a.h"


void netif_link_callback(struct netif *netif) {
  printf("netif link status changed %s\n",
         netif_is_link_up(netif) ? "up" : "down");

  // Do a soft reset if link goes down
  if (netif_is_link_up(netif) == 0) {
    // Send reset command
    netif_rmii_ethernet_mdio_write(phy_address, LAN8720A_BASIC_CONTROL_REG,
				   0x8000);
  }
}

void netif_status_callback(struct netif *netif) {
  printf("netif status changed %s\n", ip4addr_ntoa(netif_ip4_addr(netif)));
}

int main() {
  // LWIP network interface
  struct netif netif;

#ifdef ORIG_ASSIGNMENT
  struct netif_rmii_ethernet_config netif_config = {
      pio0, // PIO:            0
      0,    // pio SM:         0 and 1
      6,    // rx pin start:   6, 7, 8    => RX0, RX1, CRS
      10,   // tx pin start:   10, 11, 12 => TX0, TX1, TX-EN
      14,   // mdio pin start: 14, 15   => ?MDIO, MDC
      21,   // rmii clock:     21, 23, 24 or 25 => RETCLK
      NULL, // MAC address (optional - NULL generates one based on flash id)
  };
#endif

#ifdef DECSTATION2040 
  struct netif_rmii_ethernet_config netif_config = {
      pio0, // PIO:            0
      0,    // pio SM:         0 and 1
      3,    // rx pin start:   3, 4, 5  => RX0, RX1, CRS
      0,    // tx pin start:   0, 1, 2  => TX0, TX1, TX-EN
      6,    // mdio pin start: 6, 7   => ?MDIO, MDC
      21,   // rmii clock:     21, 23, 24 or 25 => RETCLK
      NULL, // MAC address (optional - NULL generates one based on flash id)
  };
#endif
  uint32_t target_clk = 300000000;
    
  // Setup overclock, which needs a bit of a voltage boost
  vreg_set_voltage(VREG_VOLTAGE_1_20);
  sleep_ms(1);
  set_sys_clock_khz(target_clk/1000, true);

#ifdef EN_1V8 
  // Set 1.8v threshold for I/O pads
  io_rw_32* addr = (io_rw_32 *)(PADS_BANK0_BASE + PADS_BANK0_VOLTAGE_SELECT_OFFSET);
  *addr = PADS_BANK0_VOLTAGE_SELECT_VALUE_1V8 << PADS_BANK0_VOLTAGE_SELECT_LSB;
#endif


  // Configure clock output on retclk pin to be 50 MHz
  float clk_50_div = clock_get_hz(clk_sys)/50000000;
  clock_gpio_init(netif_config.retclk_pin, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLK_SYS, clk_50_div);

  // Generate clk index from GPIO pin number
  uint gpclk = 0;
  uint gpio = netif_config.retclk_pin;
  if      (gpio == 21) gpclk = clk_gpout0;
  else if (gpio == 23) gpclk = clk_gpout1;
  else if (gpio == 24) gpclk = clk_gpout2;
  else if (gpio == 25) gpclk = clk_gpout3;
 
  // Enable 50% duty cycle for odd divisors
  clocks_hw->clk[gpclk].ctrl |= CLOCKS_CLK_GPOUT0_CTRL_DC50_BITS;

  // Initialize stdio after the clock change
  sleep_ms(1);
  stdio_init_all();
  sleep_ms(3000);

  printf("pico rmii ethernet - httpd\n");

  // Initilize LWIP in NO_SYS mode
  lwip_init();

  // Initialize the PIO-based RMII Ethernet network interface
  netif_rmii_ethernet_init(&netif, &netif_config);

  // Assign callbacks for link and status
  netif_set_link_callback(&netif, netif_link_callback);
  netif_set_status_callback(&netif, netif_status_callback);

  // Set the default interface and bring it up
  netif_set_default(&netif);
  netif_set_up(&netif);

  // Start DHCP client and httpd
  dhcp_start(&netif);
  httpd_init();

  // Setup core 1 to monitor the RMII ethernet interface
  // This allows core 0 do other things :)
  multicore_launch_core1(netif_rmii_ethernet_loop);

  while (1) {
    tight_loop_contents();
  }

  return 0;
}
