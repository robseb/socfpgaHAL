/*
 * soc_clockManager.h
 *
 *  Created on: 29.08.2020
 *      Author: robin
 */

#ifndef SOCFPGAHAL_INC_SOC_CLOCKMANAGER_H_
#define SOCFPGAHAL_INC_SOC_CLOCKMANAGER_H_

#include "SOCfpgaHAL.h"
#include <stdint.h>

typedef uint32_t soc_freq_t;

typedef enum SOC_CLK_e
{
	/* Clock Input Pins */
	SOC_CLK_IN_PIN_OSC1,
										/*!< \b OSC_CLK_1_HPS
										*   External oscillator input:
										*   * Input Pin
										*   * Clock source to Main PLL
										*   * Clock source to SDRAM PLL
										*     and Peripheral PLL if selected via
										*     register write
										*   * Clock source for clock in safe mode
										*/

	SOC_CLK_IN_PIN_OSC2,
										/*!< \b OSC_CLK_2_HPS
										*   External Oscillator input:
										*   * Input Pin
										*   * Optional clock source to SDRAM PLL
										*     and Peripheral PLL if selected
										*   * Typically used for Ethernet
										*     reference clock
										*/


	/* FPGA Clock Sources External to HPS */
	SOC_CLK_F2H_PERIPH_REF,
										/*<! SOCernate clock source from FPGA
										* for HPS Peripheral PLL. */

	SOC_CLK_F2H_SDRAM_REF,
										/*<! SOCernate clock source from FPGA
										* for HPS SDRAM PLL. */


	/* Other Clock Sources External to HPS */
	SOC_CLK_IN_PIN_JTAG,
										/*!< \b JTAG_TCK_HPS
										*   * Input Pin
										*   * External HPS JTAG clock input.
										*/

	SOC_CLK_IN_PIN_ULPI0,
										/*!< \b ULPI0_CLK
										*   ULPI Clock provided by external USB0
										*   PHY
										*   * Input Pin
										*/

	SOC_CLK_IN_PIN_ULPI1,
										/*!< \b ULPI1_CLK
										*   ULPI Clock provided by external USB1
										*   PHY
										*   * Input Pin
										*/

	SOC_CLK_IN_PIN_EMAC0_RX,
										/*!< \b EMAC0:RX_CLK
										*   Rx Reference Clock for EMAC0
										*   * Input Pin
										*/

	SOC_CLK_IN_PIN_EMAC1_RX,
										/*!< \b EMAC1:RX_CLK
										*   Rx Reference Clock for EMAC1
										*   * Input Pin
										*/


	/* PLLs */
	SOC_CLK_MAIN_PLL,
										/*!< \b main_pll_ref_clkin
										*   Main PLL input reference clock,
										*   used to designate the Main PLL in
										*   PLL clock selections.
										*/

	SOC_CLK_PERIPHERAL_PLL,
										/*!< \b periph_pll_ref_clkin
										*   Peripheral PLL input reference
										*   clock, used to designate the
										*   Peripheral PLL in PLL clock
										*   selections.
										*/

	SOC_CLK_SDRAM_PLL,
										/*!< \b sdram_pll_ref_clkin
										*   SDRAM PLL input reference clock,
										*   used to designate the SDRAM PLL in
										*   PLL clock selections.
										*/

	/* OSC1 Clock Group - The OSC1 clock group contains those clocks which are derived
	* directly from the osc_clk_1_HPS pin */
	SOC_CLK_OSC1,
										/*!< \b osc1_clk
										*   OSC1 Clock Group - The
										*   OSC1 clock group contains
										*   those clocks which are
										*   derived directly from the
										*   osc_clk_1_HPS pin.
										*   * alias for SOC_CLK_IN_PIN_OSC1
										*/

	/* Main Clock Group - The following clocks are derived from the Main PLL. */
	SOC_CLK_MAIN_PLL_C0,
										/*!< \b Main PLL C0 Output */

	SOC_CLK_MAIN_PLL_C1,
										/*!< \b Main PLL C1 Output */

	SOC_CLK_MAIN_PLL_C2,
										/*!< \b Main PLL C2 Output */

	SOC_CLK_MAIN_PLL_C3,
										/*!< \b Main PLL C3 Output */

	SOC_CLK_MAIN_PLL_C4,
										/*!< \b Main PLL C4 Output */

	SOC_CLK_MAIN_PLL_C5,
										/*!< \b Main PLL C5 Output */

	SOC_CLK_MPU,
										/*!< \b mpu_clk
										*   Main PLL C0 Output. Clock for MPU
										*   subsystem, including CPU0 and CPU1.
										*   * Alias for \e SOC_CLK_MAIN_PLL_C0
										*/

	SOC_CLK_MPU_L2_RAM,
										/*!< \b mpu_l2_ram_clk
										*   Clock for MPU level 2 (L2) RAM
										*/

	SOC_CLK_MPU_PERIPH,
										/*!< \b mpu_periph_clk
										*   Clock for MPU snoop control unit
										*   (SCU) peripherals, such as the
										*   general interrupt controller (GIC)
										*/

	SOC_CLK_L3_MAIN,
										/*!< \b main_clk
										*   Main PLL C1 Output
										*   * Alias for \e SOC_CLK_MAIN_PLL_C1
										*/

	SOC_CLK_L3_MP,
										/*!< \b l3_mp_clk
										*   Clock for L3 Master Peripheral Switch
										*/

	SOC_CLK_L3_SP,
										/*!< \b l3_sp_clk
										*   Clock for L3 Slave Peripheral Switch
										*/

	SOC_CLK_L4_MAIN,
										/*!< \b l4_main_clk
										*   Clock for L4 main bus
										*   * Clock for DMA
										*   * Clock for SPI masters
										*/

	SOC_CLK_L4_MP,
										/*!< \b l4_mp_clk
										*   Clock for L4 master peripherals (MP) bus
										*/

	SOC_CLK_L4_SP,
										/*!< \b l4_sp_clk
										*   Clock for L4 slave peripherals (SP) bus
										*/

	SOC_CLK_DBG_BASE,
										/*!< \b dbg_base_clk
										*   Main PLL C2 Output
										*   * Alias for \e SOC_CLK_MAIN_PLL_C2
										*/

	SOC_CLK_DBG_AT,
										/*!< \b dbg_at_clk
										*   Clock for CoreSight debug Advanced
										*   Microcontroller Bus Architecture
										*   (AMBA) Trace Bus (ATB)
										*/

	SOC_CLK_DBG_TRACE,
										/*!< \b dbg_trace_clk
										*   Clock for CoreSight debug Trace
										*   Port Interface Unit (TPIU)
										*/

	SOC_CLK_DBG_TIMER,
										/*!< \b dbg_timer_clk
										*   Clock for the trace timestamp
										*   generator
										*/

	SOC_CLK_DBG,
										/*!< \b dbg_clk
										*   Clock for Debug Access Port (DAP)
										*   and debug Advanced Peripheral Bus
										*   (APB)
										*/

	SOC_CLK_MAIN_QSPI,
										/*!< \b main_qspi_clk
										*   Main PLL C3 Output. Quad SPI flash
										*   internal logic clock.
										*   * Alias for \e SOC_CLK_MAIN_PLL_C3
										*/

	SOC_CLK_MAIN_NAND_SDMMC,
										/*!< \b main_nand_sdmmc_clk
										*   Main PLL C4 Output. Input clock to
										*   flash controller clocks block.
										*   * Alias for \e SOC_CLK_MAIN_PLL_C4
										*/

	SOC_CLK_CFG,
										/*!< \b cfg_clk
										*   FPGA manager configuration clock.
										*/

	SOC_CLK_H2F_USER0,
										/*!< \b h2f_user0_clock
										*   Clock to FPGA fabric
										*/


	/* Peripherals Clock Group - The following clocks are derived from the Peripheral PLL */
	SOC_CLK_PERIPHERAL_PLL_C0,
										/*!< \b Peripheral PLL C0 Output */

	SOC_CLK_PERIPHERAL_PLL_C1,
										/*!< \b Peripheral PLL C1 Output */

	SOC_CLK_PERIPHERAL_PLL_C2,
										/*!< \b Peripheral PLL C2 Output */

	SOC_CLK_PERIPHERAL_PLL_C3,
										/*!< \b Peripheral PLL C3 Output */

	SOC_CLK_PERIPHERAL_PLL_C4,
										/*!< \b Peripheral PLL C4 Output */

	SOC_CLK_PERIPHERAL_PLL_C5,
										/*!< \b Peripheral PLL C5 Output */

	SOC_CLK_USB_MP,
										/*!< \b usb_mp_clk
										*   Clock for USB
										*/

	SOC_CLK_SPI_M,
										/*!< \b spi_m_clk
										*   Clock for L4 SPI master bus
										*/

	SOC_CLK_QSPI,
										/*!< \b qspi_clk
										*   Clock for Quad SPI
										*/

	SOC_CLK_NAND_X,
										/*!< \b nand_x_clk
										*   NAND flash controller master and
										*   slave clock
										*/

	SOC_CLK_NAND,
										/*!< \b nand_clk
										*   Main clock for NAND flash
										*   controller
										*/

	SOC_CLK_SDMMC,
										/*!< \b sdmmc_clk
										*   Clock for SD/MMC logic input clock
										*/

	SOC_CLK_EMAC0,
										/*!< \b emac0_clk
										*   EMAC 0 clock - Peripheral PLL C0
										*   Output
										*   * Alias for \e SOC_CLK_PERIPHERAL_PLL_C0
										*/

	SOC_CLK_EMAC1,
										/*!< \b emac1_clk
										*   EMAC 1 clock - Peripheral PLL C1
										*   Output
										*   * Alias for \e SOC_CLK_PERIPHERAL_PLL_C1
										*/

	SOC_CLK_CAN0,
										/*!< \b can0_clk
										*   Controller area network (CAN)
										*   controller 0 clock
										*/

	SOC_CLK_CAN1,
										/*!< \b can1_clk
										*   Controller area network (CAN)
										*   controller 1 clock
										*/

	SOC_CLK_GPIO_DB,
										/*!< \b gpio_db_clk
										*   Debounce clock for GPIO0, GPIO1,
										*   and GPIO2
										*/

	SOC_CLK_H2F_USER1,
										/*!< \b h2f_user1_clock
										*   Clock to FPGA fabric - Peripheral
										*   PLL C5 Output
										*   * Alias for \e SOC_CLK_PERIPHERAL_PLL_C5
										*/


	/* SDRAM Clock Group - The following clocks are derived from the SDRAM PLL */
	SOC_CLK_SDRAM_PLL_C0,
										/*!< \b SDRAM PLL C0 Output */

	SOC_CLK_SDRAM_PLL_C1,
										/*!< \b SDRAM PLL C1 Output */

	SOC_CLK_SDRAM_PLL_C2,
										/*!< \b SDRAM PLL C2 Output */

	SOC_CLK_SDRAM_PLL_C3,
										/*!< \b SDRAM PLL C3 Output */

	SOC_CLK_SDRAM_PLL_C4,
										/*!< \b SDRAM PLL C4 Output */

	SOC_CLK_SDRAM_PLL_C5,
										/*!< \b SDRAM PLL C5 Output */

	SOC_CLK_DDR_DQS,
										/*!< \b ddr_dqs_clk
										*   Clock for MPFE, single-port
										*   controller, CSR access, and PHY -
										*   SDRAM PLL C0 Output
										*   * Alias for \e SOC_CLK_SDRAM_PLL_C0
										*/

	SOC_CLK_DDR_2X_DQS,
										/*!< \b ddr_2x_dqs_clk
										*    Clock for PHY - SDRAM PLL C1 Output
										*   * Alias for \e SOC_CLK_SDRAM_PLL_C1
										*/

	SOC_CLK_DDR_DQ,
										/*!< \b ddr_dq_clk
										*   Clock for PHY - SDRAM PLL C2 Output
										*   * Alias for \e SOC_CLK_SDRAM_PLL_C2
										*/

	SOC_CLK_H2F_USER2,
										/*!< \b h2f_user2_clock
										*   Clock to FPGA fabric - SDRAM PLL C5
										*   Output
										*   * Alias for \e SOC_CLK_SDRAM_PLL_C5
										*/

	/* Clock Output Pins */
	SOC_CLK_OUT_PIN_EMAC0_TX,
									/*!< \b EMAC0:TX_CLK
										*   Tx Reference Clock for EMAC0
										*   * Output Pin
										*/

	SOC_CLK_OUT_PIN_EMAC1_TX,
									/*!< \b EMAC1:TX_CLK
										*   Tx Reference Clock for EMAC1
										*   * Output Pin
										*/

	SOC_CLK_OUT_PIN_SDMMC,
									/*!< \b SDMMC:CLK
										*   SD/MMC Card Clock
										*   * Output Pin
										*/

	SOC_CLK_OUT_PIN_I2C0_SCL,
									/*!< \b I2C0:SCL
										*   I2C Clock for I2C0
										*   * Output Pin
										*/

	SOC_CLK_OUT_PIN_I2C1_SCL,
									/*!< \b I2C1:SCL
										*   I2C Clock for I2C1
										*   * Output Pin
										*/

	SOC_CLK_OUT_PIN_I2C2_SCL,
									/*!< \b I2C2:SCL
										*   I2C Clock for I2C2/2 wire
										*   * Output Pin
										*/

	SOC_CLK_OUT_PIN_I2C3_SCL,
									/*!< \b I2C3:SCL
										*   I2C Clock for I2C1/2 wire
										*   * Output Pin
										*/

	SOC_CLK_OUT_PIN_SPIM0,
									/*!< \b SPIM0:CLK
										*   SPI Clock
										*   * Output Pin
										*/

	SOC_CLK_OUT_PIN_SPIM1,
									/*!< \b SPIM1:CLK
										*   SPI Clock
										*   * Output Pin
										*/

	SOC_CLK_OUT_PIN_QSPI,
									/*!< \b QSPI:CLK
										*   QSPI Flash Clock
										*   * Output Pin
										*/

	SOC_CLK_UNKNOWN
} SOC_CLK_t;

/*!
* Return whether the specified clock is enabled or not.
*
* \param       clk
*              The clock to check whether enabled or not.
*
* \retval      ALT_E_TRUE      The clock is enabled.
* \retval      ALT_E_FALSE     The clock is not enabled.
* \retval      ALT_E_BAD_ARG   The \e clk argument designates a non gated clock
*                              value.
*/
/*

ALT_STATUS_CODE alt_clk_is_enabled(ALT_CLK_t clk);
*/
#endif /* SOCFPGAHAL_INC_SOC_CLOCKMANAGER_H_ */
