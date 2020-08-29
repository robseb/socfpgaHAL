/*
 * soc_clockManager.c
 *
 *  Created on: 29.08.2020
 *      Author: robin
 */
#include "SOCfpgaHAL.h"
#include "soc_clockManager.h"
#include <stdint.h>
//
//SOC_STATUS_CODE SOC_clk_is_enabled(SOC_CLK_t clk)
//{
//	SOC_STATUS_CODE status = SOC_E_BAD_ARG;
//
//	switch (clk)
//	{
//		/* For PLLs, this function checks if the PLL is bypassed or not. */
//	case SOC_CLK_MAIN_PLL:
//	case SOC_CLK_PERIPHERAL_PLL:
//	case SOC_CLK_SDRAM_PLL:
//		status = (SOC_clk_pll_is_bypassed(clk) != SOC_E_TRUE);
//		break;
//
//		/* These clocks are not gated, so must return a SOC_E_BAD_ARG type error. */
//	case SOC_CLK_MAIN_PLL_C0:
//	case SOC_CLK_MAIN_PLL_C1:
//	case SOC_CLK_MAIN_PLL_C2:
//	case SOC_CLK_MAIN_PLL_C3:
//	case SOC_CLK_MAIN_PLL_C4:
//	case SOC_CLK_MAIN_PLL_C5:
//	case SOC_CLK_MPU:
//	case SOC_CLK_MPU_L2_RAM:
//	case SOC_CLK_MPU_PERIPH:
//	case SOC_CLK_L3_MAIN:
//	case SOC_CLK_L3_SP:
//	case SOC_CLK_DBG_BASE:
//	case SOC_CLK_MAIN_QSPI:
//	case SOC_CLK_MAIN_NAND_SDMMC:
//	case SOC_CLK_PERIPHERAL_PLL_C0:
//	case SOC_CLK_PERIPHERAL_PLL_C1:
//	case SOC_CLK_PERIPHERAL_PLL_C2:
//	case SOC_CLK_PERIPHERAL_PLL_C3:
//	case SOC_CLK_PERIPHERAL_PLL_C4:
//	case SOC_CLK_PERIPHERAL_PLL_C5:
//	case SOC_CLK_SDRAM_PLL_C0:
//	case SOC_CLK_SDRAM_PLL_C1:
//	case SOC_CLK_SDRAM_PLL_C2:
//	case SOC_CLK_SDRAM_PLL_C5:
//		status = SOC_E_BAD_ARG;
//		break;
//
//		/* Clocks that originate at the Main PLL. */
//	case SOC_CLK_L4_MAIN:
//		status = (SOC_CLKMGR_MAINPLL_EN_L4MAINCLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_L3_MP:
//		status = (SOC_CLKMGR_MAINPLL_EN_L3MPCLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_L4_MP:
//		status = (SOC_CLKMGR_MAINPLL_EN_L4MPCLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_L4_SP:
//		status = (SOC_CLKMGR_MAINPLL_EN_L4SPCLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_DBG_AT:
//		status = (SOC_CLKMGR_MAINPLL_EN_DBGATCLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_DBG:
//		status = (SOC_CLKMGR_MAINPLL_EN_DBGCLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_DBG_TRACE:
//		status = (SOC_CLKMGR_MAINPLL_EN_DBGTRACECLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_DBG_TIMER:
//		status = (SOC_CLKMGR_MAINPLL_EN_DBGTMRCLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_CFG:
//		status = (SOC_CLKMGR_MAINPLL_EN_CFGCLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_H2F_USER0:
//		status = (SOC_CLKMGR_MAINPLL_EN_S2FUSER0CLK_GET(SOC_read_word(SOC_CLKMGR_MAINPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//
//		/* Clocks that originate at the Peripheral PLL. */
//	case SOC_CLK_EMAC0:
//		status = (SOC_CLKMGR_PERPLL_EN_EMAC0CLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_EMAC1:
//		status = (SOC_CLKMGR_PERPLL_EN_EMAC1CLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_USB_MP:
//		status = (SOC_CLKMGR_PERPLL_EN_USBCLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_SPI_M:
//		status = (SOC_CLKMGR_PERPLL_EN_SPIMCLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_CAN0:
//		status = (SOC_CLKMGR_PERPLL_EN_CAN0CLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_CAN1:
//		status = (SOC_CLKMGR_PERPLL_EN_CAN1CLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_GPIO_DB:
//		status = (SOC_CLKMGR_PERPLL_EN_GPIOCLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_H2F_USER1:
//		status = (SOC_CLKMGR_PERPLL_EN_S2FUSER1CLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//
//		/* Clocks that may originate at the Main PLL, the Peripheral PLL, or the FPGA. */
//	case SOC_CLK_SDMMC:
//		status = (SOC_CLKMGR_PERPLL_EN_SDMMCCLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_NAND_X:
//		status = (SOC_CLKMGR_PERPLL_EN_NANDXCLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_NAND:
//		status = (SOC_CLKMGR_PERPLL_EN_NANDCLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_QSPI:
//		status = (SOC_CLKMGR_PERPLL_EN_QSPICLK_GET(SOC_read_word(SOC_CLKMGR_PERPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//
//		/* Clocks that originate at the SDRAM PLL. */
//	case SOC_CLK_DDR_DQS:
//		status = (SOC_CLKMGR_SDRPLL_EN_DDRDQSCLK_GET(SOC_read_word(SOC_CLKMGR_SDRPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_DDR_2X_DQS:
//		status = (SOC_CLKMGR_SDRPLL_EN_DDR2XDQSCLK_GET(SOC_read_word(SOC_CLKMGR_SDRPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_DDR_DQ:
//		status = (SOC_CLKMGR_SDRPLL_EN_DDRDQCLK_GET(SOC_read_word(SOC_CLKMGR_SDRPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//	case SOC_CLK_H2F_USER2:
//		status = (SOC_CLKMGR_SDRPLL_EN_S2FUSER2CLK_GET(SOC_read_word(SOC_CLKMGR_SDRPLL_EN_ADDR)))
//			? SOC_E_TRUE : SOC_E_FALSE;
//		break;
//
//	default:
//		status = SOC_E_BAD_ARG;
//		break;
//
//	}
//
//	return status;
//}

