/*
 * soc_gpio.c
 *
 *  Created on: 27.08.2020
 *      Author: robin
 */
#include "socfpgaHAL/inc/socfpgaHAL.h"
#include "socfpgaHAL/inc/soc_gpio.h"
#include "socfpgaHAL/inc/soc_regacc.h"

soc_staus_t soc_gpio_init(void)
{
	if (!soc_read_word(SOCRO_GPIO_VER_ID_CODE) == SOCC_GPIO_VER_ID_CODE)
		return SOC_S_FAIELD_TO_ACCESS;

	/*
	soc_replbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_GPIO0_SET_MSK |
					  ALT_RSTMGR_PERMODRST_GPIO1_SET_MSK |
					  ALT_RSTMGR_PERMODRST_GPIO2_SET_MSK, 0);
	*/
	return SOC_S_SUCCESSFULLY;
}
soc_staus_t soc_gpio_deinit(void)
{
	/*
	alt_replbits_word(ALT_RSTMGR_PERMODRST_ADDR, ALT_RSTMGR_PERMODRST_GPIO0_SET_MSK |
	                      ALT_RSTMGR_PERMODRST_GPIO1_SET_MSK |
	                      ALT_RSTMGR_PERMODRST_GPIO2_SET_MSK,
	                      ALT_GPIO_BITMASK);
	*/
	return SOC_S_SUCCESSFULLY;
}



soc_staus_t soc_gpio_configPort(soc_gpio_config_t pinConf)
{
	return SOC_S_SUCCESSFULLY;
}

soc_staus_t soc_gpio_ISRconfig(soc_gpio_isrconfig_t isrConf)
{
	return SOC_S_SUCCESSFULLY;
}

soc_staus_t soc_gpio_ISRenable(soc_gpio_port_t port, uint32_t pin_mask)
{
	return SOC_S_SUCCESSFULLY;
}

soc_staus_t soc_gpio_ISRclear(soc_gpio_port_t port, uint32_t pin_mask)
{
	return SOC_S_SUCCESSFULLY;
}

soc_staus_t soc_gpio_write(soc_gpio_port_t port, uint32_t pin_mask)
{
	return SOC_S_SUCCESSFULLY;
}

uint32_t    soc_gpio_read (soc_gpio_port_t port, uint32_t pin_mask)
{
	return 0;
}
