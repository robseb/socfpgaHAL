/*
 * soc_gpio.c
 *
 *  Created on: 27.08.2020
 *      Author: robin
 */
#include <soc_resetManager.h>
#include "socfpgaHAL.h"
#include "soc_gpio.h"
#include "soc_regacc.h"



#define     SOC_GPIO_BITMASK    0x1FFFFFFF

#define SOC__GPIO_PORT_CHECK(_port_, _mask_){ 					\
		if((_mask_ >= 0x200) && (_port_ ==SOC_GPIO_PORTC)) 		\
		return SOC_S_WRONG_INPUT; 								\
		if(((_mask_ & (1<<29)) || 								\
				(_mask_ & (1<<30)) ||							\
				(_mask_ & (1<<31))) &&							\
				((_port_ == SOC_GPIO_PORTA) || 					\
						(_port_ == SOC_GPIO_PORTB))) 			\
					return SOC_S_WRONG_INPUT;}

soc_status_t soc_gpio_init(void)
{
	if (!soc_read_word(SOCRO_GPIO_VER_ID_CODE) == SOCC_GPIO_VER_ID_CODE)
		return SOC_S_FAIELD_TO_ACCESS;

	soc_gpio_deinit();

	soc_replbits_word(SOCR_RESTMAN_BASE+SOC_RSTMGR_PERMODRST_OFST, SOC_RSTMGR_PERMODRST_GPIO0_SET_MSK |
					  SOC_RSTMGR_PERMODRST_GPIO1_SET_MSK |
					  SOC_RSTMGR_PERMODRST_GPIO2_SET_MSK, 0);

	return SOC_S_SUCCESSFULLY;
}


soc_status_t soc_gpio_deinit(void)
{
	soc_replbits_word(SOCR_RESTMAN_BASE+SOC_RSTMGR_PERMODRST_OFST, SOC_RSTMGR_PERMODRST_GPIO0_SET_MSK |
	                      SOC_RSTMGR_PERMODRST_GPIO1_SET_MSK |
	                      SOC_RSTMGR_PERMODRST_GPIO2_SET_MSK,
	                      SOC_GPIO_BITMASK);

	return SOC_S_SUCCESSFULLY;
}



soc_status_t soc_gpio_configPort(soc_gpio_config_t* pinConf)
{
	// Input check
	SOC_ASSERT(pinConf->port);
	SOC_ASSERT(pinConf->pin_dir);

	// Read the port
	volatile uint32_t base_addr = SOCR_GPIO_BASE((int) pinConf->port);
	SOC_ASSERT(base_addr);

	// Check that all selected GPIO pins are available on the port
	SOC__GPIO_PORT_CHECK(pinConf->port,pinConf->pin_mask);

	// Set the direction of the pin
	if (pinConf->pin_dir==SOC_GPIO_PIN_OUTPUT)
	{
		// Write a "1" to every pin mask register to the pins to output
		soc_replbits_word(base_addr+SOCRO_SWPORTA_DDR,pinConf->pin_mask,\
				SOC_GPIO_BITMASK);
	}

	// Set the synchronization
	if (pinConf->pin_sync==SOC_GPIO_SYNC_L4_MP_CLK)
	{
		soc_replbits_word(base_addr+SOCRO_GPIO_LS_SYNC,pinConf->pin_mask,\
				1);
	}

	// Enable Debounce mode
	if (pinConf->pin_deb==SOC_GPIO_PIN_DEBOUNCE)
	{
		// Write a "1" to every pin mask register
		soc_replbits_word(base_addr+SOCRO_GPIO_DEBPUNCE,pinConf->pin_mask,\
				SOC_GPIO_BITMASK);
	}

	// Set the output pins
	if (pinConf->pin_dir==SOC_GPIO_PIN_OUTPUT)
		soc_gpio_write(pinConf->port,pinConf->pin_output_val);


	return SOC_S_SUCCESSFULLY;
}

soc_status_t soc_gpio_ISRconfig(soc_gpio_config_t* pinConf)
{
	SOC_ASSERT(pinConf->pin_type);
	SOC_ASSERT(pinConf->pin_pol);
	SOC_ASSERT(pinConf->isr_mask);

	// Read the port
	volatile uint32_t base_addr = SOCR_GPIO_BASE((int) pinConf->port);
	SOC_ASSERT(base_addr);


	// Check that all selected GPIO pins are available on the port
	SOC__GPIO_PORT_CHECK(pinConf->port,pinConf->isr_pin_mask);

	// Clear the ISR
	soc_gpio_ISRenable(pinConf->port,pinConf->isr_pin_mask,false);

	// Set trigger Type
	if (pinConf->pin_type==SOC_GPIO_PIN_EDGE_TRIG_INT)
	{
		// Write a "1" to every pin mask register
		soc_replbits_word(base_addr+SOCRO_GPIO_INTTYPE_LEVEL,pinConf->isr_pin_mask,\
				SOC_GPIO_BITMASK);
	}

	// Set polarity Type
	if (pinConf->pin_pol==SOC_GPIO_PIN_ACTIVE_HIGH)
	{
		// Write a "1" to every pin mask register
		soc_replbits_word(base_addr+SOCRO_GPIO_INT_POLARITY,pinConf->isr_pin_mask,\
				SOC_GPIO_BITMASK);
	}

	// Set Interrupting
	if(pinConf->isr_mask == SOC_GPIO_ISR_MASKED)
	{
		// Write a "1" to every pin mask register
		soc_replbits_word(base_addr+SOCRO_GPIO_INTMASK,pinConf->isr_pin_mask,\
				SOC_GPIO_BITMASK);
	}

	return SOC_S_SUCCESSFULLY;
}

soc_status_t soc_gpio_ISRenable(soc_gpio_port_t port, uint32_t pin_mask, \
		bool enable_disable)
{

	// Check that all selected GPIO pins are available on the port
	SOC__GPIO_PORT_CHECK(port,pin_mask);

	// Read the port
	volatile uint32_t base_addr = SOCR_GPIO_BASE((int) port);
	SOC_ASSERT(base_addr);

	if(enable_disable)
	{
		// Enable or disable the interrupt line for the pin
		soc_setbits_word(base_addr+SOCRO_GPIO_INTEN,pin_mask);
	}
	else
	{
		// Disable or disable the interrupt line for the pin
		soc_clrbits_word(base_addr+SOCRO_GPIO_INTEN,pin_mask);
	}

	return SOC_S_SUCCESSFULLY;
}


soc_status_t soc_gpio_write(soc_gpio_port_t port, uint32_t pin_output_val)
{
	// Read the port
	volatile uint32_t base_addr = SOCR_GPIO_BASE((int) port);
	SOC_ASSERT(base_addr);

	// Check that all selected GPIO pins are available on the port
	SOC__GPIO_PORT_CHECK(port,pin_output_val);

	soc_write_word(base_addr+SOCRO_SWPORTA_DR,pin_output_val);

	return SOC_S_SUCCESSFULLY;
}

soc_status_t soc_gpio_set(soc_gpio_port_t port, uint32_t pin_mask)
{
	// Read the port
	volatile uint32_t base_addr = SOCR_GPIO_BASE((int) port);
	SOC_ASSERT(base_addr);

	// Check that all selected GPIO pins are available on the port
	SOC__GPIO_PORT_CHECK(port,pin_mask);

	soc_write_word(base_addr+SOCRO_SWPORTA_DR,pin_mask);

	return SOC_S_SUCCESSFULLY;
}

soc_status_t soc_gpio_clear(soc_gpio_port_t port, uint32_t pin_mask)
{
	// Read the port
	volatile uint32_t base_addr = SOCR_GPIO_BASE((int) port);
	SOC_ASSERT(base_addr);

	// Check that all selected GPIO pins are available on the port
	SOC__GPIO_PORT_CHECK(port,pin_mask);

	soc_write_word(base_addr+SOCRO_SWPORTA_DR,~pin_mask);

	return SOC_S_SUCCESSFULLY;
}

uint8_t soc_gpio_read (soc_gpio_port_t port, uint32_t pin)
{
	// Read the port
	volatile uint32_t base_addr = SOCR_GPIO_BASE((int) port);
	SOC_ASSERT(base_addr);

	// Check that all selected GPIO pins are available on the port
	SOC__GPIO_PORT_CHECK(port,(1<<pin));

	uint32_t value = soc_read_word(base_addr+SOCRO_EXT_PORTA);

	return value & (1<<pin) ? 1:0;


}
