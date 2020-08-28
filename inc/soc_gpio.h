/*
 * soc_gpio.h
 *
 *  Created on: 27.08.2020
 *      Author: robin
 */

#ifndef SOCFPGAHAL_INC_SOC_GPIO_H_
#define SOCFPGAHAL_INC_SOC_GPIO_H_


#include "socfpgaHAL/inc/socfpgaHAL.h"
#include <stdio.h>

/*
 *
 *  Base Registers Address
 *
 */
#define SOCR_GPIO0_BASE 0xFF708000
#define SOCR_GPIO1_BASE 0xFF709000
#define SOCR_GPIO2_BASE 0xFF70A000

#define SOCR_GPIO_BASE(no) if(no==0) SOCR_GPIO0_BASE; \
						   else if(no==1) SOCR_GPIO1_BASE; \
						   else SOCR_GPIO2_BASE;

/*
 *
 *  Register offset
 *
 */
#define SOCRO_SWPORTA_DR      		0x00 // Port A Data Register
#define SOCRO_SWPORTA_DDR     		0x04 // Port A Data Direction Register
#define SOCRO_GPIO_INTEN      		0x30 // Interrupt Enable Register
#define SOCRO_GPIO_INTMASK    		0x34 // Interrupt Mask Register
#define SOCRO_GPIO_INTTYPE_LEVEL    0x38 // Interrupt Level Register
#define SOCRO_GPIO_INT_POLARITY     0x3C // Interrupt Polarity Register
#define SOCRO_GPIO_INTSTATUS        0x40 // Interrupt Status Register

#define SOCRO_GPIO_RAW_INITSTATUS   0x44 // Raw Interrupt Status Register
#define SOCRO_GPIO_DEBPUNCE         0x48 // Debounce Enable Register
#define SOCRO_GPIO_PORTA_EOI        0x4C // Clear Interrupt Register
#define SOCRO_EXT_PORTA             0x50 // External Port A Register
#define SOCRO_GPIO_LS_SYNC          0x60 // Synchronization Level Register

#define SOCRO_GPIO_ID_CODE        	0x64 // ID Code Register
#define SOCRO_GPIO_VER_ID_CODE      0x6C // GPIO Version Register
#define SOCRO_GPIO_CONFIG_REG2      0x70 // Configuration Register 2
#define SOCRO_GPIO_CONFIG_REG1      0x74 // Configuration Register 1


#define SOCC_GPIO_VER_ID_CODE		0x3230382A // GPIO Version reset value

/*
ALT_STATUS_CODE alt_gpio_port_config(ALT_GPIO_PORT_t gpio_pid,
        uint32_t mask, ALT_GPIO_PIN_DIR_t dir,
        ALT_GPIO_PIN_TYPE_t type,
        ALT_GPIO_PIN_POL_t pol, ALT_GPIO_PIN_DEBOUNCE_t debounc,
        uint32_t data)
        */

/*!
 * This type definition enumerates the data direction (input or output) of
 * the GPIO signals.
 */
typedef enum
{
    SOC_GPIO_PIN_INPUT,
    SOC_GPIO_PIN_OUTPUT
} soc_gpio_dir_t;

typedef enum
{
    SOC_GPIO_PORTA,
    SOC_GPIO_PORTB,
}  soc_gpio_port_t;


/*!
 * This type definition enumerates the type of interrupt source
 * (level-triggered or edge-triggered) of the GPIO signals.
 */

typedef enum ALT_GPIO_PIN_TYPE_e
{
    SOC_GPIO_PIN_LEVEL_TRIG_INT,
    SOC_GPIO_PIN_EDGE_TRIG_INT
} soc_gpio_type_t;


/*!
 * This type definition enumerates the polarity of the interrupt sources
 * (falling-edge or rising-edge for edge-triggered interrupts, active-low or
 * active-high for level-triggered interrupts) of the GPIO signals.
 */

typedef enum
{
    /*! Indicates active-low for level-triggered interrupts and
     * falling-edge for edge-triggered interrupts */
    SOC_GPIO_PIN_ACTIVE_LOW,

    /*! Indicates active-high for level-triggered interrupts and
     * rising-edge for edge-triggered interrupt */
    SOC_GPIO_PIN_ACTIVE_HIGH
} soc_gpio_pol_t;


/*!
 * This type definition enumerates whether or not the debounce metastability
 * flip-flops are inserted or not. These are used to debounce signals presented
 * to the GPIO inputs. A signal must be steady for two periods of the
 * gpio_db_clk clock before it is considered valid. The frequency of the
 * gpio_db_clk clock may be set using the Clock Manager API.
 */

typedef enum
{
    SOC_GPIO_PIN_NODEBOUNCE,
    SOC_GPIO_PIN_DEBOUNCE
} soc_gpio_debounce_t;

typedef enum
{
    SOC_GPIO_ISR_LEVEL_SENSITIVE =0,
	SOC_GPIO_ISR_EDGE_TRIGGERED  =1
} soc_gpio_isr_type_t;

typedef enum
{
	SOC_GPIO_INT_POL_ACTIVE_LOW  =0,
	SOC_GPIO_INT_POL_ACTIVE_HIGH =1
}soc_gpio_int_polarity_t;


/*!
 * GPIO Pin configuration
 */
typedef struct{
	soc_gpio_port_t     port;
	uint32_t            pin_mask;
	soc_gpio_dir_t      pin_dir;
	soc_gpio_type_t     pin_type;
	soc_gpio_pol_t      pin_pol;
	soc_gpio_debounce_t pin_deb;
	uint32_t		    pin_output_val;
}soc_gpio_config_t;

typedef struct{
	soc_gpio_isr_type_t     isr_type;
	soc_gpio_int_polarity_t isr_pol;
}soc_gpio_isrconfig_t;


soc_staus_t soc_gpio_init(void);
soc_staus_t soc_gpio_deinit(void);
soc_staus_t soc_gpio_configPort(soc_gpio_config_t pinConf);

soc_staus_t soc_gpio_ISRconfig(soc_gpio_isrconfig_t isrConf);
soc_staus_t soc_gpio_ISRenable(soc_gpio_port_t port, uint32_t pin_mask);
soc_staus_t soc_gpio_ISRclear(soc_gpio_port_t port, uint32_t pin_mask);

soc_staus_t soc_gpio_write(soc_gpio_port_t port, uint32_t pin_mask);
uint32_t    soc_gpio_read (soc_gpio_port_t port, uint32_t pin_mask);



#endif /* SOCFPGAHAL_INC_SOC_GPIO_H_ */


