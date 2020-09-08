/*
 * soc_gpio.h
 *
 *  Created on: 27.08.2020
 *      Author: robin
 */

#ifndef SOCFPGAHAL_INC_SOC_GPIO_H_
#define SOCFPGAHAL_INC_SOC_GPIO_H_

#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

#include "socfpgaHAL.h"
#include <stdint.h>
#include <stdbool.h>

#include "soc_regacc.h"
#include "soc_resetManager.h"


/*
 *
 *  Base Registers Address
 *
 */
#ifdef SOC_CY_AV
	#define SOCR_GPIO0_BASE (0xFF708000+FPGA2HPS_BASE_OFFSET-SUB_WINDOW_OFFSET)
	#define SOCR_GPIO1_BASE (0xFF709000+FPGA2HPS_BASE_OFFSET-SUB_WINDOW_OFFSET)
	#define SOCR_GPIO2_BASE (0xFF70A000+FPGA2HPS_BASE_OFFSET-SUB_WINDOW_OFFSET)

	#define SOCR_GPIO_BASE(no) (no==1) ? SOCR_GPIO0_BASE : \
							   (no==2) ? SOCR_GPIO1_BASE : \
							   (no==3) ? SOCR_GPIO2_BASE: \
							    0

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
#endif

/*!
 * This type definition enumerates the data direction (input or output) of
 * the GPIO signals.
 */
typedef enum
{
    SOC_GPIO_PIN_INPUT =1,
    SOC_GPIO_PIN_OUTPUT =2
} soc_gpio_dir_t;

typedef enum
{
    SOC_GPIO_PORTA =1, // GPIO 0
    SOC_GPIO_PORTB =2, // GPIO 1
	SOC_GPIO_PORTC =3  // GPIO 2
}  soc_gpio_port_t;


/*!
 * This type definition enumerates the type of interrupt source
 * (level-triggered or edge-triggered) of the GPIO signals.
 */

typedef enum ALT_GPIO_PIN_TYPE_e
{
    SOC_GPIO_PIN_LEVEL_TRIG_INT =1,
    SOC_GPIO_PIN_EDGE_TRIG_INT =2
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
    SOC_GPIO_PIN_ACTIVE_LOW =1,

    /*! Indicates active-high for level-triggered interrupts and
     * rising-edge for edge-triggered interrupt */
    SOC_GPIO_PIN_ACTIVE_HIGH =2,


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
    SOC_GPIO_PIN_NODEBOUNCE =1,
    SOC_GPIO_PIN_DEBOUNCE =2
} soc_gpio_debounce_t;

typedef enum
{
    SOC_GPIO_ISR_LEVEL_SENSITIVE =1,
	SOC_GPIO_ISR_EDGE_TRIGGERED  =2
} soc_gpio_isr_type_t;

typedef enum
{
    SOC_GPIO_NO_SYNC =1,
	SOC_GPIO_SYNC_L4_MP_CLK  =2
} soc_gpio_sync_t;

typedef enum
{
    SOC_GPIO_ISR_UNMASKED =1,
	SOC_GPIO_ISR_MASKED  =2
} soc_gpio_isr_mask_t;


/*!
 * GPIO Pin configuration
 */
typedef struct{
	soc_gpio_port_t     	port;
	uint32_t            	pin_mask;
	soc_gpio_dir_t      	pin_dir;
	soc_gpio_type_t     	pin_type;
	soc_gpio_pol_t      	pin_pol;
	soc_gpio_debounce_t 	pin_deb;
	soc_gpio_sync_t         pin_sync;
	uint32_t		    	pin_output_val;
	soc_gpio_isr_type_t     isr_type;
	uint32_t 				isr_pin_mask;
	soc_gpio_isr_mask_t     isr_mask;

}soc_gpio_config_t;


soc_status_t soc_gpio_init(void);
soc_status_t soc_gpio_deinit(void);
soc_status_t soc_gpio_configPort(soc_gpio_config_t* pinConf);

soc_status_t soc_gpio_ISRconfig(soc_gpio_config_t* pinConf);
soc_status_t soc_gpio_ISRenable(soc_gpio_port_t port, uint32_t pin_mask, \
		bool enable_disable);

soc_status_t soc_gpio_write(soc_gpio_port_t port, uint32_t pin_output_val);
soc_status_t soc_gpio_set(soc_gpio_port_t port, uint32_t pin_mask);
soc_status_t soc_gpio_clear(soc_gpio_port_t port, uint32_t pin_mask);

uint8_t soc_gpio_read (soc_gpio_port_t port, uint32_t pin);
#ifdef __cplusplus
}
#endif  /* __cplusplus */

#endif /* SOCFPGAHAL_INC_SOC_GPIO_H_ */


