/*
 * socfpgaHAL_config.h
 *
 *  Created on: 28.08.2020
 *      Author: robin
 */

#ifndef SOCFPGAHAL_INC_SOCFPGAHAL_CONFIG_H_
#define SOCFPGAHAL_INC_SOCFPGAHAL_CONFIG_H_

#include "socfpgaHAL.h"


#define FPGA2HPS_BASE_OFFSET 0

/*
 *
 *
 */
#define SUB_WINDOW_OFFSET ADDRESS_SPAN_EXTENDER_WINDOWED_SLAVE_SPAN


// Operating Modes --> for using a NIOS II Processor to
//                     access the HPS hard IP via the
//                     FPGA2HPS bridge
//#define SOCFPGAHAL_MODE_NIOS

#if defined(SOC_CY_AV) && defined(SOCFPGAHAL_MODE_NIOS)
// GPIO
	#define SOCFPGAHAL_ENABLE_GPIO 			(1)

// Timer
	#define SOCFPGAHAL_ENABLE_SPTIMER0 	    (1)
	#define SOCFPGAHAL_ENABLE_SPTIMER1  	(1)
	#define SOCFPGAHAL_ENABLE_OSCTIMER0 	(1)
	#define SOCFPGAHAL_ENABLE_OSCTIMER1 	(1)
// BUS
	#define SOCFPGAHAL_ENABLE_UART1      	(1)


#else
	#error "The socfpga HAL operation mode or the FPGA device is not speczified"
#endif

#endif /* SOCFPGAHAL_INC_SOCFPGAHAL_CONFIG_H_ */
