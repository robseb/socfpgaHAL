/*
 * socfpgaHAL.h
 *
 *  Created on: 26.08.2020
 *      Author: robin
 */

#ifndef SOCFPGAHAL_INC_SOCFPGAHAL_H_
#define SOCFPGAHAL_INC_SOCFPGAHAL_H_


#define FPGA2HPS_BASE_OFFSET 0


#define SOC_S_SUCCESSFULLY      1
#define SOC_S_FAIELD_TO_ACCESS -1
typedef enum{
	SUCCESSFULLY = 1,
	FAIELD_TO_ACCESS =-1
} soc_staus_t;

// Operating Modes --> for using a NIOS II Processor to
//                     access the HPS hard IP via the
//                     FPGA2HPS bridge
//#define SOCFPGAHAL_MODE_NIOS

#ifdef SOCFPGAHAL_MODE_NIOS
	#define SOCFPGAHAL_ENABLE_GPIO


#else
#error "The socfpga HAL operation mode is not speczified"
#endif


#ifdef SOCFPGAHAL_MODE_NIOS
	#ifdef SOCFPGAHAL_ENABLE_GPIO
		#include "socfpgaHAL/inc/soc_gpio.h"
	#endif
#endif

#endif /* SOCFPGAHAL_INC_SOCFPGAHAL_H_ */
