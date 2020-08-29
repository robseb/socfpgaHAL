/*
 * socfpgaHAL.h
 *
 *  Created on: 26.08.2020
 *      Author: robin
 */

#ifndef SOCFPGAHAL_INC_SOCFPGAHAL_H_
#define SOCFPGAHAL_INC_SOCFPGAHAL_H_



#include "system.h"
#include "socfpgaHAL_config.h"


#define SOC_S_SUCCESSFULLY      1

#define SOC_S_GENERAL_ERROR    -1
#define SOC_S_WRONG_INPUT      -2
#define SOC_S_FAIELD_TO_ACCESS -3
#define SOC_S_BAD_CLOCK		   -4



#define SOC_ASSERT(in) if((int)in==0) asm( "break" )

typedef enum{
	SUCCESSFULLY 		= 1,

	GENERAL_ERROR 		=-1,
	WRONG_INPUT			=-2,
	FAIELD_TO_ACCESS 	=-3,
	BAD_CLOCK		    =-4

} soc_status_t;


/*
 *
 *  Include the Files depending on the hardware configuration
 */
#ifdef SOCFPGAHAL_MODE_NIOS

#define SOCFPGAHAL_ENABLE_SPTIMER0 	    (1)
#define SOCFPGAHAL_ENABLE_SPTIMER1  	(1)
#define SOCFPGAHAL_ENABLE_OSCTIMER0 	(1)
#define SOCFPGAHAL_ENABLE_OSCTIMER1 	(1)


	#if SOCFPGAHAL_ENABLE_GPIO == 1
		#include "soc_gpio.h"
	#endif
	#if SOCFPGAHAL_ENABLE_SPTIMER0==1 || SOCFPGAHAL_ENABLE_SPTIMER1==1 || \
		SOCFPGAHAL_ENABLE_SPTIMER1==1 || SOCFPGAHAL_ENABLE_OSCTIMER1==1
		#include "soc_timer.h"
	#endif
	#if SOCFPGAHAL_ENABLE_UART1==1
		#include "soc_uart.h"
	#endif
#endif

#endif /* SOCFPGAHAL_INC_SOCFPGAHAL_H_ */
