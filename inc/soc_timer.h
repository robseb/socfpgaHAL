/*
 * soc_timer.h
 *
 *  Created on: 28.08.2020
 *      Author: robin
 */

#ifndef SOCFPGAHAL_INC_SOC_TIMER_H_
#define SOCFPGAHAL_INC_SOC_TIMER_H_


#ifdef __cplusplus
extern "C"
{
#endif  /* __cplusplus */

#include "socfpgaHAL.h"
#include <stdint.h>
#include <stdbool.h>

/*
 *
 *  Base Registers Address
 *
 */
#ifdef SOC_CY_AV
	#define SOCR_SPTIMER0_BASE  (0xFFC08000+FPGA2HPS_BASE_OFFSET-SUB_WINDOW_OFFSET)
	#define SOCR_SPTIMER1_BASE  (0xFFC09000+FPGA2HPS_BASE_OFFSET-SUB_WINDOW_OFFSET)
	#define SOCR_OSCTIMER0_BASE (0xFFD00000+FPGA2HPS_BASE_OFFSET-SUB_WINDOW_OFFSET)
	#define SOCR_OSCTIMER1_BASE (0xFFD01000+FPGA2HPS_BASE_OFFSET-SUB_WINDOW_OFFSET)


	#define SOCR_TIMER_BASE(no)(no==1) ? SOCR_SPTIMER0_BASE : \
							   (no==2) ? SOCR_SPTIMER1_BASE : \
							   (no==3) ? SOCR_OSCTIMER0_BASE: \
							   (no==4) ? SOCR_OSCTIMER1_BASE: \
							    0

	/*
	 *
	 *  Register offset
	 *
	 */

	#define SOCRO_TIMER1LOADCOUNT      	0x00 // Timer1 Load Count Register
	#define SOCRO_TIMER1CURRENTVAL     	0x04 // Timer1 Current Value Register
	#define SOCRO_TIMER1CONTROLREG   	0x08 // Timer1 Control Register
	#define SOCRO_TIMER1EOI			   	0x0C // Timer1 End-of-Interrupt Register
	#define SOCRO_TIMER1INTSTAT			0x10 // Timer1 Interrupt Status Register
	#define SOCRO_TIMERSINTSTAT			0xA0 // Timers Interrupt Status Register
	#define SOCRO_TIMERSEOI			   	0xA4 // Timers End-of-Interrupt Register
	#define SOCRO_TIMERSRAWINTSTAT	    0xA8 // Timers Interrupt Status Register
	#define SOCRO_TIMERSCOMPVERSION	    0xAC // Timers Component Version Register

	#define SOCCC_TIMERSRAWINTSTAT		0x3230352A // Timers Component Version Register Reset value


	#define SOCRO_TIMER1CONTROLREG_EN  	   (1<<0) // Enable timer
	#define SOCRO_TIMER1CONTROLREG_PERODIC (1<<1) // Periodic Mode
	#define SOCRO_TIMER1CONTROLREG_ISRMASK (1<<2) // Interrupt mode
#endif


typedef enum
{
#if SOCFPGAHAL_ENABLE_SPTIMER0==1
	SOC_TIMER_SPTIMER0  =1,
#endif
#if SOCFPGAHAL_ENABLE_SPTIMER1==1
	SOC_TIMER_SPTIMER1  =2,
#endif
#if SOCFPGAHAL_ENABLE_OSCTIMER0==1
	SOC_TIMER_OSCTIMER0 =3,
#endif
#if SOCFPGAHAL_ENABLE_OSCTIMER1==1
	SOC_TIMER_OSCTIMER1 =4,
#endif
	____DO_NOT_USE =-1
}  soc_timer_module_t;

typedef enum
{
	SOC_TIMER_ONESHOT   =1,
	SOC_TIMER_PERIODIC  =2,
}  soc_timer_mode_t;


soc_status_t soc_timer_init(void);
soc_status_t soc_timer_deinit(void);


soc_status_t soc_timer_start (soc_timer_module_t module, soc_timer_mode_t mode);
soc_status_t soc_timer_stop  (soc_timer_module_t module);
soc_status_t soc_timer_status(soc_timer_module_t module, bool* status);

soc_status_t soc_timer_counter_set (soc_timer_module_t module, uint32_t val);
soc_status_t soc_timer_prescaler_set (soc_timer_module_t module,uint32_t val);

soc_status_t soc_timer_get (soc_timer_module_t module,uint32_t* val);

soc_status_t soc_timer_ISRconfig (soc_timer_module_t module);
soc_status_t soc_timer_ISRenable (soc_timer_module_t module,bool enable_disable);
soc_status_t soc_timer_ISRclearPending (soc_timer_module_t module);

#ifdef __cplusplus
}
#endif  /* __cplusplus */


#endif /* SOCFPGAHAL_INC_SOC_TIMER_H_ */
