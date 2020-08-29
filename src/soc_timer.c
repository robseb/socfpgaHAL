/*
 * soc_timer.c
 *
 *  Created on: 28.08.2020
 *      Author: robin
 */

#include "socfpgaHAL.h"
#include "soc_timer.h"

#include "soc_resetManager.h"
#include "soc_regacc.h"

soc_status_t soc_timer_init(void)
{
	soc_timer_deinit();

	// Relase timer modules form RESET
	uint32_t mask =0;

#if SOCFPGAHAL_ENABLE_SPTIMER0==1
	mask = SOC_RSTMGR_PERMODRST_SPTMR0_SET_MSK;
#endif
#if SOCFPGAHAL_ENABLE_SPTIMER1==1
	mask |= SOC_RSTMGR_PERMODRST_SPTMR1_SET_MSK;
#endif
#if SOCFPGAHAL_ENABLE_OSCTIMER0==1
	mask | = SOC_RSTMGR_PERMODRST_OSC1TMR0_SET_MSK;
#endif
#if SOCFPGAHAL_ENABLE_OSCTIMER1==1
	mask | = SOC_RSTMGR_PERMODRST_OSC1TMR1_SET_MSK;
#endif

	alt_clrbits_word(SOCR_RESTMAN_BASE+SOC_RSTMGR_PERMODRST_OFST,mask);

	return SOC_S_SUCCESSFULLY;
}

soc_status_t soc_timer_deinit(void)
{
	// L4 general-purpose timer modules into system manager reset
	uint32_t mask =0;

#if SOCFPGAHAL_ENABLE_SPTIMER0==1
	mask = SOC_RSTMGR_PERMODRST_SPTMR0_SET_MSK;
#endif
#if SOCFPGAHAL_ENABLE_SPTIMER1==1
	mask |= SOC_RSTMGR_PERMODRST_SPTMR1_SET_MSK;
#endif
#if SOCFPGAHAL_ENABLE_OSCTIMER0==1
	mask | = SOC_RSTMGR_PERMODRST_OSC1TMR0_SET_MSK;
#endif
#if SOCFPGAHAL_ENABLE_OSCTIMER1==1
	mask | = SOC_RSTMGR_PERMODRST_OSC1TMR1_SET_MSK;
#endif

	alt_setbits_word(SOCR_RESTMAN_BASE+SOC_RSTMGR_PERMODRST_OFST,mask);

	return SOC_S_SUCCESSFULLY;
}


soc_status_t soc_timer_start (soc_timer_module_t module, soc_timer_mode_t mode)
{
	uint32_t             regmask;
	volatile uint32_t   *regaddr;

	SOC_ASSERT(SOCR_TIMER_BASE((int) module));
	SOC_ASSERT(module);
	SOC_ASSERT(mode);

	volatile uint32_t base_addr = SOCR_TIMER_BASE((int) module) +SOCRO_TIMER1CONTROLREG;
	regmask = SOCRO_TIMER1CONTROLREG_EN;

	// Enable instat free running mode the perodic mode
	if (mode==SOC_TIMER_PERIODIC)
		regmask |= SOCRO_TIMER1CONTROLREG_PERODIC;

	soc_write_word(regaddr, soc_read_word(regaddr) | regmask);

	return SOC_S_SUCCESSFULLY;
}

soc_status_t soc_timer_stop (soc_timer_module_t module)
{
	uint32_t             regmask;

	SOC_ASSERT(SOCR_TIMER_BASE((int) module));
	SOC_ASSERT(module);

	volatile uint32_t addr = SOCR_TIMER_BASE((int) module)+SOCRO_TIMER1CONTROLREG;
	regmask &= ~SOCRO_TIMER1CONTROLREG_EN;

	soc_write_word(addr, soc_read_word(addr) | regmask);

	return SOC_S_SUCCESSFULLY;
}

soc_status_t soc_timer_status(soc_timer_module_t module, bool* status)
{
	uint32_t	regdata=0;

	SOC_ASSERT(SOCR_TIMER_BASE((int) module));
	SOC_ASSERT(module);

	volatile uint32_t base_addr = SOCR_TIMER_BASE((int) module)+SOCRO_TIMER1CONTROLREG;

	regdata = alt_read_word(base_addr);
	status = (regdata & SOCRO_TIMER1CONTROLREG_EN);

	return SOC_S_SUCCESSFULLY;
}

soc_status_t soc_timer_counter_set (soc_timer_module_t module, uint32_t val)
{

	SOC_ASSERT(SOCR_TIMER_BASE((int) module));
	SOC_ASSERT(module);

	volatile uint32_t addr = SOCR_TIMER_BASE((int) module) +SOCRO_TIMER1LOADCOUNT;

	soc_write_word(addr, val);

	return SOC_S_SUCCESSFULLY;
}


soc_status_t soc_timer_get (soc_timer_module_t module,uint32_t* val)
{

	SOC_ASSERT(SOCR_TIMER_BASE((int) module));
	SOC_ASSERT(module);

	volatile uint32_t addr = SOCR_TIMER_BASE((int) module) +SOCRO_TIMER1CURRENTVAL;

	val = soc_read_word(addr);

	return SOC_S_SUCCESSFULLY;
}


soc_status_t soc_timer_ISRenable (soc_timer_module_t module, bool enable_disable)
{
	volatile uint32_t *regaddr;

	SOC_ASSERT(SOCR_TIMER_BASE((int) module));
	SOC_ASSERT(module);

#define ALT_TMR_TMR1CTLREG_TMR1_INT_MSK_CLR_MSK    0xfffffffb

	volatile uint32_t base_addr = SOCR_TIMER_BASE((int) module) +SOCRO_TIMER1CONTROLREG;

	if (enable_disable)
		alt_write_word(regaddr, alt_read_word(regaddr) & SOCRO_TIMER1CONTROLREG_ISRMASK);
	else
		alt_write_word(regaddr, alt_read_word(regaddr) | SOCRO_TIMER1CONTROLREG_ISRMASK);

	return SOC_S_SUCCESSFULLY;

}

soc_status_t soc_timer_ISRclearPending (soc_timer_module_t module)
{
	SOC_ASSERT(SOCR_TIMER_BASE((int) module));
	SOC_ASSERT(module);

	volatile uint32_t addr = SOCR_TIMER_BASE((int) module) +SOCRO_TIMER1EOI;
    (void) alt_read_word(addr);

	return SOC_S_SUCCESSFULLY;
}

