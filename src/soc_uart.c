
#include "soc_16550_uart.h"
#include "soc_clock_manager.h"
#include "socal/soc_rstmgr.h"
#include "socal/soc_uart.h"
#include "socal/hps.h"
#include "socal/socal.h"

#define DEFAULT_BAUD soc_16550_BAUDRATE_115200

#define soc_16550_HANDLE_DATA_UART_ENABLED_MSK   (1UL << 31)
#define soc_16550_HANDLE_DATA_DIVISOR_VALUE_GET(value) (value & 0xffff)

#define soc_socERA_16550_CPR_OFST        (0xF4)
#define soc_socERA_16550_CPR_ADDR(base)  soc_CAST(void *, (soc_CAST(char *, (base)) + soc_socERA_16550_CPR_OFST))
#define soc_socERA_16550_CPR_FIFO_MODE_GET(value) (((value) >> 16) & 0xff)
#define soc_socERA_16550_CPR_AFCE_MODE_SET_MSK (1 << 4)

/*  Remove these macros as part of case:123835.*/
#define soc_UART_IER_DLH_VALUE_SET(value) ((value) & 0xff)
#define soc_UART_IER_DLH_ETBEI_DLH1_SET_MSK soc_UART_IER_DLH_ETBEI_DLHL_SET_MSK


/*
// Helper function which resets the UART and if requested, initializes the UART
// to the default settings. Currently the default settings are:
//  - 8 databits
//  - no parity
//  - 1 stopbit
//  - 57600 baudrate
// The reset routines depends on the hardware implementation of the UART.
*/

/* This helper is needed because the regular soc_read_word(src) essentially
// resolves to "*(volatile uint32_t *)src". As there is no assignment, this
// could potentially be optimized away. With the helper, the actual register
// read should occur and be returned (and subsequently discarded).
*/
static __inline uint32_t soc_read_word_helper(const void * addr)
{
    return soc_read_word(addr);
}

/*
// Helper function write the divisor in hardware.
*/
static soc_STATUS_CODE soc_16550_write_divisor_helper(soc_16550_HANDLE_t * handle,
                                                      uint32_t divisor)
{
    /* Validate the divisor parameter. */
    if (divisor > 0xffff)
    {
        /* This should never happen as it is verified in divisor_set. */
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Set LCR::DLAB (Line Control Register :: Divisor Latch Access Bit) */
        soc_setbits_word(soc_UART_LCR_ADDR(handle->location), soc_UART_LCR_DLAB_SET_MSK);

        /* Write DLL (Divisor Latch Low). */
        soc_write_word(soc_UART_RBR_THR_DLL_ADDR(handle->location), soc_UART_RBR_THR_DLL_VALUE_SET(divisor));

        /* Write DLH (Divisor Latch High). */
        soc_write_word(soc_UART_IER_DLH_ADDR(handle->location), soc_UART_IER_DLH_VALUE_SET(divisor >> 8));

        /* Clear LCR::DLAB (Line Control Register :: Divisor Latch Access Bit) */
        soc_clrbits_word(soc_UART_LCR_ADDR(handle->location), soc_UART_LCR_DLAB_SET_MSK);

        break;

    default:
        return soc_E_ERROR;
    }

    /* Update the enabled state in the handle data. */
    if (divisor != 0)
    {
        handle->data |= soc_16550_HANDLE_DATA_UART_ENABLED_MSK;
    }
    else
    {
        handle->data &= ~soc_16550_HANDLE_DATA_UART_ENABLED_MSK;
    }

    return soc_E_SUCCESS;
}

/*
// Helper function to reset the UART.
*/
static soc_STATUS_CODE soc_16550_reset_helper(soc_16550_HANDLE_t * handle, bool enable_init)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
        /* Write SRR::UR (Shadow Reset Register :: UART Reset) */
        soc_write_word(soc_UART_SRR_ADDR(handle->location), soc_UART_SRR_UR_SET_MSK);

        /* Read the MSR to work around case:119085. */
        soc_read_word_helper(soc_UART_MSR_ADDR(handle->location));
        break;

    case soc_16550_DEVICE_socERA_16550_UART:
        soc_16550_write_divisor_helper(handle, 0); /* Disable UART */
        soc_16550_int_disable_all(handle);         /* Disable interrupts */
        soc_16550_fifo_disable(handle);            /* Disable FIFOs */
        soc_write_word(soc_UART_MCR_ADDR(handle->location), 0); /* 0 -> MCR (AFCE, LP, OUT2, OUT1, RTS, DTR) */
        break;

    default:
        return soc_E_ERROR;
    }

    /* If we are initializing (as opposed to just uninitializing) */
    if (enable_init)
    {
        soc_STATUS_CODE status;
        uint32_t divisor;

        /* Set bit IER::PTIME (Interrupt Enable Register :: Programmable THRE Mode Enable) */
        soc_setbits_word(soc_UART_IER_DLH_ADDR(handle->location), soc_UART_IER_DLH_PTIME_DLH7_SET_MSK);

        /* Set the line configuration to use 8-N-1. */
        status = soc_16550_line_config_set(handle, soc_16550_DATABITS_8,
                                                   soc_16550_PARITY_DISABLE,
                                                   soc_16550_STOPBITS_1);
        if (status != soc_E_SUCCESS)
        {
            return status;
        }

        divisor = soc_16550_HANDLE_DATA_DIVISOR_VALUE_GET(handle->data);
        if (divisor == 0)
        {
            status = soc_16550_baudrate_set(handle, DEFAULT_BAUD);
            if (status != soc_E_SUCCESS)
            {
                return status;
            }
        }
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_init(soc_16550_DEVICE_t device,
                               void * location,
                               soc_freq_t clock_freq,
                               soc_16550_HANDLE_t * handle)
{
    handle->device = device;
    handle->data   = 0;
    handle->fcr    = 0;

    switch (device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
        /* The soc_CLK_L4_SP is required for all SoCFPGA UARTs. Check that it's enabled. */
        if (soc_clk_is_enabled(soc_CLK_L4_SP) != soc_E_TRUE)
        {
            return soc_E_BAD_CLK;
        }
        else
        {
            soc_STATUS_CODE status;
            #ifdef SOC_CY_AV
                uint32_t ucr;
            #endif

            status = soc_clk_freq_get(soc_CLK_L4_SP, &handle->clock_freq);
            if (status != soc_E_SUCCESS)
            {
                return status;
            }

            if (device == soc_16550_DEVICE_SOCFPGA_UART0)
            {
                handle->location = soc_UART0_ADDR;

                /* Bring UART0 out of reset. */
#ifdef SOC_CY_AV
                soc_clrbits_word(soc_RSTMGR_PERMODRST_ADDR, soc_RSTMGR_PERMODRST_UART0_SET_MSK);
#else
                soc_clrbits_word(soc_RSTMGR_PER1MODRST_ADDR, soc_RSTMGR_PER1MODRST_UART0_SET_MSK);
#endif
            }
            else /* device == soc_16550_DEVICE_SOCFPGA_UART1 */
            {
                handle->location = soc_UART1_ADDR;

                /* Bring UART1 out of reset. */
#ifdef SOC_CY_AV
                soc_clrbits_word(soc_RSTMGR_PERMODRST_ADDR, soc_RSTMGR_PERMODRST_UART1_SET_MSK);
#else
                soc_clrbits_word(soc_RSTMGR_PER1MODRST_ADDR, soc_RSTMGR_PER1MODRST_UART1_SET_MSK);
#endif
            }
#ifdef SOC_CY_AV
            /* Verify the UCR (UART Component Version) */
            ucr = soc_read_word(soc_UART_UCV_ADDR(handle->location));
            if (ucr != soc_UART_UCV_UART_COMPONENT_VER_RESET)
            {
                return soc_E_ERROR;
            }
#endif
        }
        break;
    case soc_16550_DEVICE_socERA_16550_UART:
        handle->location   = location;
        handle->clock_freq = clock_freq;
        break;
    default:
        return soc_E_BAD_ARG;
    }

    return soc_16550_reset_helper(handle, true);
}

soc_STATUS_CODE soc_16550_uninit(soc_16550_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
#ifdef SOC_CY_AV
        soc_setbits_word(soc_RSTMGR_PERMODRST_ADDR, soc_RSTMGR_PERMODRST_UART0_SET_MSK);
#else
        soc_setbits_word(soc_RSTMGR_PER1MODRST_ADDR, soc_RSTMGR_PER1MODRST_UART0_SET_MSK);
#endif
        return soc_E_SUCCESS;
    case soc_16550_DEVICE_SOCFPGA_UART1:
#ifdef SOC_CY_AV
        soc_setbits_word(soc_RSTMGR_PERMODRST_ADDR, soc_RSTMGR_PERMODRST_UART1_SET_MSK);
#else
        soc_setbits_word(soc_RSTMGR_PER1MODRST_ADDR, soc_RSTMGR_PER1MODRST_UART1_SET_MSK);
#endif
        return soc_E_SUCCESS;
    case soc_16550_DEVICE_socERA_16550_UART:
    default:
        return soc_16550_reset_helper(handle, false);
    }
}

soc_STATUS_CODE soc_16550_reset(soc_16550_HANDLE_t * handle)
{
    return soc_16550_reset_helper(handle, true);
}

soc_STATUS_CODE soc_16550_enable(soc_16550_HANDLE_t * handle)
{
    /* Enable Terminal*/
  soc_16550_modem_enable_dtr(handle);
  soc_16550_modem_enable_rts(handle);

    /* Write the divisor cached in the handle data to the divisor registers. */
    /* This will effectively enable the UART. */
    return soc_16550_write_divisor_helper(handle,
                                          soc_16550_HANDLE_DATA_DIVISOR_VALUE_GET(handle->data));

}

soc_STATUS_CODE soc_16550_disable(soc_16550_HANDLE_t * handle)
{
    /* Write 0 to the divisor the divisor registers. This will effectively */
    /* disable the UART. */
    return soc_16550_write_divisor_helper(handle, 0);
}

soc_STATUS_CODE soc_16550_read(soc_16550_HANDLE_t * handle,
                               char * item)
{
    /* Verify that the UART is enabled */
    if (!(handle->data & soc_16550_HANDLE_DATA_UART_ENABLED_MSK))
    {
        return soc_E_ERROR;
    }

    /* Verify that the FIFO is disabled */
    if (handle->fcr & soc_UART_FCR_FIFOE_SET_MSK)
    {
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Read the RBR (Receive Buffer Register) into *item. */
        *item = soc_UART_RBR_THR_DLL_VALUE_GET(soc_read_word(soc_UART_RBR_THR_DLL_ADDR(handle->location)));
        break;
    default:
        return soc_E_ERROR;
    }
    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_write(soc_16550_HANDLE_t * handle,
                                char item)
{
    /* Verify that the UART is enabled */
    if (!(handle->data & soc_16550_HANDLE_DATA_UART_ENABLED_MSK))
    {
        return soc_E_ERROR;
    }

    /* Verify that the FIFO is disabled */
    if (handle->fcr & soc_UART_FCR_FIFOE_SET_MSK)
    {
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Write the buffer into the THR (Transmit Holding Register) */
        soc_write_word(soc_UART_RBR_THR_DLL_ADDR(handle->location), item);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_enable(soc_16550_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Set FCR::FIFOE (FIFO Control Register :: FIFO Enable) bit. */
        handle->fcr |= soc_UART_FCR_FIFOE_SET_MSK | soc_UART_FCR_RFIFOR_SET_MSK | soc_UART_FCR_XFIFOR_SET_MSK;
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr);
        break;
    default:
        return soc_E_ERROR;
    }

    /* No need to reset / clear the FIFOs. This is done automatically when */
    /* FCR::FIFOE is changed. */
    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_disable(soc_16550_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Clear FCR::FIFOE (FIFO Control Register :: FIFO Enable) bit. */
        handle->fcr &= ~soc_UART_FCR_FIFOE_SET_MSK;
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_read(soc_16550_HANDLE_t * handle,
                                    char * buffer,
                                    size_t count)
{
    size_t i;
    /* Verify that the UART is enabled */
    if (!(handle->data & soc_16550_HANDLE_DATA_UART_ENABLED_MSK))
    {
        return soc_E_ERROR;
    }

    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Read the RBR (Receive Buffer Register) into the buffer */
        for (i = 0; i < count; ++i)
        {
            buffer[i] = soc_UART_RBR_THR_DLL_VALUE_GET(soc_read_word(soc_UART_RBR_THR_DLL_ADDR(handle->location)));
        }
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_write_safe(soc_16550_HANDLE_t * handle,
                                     const char * buffer,
                                     size_t count,
                                     bool safe)
{
    size_t i;
    /* Verify that the UART is enabled */
    if (!(handle->data & soc_16550_HANDLE_DATA_UART_ENABLED_MSK))
    {
        return soc_E_ERROR;
    }

    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Write the buffer into the THR (Transmit Holding Register) */
        for (i = 0; i < count; ++i)
        {
            if (safe)
                while (soc_UART_LSR_THRE_GET(soc_read_word(soc_UART_LSR_ADDR(handle->location))))
                    ; /* Spin waiting for space */
            soc_write_word(soc_UART_RBR_THR_DLL_ADDR(handle->location), buffer[i]);
        }
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_write(soc_16550_HANDLE_t * handle,
                                     const char * buffer,
                                     size_t count)
{
    return soc_16550_fifo_write_safe(handle, buffer, count, false);
}

soc_STATUS_CODE soc_16550_fifo_clear_rx(soc_16550_HANDLE_t * handle)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
        /* Write SRR::RFR (Shadow Reset Register :: Receiver FIFO Reset) bit. */
        soc_write_word(soc_UART_SRR_ADDR(handle->location), soc_UART_SRR_RFR_SET_MSK);
        break;
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Write FCR::RFIFOR (FIFO Control Register :: Receiver FIFO Reset) bit. */
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr | soc_UART_FCR_RFIFOR_SET_MSK);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_clear_tx(soc_16550_HANDLE_t * handle)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
        /* Write SRR::XFR (Shadow Reset Register :: Xmitter FIFO Reset) bit. */
        soc_write_word(soc_UART_SRR_ADDR(handle->location), soc_UART_SRR_XFR_SET_MSK);
        break;
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Write FCR::XFIFOR (FIFO Control Register :: Xmitter FIFO Reset) bit. */
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr | soc_UART_FCR_XFIFOR_SET_MSK);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_clear_all(soc_16550_HANDLE_t * handle)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
        /* Write SRR::(RFR | XFR) */
        /*   (Shadow Reset Register :: (Receiver FIFO Reset | Xmitter FIFO Reset)) bits. */
        soc_write_word(soc_UART_SRR_ADDR(handle->location),
                       soc_UART_SRR_RFR_SET_MSK | soc_UART_SRR_XFR_SET_MSK);
        break;
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Write FCR::(RFIFOR |XFIFOR) */
        /*   (FIFO Control Register :: (Receiver FIFO Reset | Xmitter FIFO Reset)) bits. */
        soc_write_word(soc_UART_FCR_ADDR(handle->location),
                       handle->fcr | soc_UART_FCR_RFIFOR_SET_MSK | soc_UART_FCR_XFIFOR_SET_MSK);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_size_get_rx(soc_16550_HANDLE_t * handle,
                                           uint32_t * size)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
        /* Read the CPR::FIFO_Mod (Component Parameter Register :: FIFO Mode). */
        /* The FIFO size is 16x this value. */
        *size = soc_UART_CPR_FIFO_MOD_GET(soc_read_word(soc_UART_CPR_ADDR(handle->location))) << 4;
        break;
    case soc_16550_DEVICE_socERA_16550_UART:
        /* socera 16550 Compatible Soft UARTs have a configurable size and is */
        /* stored in the CPR::FIFO_Mode (Component Parameter Register :: FIFO Depth). */
        *size = soc_socERA_16550_CPR_FIFO_MODE_GET(soc_read_word(soc_socERA_16550_CPR_ADDR(handle->location))) << 4;
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_size_get_tx(soc_16550_HANDLE_t * handle,
                                           uint32_t * size)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
        /* Read the CPR::FIFO_Mod (Component Parameter Register :: FIFO Mode). */
        /* The FIFO size is 16x this value. */
        *size = soc_UART_CPR_FIFO_MOD_GET(soc_read_word(soc_UART_CPR_ADDR(handle->location))) << 4;
        break;
    case soc_16550_DEVICE_socERA_16550_UART:
        /* socera 16550 Compatible Soft UARTs have a configurable size and is */
        /* stored in the CPR::FIFO_Mode (Component Parameter Register :: FIFO Depth). */
        /* The FIFO size is 16x this value. */
        *size = soc_socERA_16550_CPR_FIFO_MODE_GET(soc_read_word(soc_socERA_16550_CPR_ADDR(handle->location))) << 4;
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_level_get_rx(soc_16550_HANDLE_t * handle,
                                            uint32_t * level)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
        /* Read RFL (Receive FIFO Level). */
        *level = soc_read_word(soc_UART_RFL_ADDR(handle->location));
        break;
    case soc_16550_DEVICE_socERA_16550_UART:
        /* RFL not implemented. Return 0. */
        *level = 0;
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_level_get_tx(soc_16550_HANDLE_t * handle,
                                            uint32_t * level)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
        /* Read TFL (Transmit FIFO Level). */
        *level = soc_read_word(soc_UART_TFL_ADDR(handle->location));
        break;
    case soc_16550_DEVICE_socERA_16550_UART:
        /* TFL not implemented. Return 0. */
        *level = 0;
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_trigger_set_rx(soc_16550_HANDLE_t * handle,
                                              soc_16550_FIFO_TRIGGER_RX_t trigger)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    /* Verify triggering parameter */
    switch (trigger)
    {
    case soc_16550_FIFO_TRIGGER_RX_ANY:
    case soc_16550_FIFO_TRIGGER_RX_QUARTER_FULL:
    case soc_16550_FIFO_TRIGGER_RX_HALF_FULL:
    case soc_16550_FIFO_TRIGGER_RX_ALMOST_FULL:
        break;
    default:
        return soc_E_BAD_ARG;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Update FCR::RT (FIFO Control Register :: Receiver Trigger) */
        handle->fcr &= ~soc_UART_FCR_RT_SET_MSK;
        handle->fcr |= soc_UART_FCR_RT_SET(trigger);
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_fifo_trigger_set_tx(soc_16550_HANDLE_t * handle,
                                              soc_16550_FIFO_TRIGGER_TX_t trigger)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    /* Verify triggering parameter */
    switch (trigger)
    {
    case soc_16550_FIFO_TRIGGER_TX_EMPTY:
    case soc_16550_FIFO_TRIGGER_TX_ALMOST_EMPTY:
    case soc_16550_FIFO_TRIGGER_TX_QUARTER_FULL:
    case soc_16550_FIFO_TRIGGER_TX_HALF_FULL:
        break;
    default:
        return soc_E_BAD_ARG;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Update FCR::TET (FIFO Control Register :: Transmit Empty Trigger) */
        handle->fcr &= ~soc_UART_FCR_TET_SET_MSK;
        handle->fcr |= soc_UART_FCR_TET_SET(trigger);
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_baudrate_get(soc_16550_HANDLE_t * handle,
                                       uint32_t * baudrate)
{
    /* Query the divisor cached in the handle data */
    uint32_t divisor = soc_16550_HANDLE_DATA_DIVISOR_VALUE_GET(handle->data);

    /* The divisor should never be zero. It is set to allow for a baud of 57600
    // on initialization and a valid value is checked at
    // soc_16550_divisor_set(). We do not check for users socering the data in
    // the handle structure.

    // Formula for calculating the baudrate:
    //    baudrate = clock / (16 * divisor) */

    *baudrate = (handle->clock_freq >> 4) / divisor;

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_baudrate_set(soc_16550_HANDLE_t * handle,
                                       uint32_t baudrate)
{
    uint32_t divisor;
    if (baudrate == 0)
    {
        return soc_E_ARG_RANGE;
    }

    /* Formula for calculating the divisor:
    //    baudrate = clock / (16 * divisor)
    // => baudrate * 16 * divisor = clock
    // => divisor = clock / (baudrate * 16)
    // => divisor = (clock / 16) / baudrate */

    /* Add half of the denominator to address rounding errors. */
    divisor = ((handle->clock_freq + (8 * baudrate)) / (16 * baudrate));

    /* Check for divisor range is in soc_16550_divisor_set(). */
    return soc_16550_divisor_set(handle, divisor);
}

soc_STATUS_CODE soc_16550_divisor_get(soc_16550_HANDLE_t * handle,
                                      uint32_t * divisor)
{
    /* Just read the divisor portion of the handle data. */
    *divisor = soc_16550_HANDLE_DATA_DIVISOR_VALUE_GET(handle->data);

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_divisor_set(soc_16550_HANDLE_t * handle,
                                      uint32_t divisor)
{
    /* Verify divisor value is in range. */
    if ((divisor > 0xffff) || (divisor == 0))
    {
        return soc_E_ARG_RANGE;
    }

    /* Set the divisor portion of the handle data. */
    handle->data &= ~(0xffff);
    handle->data |= divisor;

    /* Even if the UART is enabled, don't do anything. It is documented that */
    /* the change will take effect when the UART move to the enabled state. */

    return soc_E_SUCCESS;
}

static soc_STATUS_CODE soc_16550_ier_mask_set_helper(soc_16550_HANDLE_t * handle, uint32_t setmask)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Set bit in IER (Interrupt Enable Register) */
        soc_setbits_word(soc_UART_IER_DLH_ADDR(handle->location), setmask);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

static soc_STATUS_CODE soc_16550_ier_mask_clr_helper(soc_16550_HANDLE_t * handle, uint32_t setmask)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Clear bit in IER (Interrupt Enable Register) */
        soc_clrbits_word(soc_UART_IER_DLH_ADDR(handle->location), setmask);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_int_enable_rx(soc_16550_HANDLE_t * handle)
{
    /* Set the IER::ERBFI (Interrupt Enable Register :: Enable Receive Buffer Full Interrupt) bit. */
    return soc_16550_ier_mask_set_helper(handle, soc_UART_IER_DLH_ERBFI_DLH0_SET_MSK);
}

soc_STATUS_CODE soc_16550_int_disable_rx(soc_16550_HANDLE_t * handle)
{
    /* Clear the IER::ERBFI (Interrupt Enable Register :: Enable Receive Buffer Full Interrupt) bit. */
    return soc_16550_ier_mask_clr_helper(handle, soc_UART_IER_DLH_ERBFI_DLH0_SET_MSK);
}

soc_STATUS_CODE soc_16550_int_enable_tx(soc_16550_HANDLE_t * handle)
{
    /* Set the IER::ETBEI (Interrupt Enable Register :: Enable Transmit Buffer Empty Interrupt) bit. */
    return soc_16550_ier_mask_set_helper(handle, soc_UART_IER_DLH_ETBEI_DLH1_SET_MSK);
}

soc_STATUS_CODE soc_16550_int_disable_tx(soc_16550_HANDLE_t * handle)
{
    /* Clear the IER::ETBEI (Interrupt Enable Register :: Enable Transmit Buffer Empty Interrupt) bit. */
    return soc_16550_ier_mask_clr_helper(handle, soc_UART_IER_DLH_ETBEI_DLH1_SET_MSK);
}

soc_STATUS_CODE soc_16550_int_enable_line(soc_16550_HANDLE_t * handle)
{
    /* Set the IER::ELSI (Interrupt Enable Register :: Enable Line Status Interrupt) bit. */
    return soc_16550_ier_mask_set_helper(handle, soc_UART_IER_DLH_ELSI_DHL2_SET_MSK);
}

soc_STATUS_CODE soc_16550_int_disable_line(soc_16550_HANDLE_t * handle)
{
    /* Clear the IER::ELSI (Interrupt Enable Register :: Enable Line Status Interrupt) bit. */
    return soc_16550_ier_mask_clr_helper(handle, soc_UART_IER_DLH_ELSI_DHL2_SET_MSK);
}

soc_STATUS_CODE soc_16550_int_enable_modem(soc_16550_HANDLE_t * handle)
{
    /* Set the IER::EDSSI (Interrupt Enable Register :: Enable Modem Status Interrupt) bit. */
    return soc_16550_ier_mask_set_helper(handle, soc_UART_IER_DLH_EDSSI_DHL3_SET_MSK);
}

soc_STATUS_CODE soc_16550_int_disable_modem(soc_16550_HANDLE_t * handle)
{
    /* Clear the IER::EDSSI (Interrupt Enable Register :: Enable Modem Status Interrupt) bit. */
    return soc_16550_ier_mask_clr_helper(handle, soc_UART_IER_DLH_EDSSI_DHL3_SET_MSK);
}

soc_STATUS_CODE soc_16550_int_disable_all(soc_16550_HANDLE_t * handle)
{
    /* Clear the IER::(ERBFI | ETBEI | ELSI | EDSSI)
    //   (Interrupt Enable Register :: (Enable Receive Buffer Full Interrupt   |
    //                                  Enable Transmit Buffer Empty Interrupt |
    //                                  Enable Line Status Interrupt           |
    //                                  Enable Modem Status Interrupt)) bits   */
    return soc_16550_ier_mask_clr_helper(handle, soc_UART_IER_DLH_ERBFI_DLH0_SET_MSK |
                                                 soc_UART_IER_DLH_ETBEI_DLH1_SET_MSK |
                                                 soc_UART_IER_DLH_ELSI_DHL2_SET_MSK  |
                                                 soc_UART_IER_DLH_EDSSI_DHL3_SET_MSK);
}

soc_STATUS_CODE soc_16550_int_status_get(soc_16550_HANDLE_t * handle,
                                         soc_16550_INT_STATUS_t * status)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Read IIR::IID (Interrupt Identity Register :: Interrupt ID) */
        *status = (soc_16550_INT_STATUS_t) soc_UART_IIR_ID_GET(soc_read_word(soc_UART_IIR_ADDR(handle->location)));
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_line_config_set(soc_16550_HANDLE_t * handle,
                                          soc_16550_DATABITS_t databits,
                                          soc_16550_PARITY_t parity,
                                          soc_16550_STOPBITS_t stopbits)
{
    /* LCR (Line Control Register) cache. */
    uint32_t lcr = 0;

    /* Validate the databits parameter. */
    switch (databits)
    {
    case soc_16550_DATABITS_5:
    case soc_16550_DATABITS_6:
    case soc_16550_DATABITS_7:
    case soc_16550_DATABITS_8:
        break;
    default:
        return soc_E_ERROR;
    }

    /* Validate the parity parameter. */
    switch (parity)
    {
    case soc_16550_PARITY_DISABLE:
    case soc_16550_PARITY_ODD:
    case soc_16550_PARITY_EVEN:
        break;
    default:
        return soc_E_ERROR;
    }

    /* Validate the stopbits parameter. */
    switch (stopbits)
    {
    case soc_16550_STOPBITS_1:
    case soc_16550_STOPBITS_2:
        break;
    default:
        return soc_E_ERROR;
    }

    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:

        /* Configure the number of databits */
        lcr |= soc_UART_LCR_DLS_SET(databits);

        /* Configure the number of stopbits */
        lcr |= soc_UART_LCR_STOP_SET(stopbits);

        /* Configure the parity */
        if (parity != soc_16550_PARITY_DISABLE)
        {
            /* Enable parity in LCR */
            lcr |= soc_UART_LCR_PEN_SET_MSK;

            if (parity == soc_16550_PARITY_EVEN)
            {
                /* Enable even parity in LCR; otherwise it's odd parity. */
                lcr |= soc_UART_LCR_EPS_SET_MSK;
            }
        }

        /* Update LCR (Line Control Register) */
        soc_replbits_word(soc_UART_LCR_ADDR(handle->location),
                          soc_UART_LCR_DLS_SET_MSK
                        | soc_UART_LCR_STOP_SET_MSK
                        | soc_UART_LCR_PEN_SET_MSK
                        | soc_UART_LCR_EPS_SET_MSK,
                        lcr);

        break;

    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_line_break_enable(soc_16550_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Set the LCR::Break (Line Control Register :: Break) bit. */
        soc_setbits_word(soc_UART_LCR_ADDR(handle->location), soc_UART_LCR_BREAK_SET_MSK);
        break;

    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_line_break_disable(soc_16550_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Clear the LCR::Break (Line Control Register :: Break) bit. */
        soc_clrbits_word(soc_UART_LCR_ADDR(handle->location), soc_UART_LCR_BREAK_SET_MSK);
        break;

    default:
        return soc_E_ERROR;
    }


    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_line_status_get(soc_16550_HANDLE_t * handle,
                                          uint32_t * status)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Read the LSR (Line Status Register). */
        *status = soc_read_word(soc_UART_LSR_ADDR(handle->location));
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

static soc_STATUS_CODE soc_16550_mcr_mask_set_helper(soc_16550_HANDLE_t * handle,
                                                     uint32_t setmask)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Set the bit in MCR (Modem Control Register). */
        soc_setbits_word(soc_UART_MCR_ADDR(handle->location), setmask);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

static soc_STATUS_CODE soc_16550_mcr_mask_clr_helper(soc_16550_HANDLE_t * handle, uint32_t setmask)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Clear the bit in MCR (Modem Control Register). */
        soc_clrbits_word(soc_UART_MCR_ADDR(handle->location), setmask);
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}

soc_STATUS_CODE soc_16550_flowcontrol_enable(soc_16550_HANDLE_t * handle)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & soc_UART_FCR_FIFOE_SET_MSK))
    {
        return soc_E_ERROR;
    }

    /* For the socera 16550 Compatible Soft UART, check that Hardware Flowcontrol is enabled. */
    if (handle->device == soc_16550_DEVICE_socERA_16550_UART)
    {
        /* Read the CPR::AFCE_Mode (Component Parameter Register :: Auto Flow Control mode) bit. */
        uint32_t cpr = soc_read_word(soc_socERA_16550_CPR_ADDR(handle->location));
        if (!(soc_socERA_16550_CPR_AFCE_MODE_SET_MSK & cpr))
        {
            return soc_E_ERROR;
        }
    }

    /* Set MCR::AFCE (Modem Control Register :: Automatic FlowControl Enable) bit. */
    return soc_16550_mcr_mask_set_helper(handle, soc_UART_MCR_AFCE_SET_MSK);
}

soc_STATUS_CODE soc_16550_flowcontrol_disable(soc_16550_HANDLE_t * handle)
{
    /* Clear MCR::AFCE (Modem Control Register :: Automatic FlowControl Enable) bit. */
    return soc_16550_mcr_mask_clr_helper(handle, soc_UART_MCR_AFCE_SET_MSK);
}

soc_STATUS_CODE soc_16550_loopback_enable(soc_16550_HANDLE_t * handle)
{
    /* Loopback is not implemented in the socera 16550 Compatible Soft UART. */
    if (handle->device == soc_16550_DEVICE_socERA_16550_UART)
    {
        return soc_E_ERROR;
    }

    /* Set MCR::Loopback (Modem Control Register :: Loopback) bit. */
    return soc_16550_mcr_mask_set_helper(handle, soc_UART_MCR_LOOPBACK_SET_MSK);
}

soc_STATUS_CODE soc_16550_loopback_disable(soc_16550_HANDLE_t * handle)
{
    /* Clear MCR::Loopback (Modem Control Register :: Loopback) bit. */
    return soc_16550_mcr_mask_clr_helper(handle, soc_UART_MCR_LOOPBACK_SET_MSK);
}

soc_STATUS_CODE soc_16550_modem_enable_out1(soc_16550_HANDLE_t * handle)
{
    /* Set MCR::Out1 (Modem Control Register :: Out1) bit. */
    return soc_16550_mcr_mask_set_helper(handle, soc_UART_MCR_OUT1_SET_MSK);
}

soc_STATUS_CODE soc_16550_modem_disable_out1(soc_16550_HANDLE_t * handle)
{
    /* Clear MCR::Out1 (Modem Control Register :: Out1) bit. */
    return soc_16550_mcr_mask_clr_helper(handle, soc_UART_MCR_OUT1_SET_MSK);
}

soc_STATUS_CODE soc_16550_modem_enable_out2(soc_16550_HANDLE_t * handle)
{
    /* Set MCR::Out2 (Modem Control Register :: Out2) bit. */
    return soc_16550_mcr_mask_set_helper(handle, soc_UART_MCR_OUT2_SET_MSK);
}

soc_STATUS_CODE soc_16550_modem_disable_out2(soc_16550_HANDLE_t * handle)
{
    /* Clear MCR::Out2 (Modem Control Register :: Out2) bit. */
    return soc_16550_mcr_mask_clr_helper(handle, soc_UART_MCR_OUT2_SET_MSK);
}

soc_STATUS_CODE soc_16550_modem_enable_rts(soc_16550_HANDLE_t * handle)
{
    /* Set MCR::RTS (Modem Control Register :: Request To Send) bit. */
    return soc_16550_mcr_mask_set_helper(handle, soc_UART_MCR_RTS_SET_MSK);
}

soc_STATUS_CODE soc_16550_modem_disable_rts(soc_16550_HANDLE_t * handle)
{
    /* Clear MCR::RTS (Modem Control Register :: Request To Send) bit. */
    return soc_16550_mcr_mask_clr_helper(handle, soc_UART_MCR_RTS_SET_MSK);
}

soc_STATUS_CODE soc_16550_modem_enable_dtr(soc_16550_HANDLE_t * handle)
{
    /* Set MCR::DTR (Modem Control Register :: Data Terminal Ready) bit. */
    return soc_16550_mcr_mask_set_helper(handle, soc_UART_MCR_DTR_SET_MSK);
}

soc_STATUS_CODE soc_16550_modem_disable_dtr(soc_16550_HANDLE_t * handle)
{
    /* Clear MCR::DTR (Modem Control Register :: Data Terminal Ready) bit. */
    return soc_16550_mcr_mask_clr_helper(handle, soc_UART_MCR_DTR_SET_MSK);
}

soc_STATUS_CODE soc_16550_modem_status_get(soc_16550_HANDLE_t * handle,
                                          uint32_t * status)
{
    switch (handle->device)
    {
    case soc_16550_DEVICE_SOCFPGA_UART0:
    case soc_16550_DEVICE_SOCFPGA_UART1:
    case soc_16550_DEVICE_socERA_16550_UART:
        /* Read the MSR (Modem Status Register). */
        *status = soc_read_word(soc_UART_MSR_ADDR(handle->location));
        break;
    default:
        return soc_E_ERROR;
    }

    return soc_E_SUCCESS;
}
