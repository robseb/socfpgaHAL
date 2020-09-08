

#include "socfpgaHAL.h"
#include "soc_uart.h"

#include "../inc/soc_resetManager.h"
#include "../inc/soc_clockManager.h"
#include "soc_regacc.h"

//#include "soc_clock_manager.h"
//#include "socal/soc_rstmgr.h"
//#include "socal/soc_uart.h"
//#include "socal/hps.h"
//#include "socal/socal.h"

#define DEFAULT_BAUD SOC_UART_BAUDRATE_115200


/*
 *
 * Macros
 *
 */
#define SOC_UART_HANDLE_DATA_UART_ENABLED_MSK   (1UL << 31)
#define SOC_UART_HANDLE_DATA_DIVISOR_VALUE_GET(value) (value & 0xffff)

#define SOC_UART_CPR_OFST        (0xF4)
#define SOC_UART_CPR_ADDR(base)  SOC_UART_CAST(void *, (SOC_UART_CAST(char *, (base)) + 0xf4))
#define SOC_UART_CPR_FIFO_MODE_GET(value) ((((uint32_t) value) >> 16) & 0xff)
#define SOC_UART_CPR_AFCE_MODE_SET_MSK (1 << 4)

/*  Remove these macros as part of case:123835.*/
#define SOC_UART_IER_DLH_VALUE_SET(value) ((value) & 0xff)
#define SOC_UART_IER_DLH_ETBEI_DLH1_SET_MSK SOC_UART_IER_DLH_ETBEI_DLHL_SET_MSK


////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *
 *  HELPER FUNCTIONS
 *
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////


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
static soc_status_t soc_uart_write_divisor_helper(SOC_UART_HANDLE_t * handle,
                                                      uint32_t divisor)
{
    /* Validate the divisor parameter. */
    if (divisor > 0xffff)
    {
        /* This should never happen as it is verified in divisor_set. */
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Set LCR::DLAB (Line Control Register :: Divisor Latch Access Bit) */
        soc_setbits_word(SOC_UART_LCR_ADDR(handle->location), SOC_UART_LCR_DLAB_SET_MSK);

        /* Write DLL (Divisor Latch Low). */
        soc_write_word(SOC_UART_RBR_THR_DLL_ADDR(handle->location), SOC_UART_RBR_THR_DLL_VALUE_SET(divisor));

        /* Write DLH (Divisor Latch High). */
        soc_write_word(SOC_UART_IER_DLH_ADDR(handle->location), SOC_UART_IER_DLH_VALUE_SET(divisor >> 8));

        /* Clear LCR::DLAB (Line Control Register :: Divisor Latch Access Bit) */
        soc_clrbits_word(SOC_UART_LCR_ADDR(handle->location), SOC_UART_LCR_DLAB_SET_MSK);

        break;

    default:
        return SOC_S_GENERAL_ERROR;
    }

    /* Update the enabled state in the handle data. */
    if (divisor != 0)
    {
        handle->data |= SOC_UART_HANDLE_DATA_UART_ENABLED_MSK;
    }
    else
    {
        handle->data &= ~SOC_UART_HANDLE_DATA_UART_ENABLED_MSK;
    }

    return SOC_S_SUCCESSFULLY;
}

/*
// Helper function to reset the UART.
*/
static soc_status_t soc_uart_reset_helper(SOC_UART_HANDLE_t * handle, bool enable_init)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
        /* Write SRR::UR (Shadow Reset Register :: UART Reset) */
        soc_write_word(SOC_UART_SRR_ADDR(handle->location), SOC_UART_SRR_UR_SET_MSK);

        /* Read the MSR to work around case:119085. */
        soc_read_word_helper(soc_UART_MSR_ADDR(handle->location));
        break;

    case SOC_UART_DEVICE_INTEL_IP:
        soc_uart_write_divisor_helper(handle, 0); /* Disable UART */
        soc_uart_int_disable_all(handle);         /* Disable interrupts */
        soc_uart_fifo_disable(handle);            /* Disable FIFOs */
        soc_write_word(SOC_UART_MCR_ADDR(handle->location), 0); /* 0 -> MCR (AFCE, LP, OUT2, OUT1, RTS, DTR) */
        break;

    default:
        return SOC_S_GENERAL_ERROR;
    }

    /* If we are initializing (as opposed to just uninitializing) */
    if (enable_init)
    {
        soc_status_t status;
        uint32_t divisor;

        /* Set bit IER::PTIME (Interrupt Enable Register :: Programmable THRE Mode Enable) */
        soc_setbits_word(soc_UART_IER_DLH_ADDR(handle->location), SOC_UART_IER_DLH_PTIME_DLH7_SET_MSK);

        /* Set the line configuration to use 8-N-1. */
        status = soc_uart_line_config_set(handle, SOC_UART_DATABITS_8,
                                                   SOC_UART_PARITY_DISABLE,
                                                   SOC_UART_STOPBITS_1);
        if (status != SOC_S_SUCCESSFULLY)
        {
            return status;
        }

        divisor = SOC_UART_HANDLE_DATA_DIVISOR_VALUE_GET(handle->data);
        if (divisor == 0)
        {
            status = soc_uart_baudrate_set(handle, DEFAULT_BAUD);
            if (status != SOC_S_SUCCESSFULLY)
            {
                return status;
            }
        }
    }

    return SOC_S_SUCCESSFULLY;
}


static soc_status_t soc_uart_ier_mask_set_helper(SOC_UART_HANDLE_t * handle, uint32_t setmask)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Set bit in IER (Interrupt Enable Register) */
        soc_setbits_word(soc_UART_IER_DLH_ADDR(handle->location), setmask);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

static soc_status_t soc_uart_ier_mask_clr_helper(SOC_UART_HANDLE_t * handle, uint32_t setmask)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Clear bit in IER (Interrupt Enable Register) */
        soc_clrbits_word(soc_UART_IER_DLH_ADDR(handle->location), setmask);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

static soc_status_t soc_uart_mcr_mask_set_helper(SOC_UART_HANDLE_t * handle,
                                                     uint32_t setmask)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Set the bit in MCR (Modem Control Register). */
        soc_setbits_word(SOC_UART_MCR_ADDR(handle->location), setmask);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

static soc_status_t soc_uart_mcr_mask_clr_helper(SOC_UART_HANDLE_t * handle, uint32_t setmask)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Clear the bit in MCR (Modem Control Register). */
        soc_clrbits_word(SOC_UART_MCR_ADDR(handle->location), setmask);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}



////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 *
 *  MAIN FUNCTIONS
 *
 */
////////////////////////////////////////////////////////////////////////////////////////////////////////////



/*!
 * Performs the initialization steps needed by the UART. This should be the
 * first API call made when accessing a particular UART
 *
 * The default UART setting is 8 databits, no parity, 1 stopbit, and 57600
 * baud.
 *
 * For the SoCFPGA UARTs, The soc_CLK_L4_SP clock needs to be setup before
 * initialization.
 *
 * \param       device
 *              The UART device identifier.
 *
 * \param       location
 *              The memory of the location for the given UART. For SoCFPGA
 *              UARTs, this parameter is ignored.
 *
 * \param       clock_freq
 *              The clock frequency of the serial clock for the given UART.
 *              For SoCFPGA UARTs, this paramter is ignored.
 *
 * \param       handle
 *              [out] A pointer to a handle that will represent the UART. This
 *              handle should subsequently be used when calling other UART
 *              APIs.
 *
 * \retval      SOC_S_SUCCESSFULLY   The operation was successful.
 * \retval      SOC_S_GENERAL_ERROR     The operation failed.
 * \retval      SOC_S_WRONG_INPUT   The given UART device identifier is invalid.
 * \retval      SOC_S_BAD_CLOCK   The required clock is not yet setup.
 */
soc_status_t soc_uart_init(SOC_UART_DEVICE_t device,
                               void * location,
                               soc_freq_t clock_freq,
                               SOC_UART_HANDLE_t * handle)
{
    handle->device = device;
    handle->data   = 0;
    handle->fcr    = 0;

    switch (device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
        /* The soc_CLK_L4_SP is required for all SoCFPGA UARTs. Check that it's enabled. */
        if (soc_clk_is_enabled(SOC_CLK_L4_SP) != 1)
        {
            return SOC_S_BAD_CLOCK;
        }
        else
        {
            soc_status_t status;
            #ifdef SOC_CY_AV
                uint32_t ucr;
            #endif

            status = soc_clk_freq_get(SOC_CLK_L4_SP, &handle->clock_freq);
            if (status != SOC_S_SUCCESSFULLY)
            {
                return status;
            }

            if (device == SOC_UART_DEVICE_SOCFPGA_UART0)
            {
                handle->location = SOC_UART0_BASE;

                /* Bring UART0 out of reset. */
#ifdef SOC_CY_AV
                soc_clrbits_word(SOC_RSTMGR_PERMODRST_ADDR, SOC_RSTMGR_PERMODRST_UART0_SET_MSK);
#else
                soc_clrbits_word(SOC_RSTMGR_PER1MODRST_ADDR, SOC_RSTMGR_PER1MODRST_UART0_SET_MSK);
#endif
            }
            else /* device == SOC_UART_DEVICE_SOCFPGA_UART1 */
            {
                handle->location = SOC_UART1_BASE;

                /* Bring UART1 out of reset. */
#ifdef SOC_CY_AV
                soc_clrbits_word(SOC_RSTMGR_PERMODRST_ADDR, SOC_RSTMGR_PERMODRST_UART1_SET_MSK);
#else
                soc_clrbits_word(SOC_RSTMGR_PER1MODRST_ADDR, SOC_RSTMGR_PER1MODRST_UART1_SET_MSK);
#endif
            }
#ifdef SOC_CY_AV
            /* Verify the UCR (UART Component Version) */
            ucr = soc_read_word(soc_UART_UCV_ADDR(handle->location));
            if (ucr != SOC_UART_UCV_UART_COMPONENT_VER_RESET)
            {
                return SOC_S_GENERAL_ERROR;
            }
#endif
        }
        break;
    case SOC_UART_DEVICE_INTEL_IP:
        handle->location   = location;
        handle->clock_freq = clock_freq;
        break;
    default:
        return SOC_S_WRONG_INPUT;
    }

    return soc_uart_reset_helper(handle, true);
}

/*!
 * Performs the uninitialization steps for the UART. This should be the last
 * API call made to cleanup the UART.
 *
 * After calling this function, the handle will need to be initialized again
 * before being used by calling soc_uart_init().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      SOC_S_WRONG_INPUT   The given UART device handle is invalid.
 */
soc_status_t soc_uart_uninit(SOC_UART_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
#ifdef SOC_CY_AV
        soc_setbits_word(SOC_RSTMGR_PERMODRST_ADDR, SOC_RSTMGR_PERMODRST_UART0_SET_MSK);
#else
        soc_setbits_word(SOC_RSTMGR_PER1MODRST_ADDR, SOC_RSTMGR_PER1MODRST_UART0_SET_MSK);
#endif
        return SOC_S_SUCCESSFULLY;
    case SOC_UART_DEVICE_SOCFPGA_UART1:
#ifdef SOC_CY_AV
        soc_setbits_word(SOC_RSTMGR_PERMODRST_ADDR, SOC_RSTMGR_PERMODRST_UART1_SET_MSK);
#else
        soc_setbits_word(SOC_RSTMGR_PER1MODRST_ADDR, SOC_RSTMGR_PER1MODRST_UART1_SET_MSK);
#endif
        return SOC_S_SUCCESSFULLY;
    case SOC_UART_DEVICE_INTEL_IP:
    default:
        return soc_uart_reset_helper(handle, false);
    }
}

/*!
 * Resets the UART to the default configuration. The UART will be reset and
 * reinitialized.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_reset(SOC_UART_HANDLE_t * handle)
{
    return soc_uart_reset_helper(handle, true);
}

/*!
 * Starts the UART after all configuration has been completed.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_enable(SOC_UART_HANDLE_t * handle)
{
    /* Enable Terminal*/
  soc_uart_modem_enable_dtr(handle);
  soc_uart_modem_enable_rts(handle);

    /* Write the divisor cached in the handle data to the divisor registers. */
    /* This will effectively enable the UART. */
    return soc_uart_write_divisor_helper(handle,
           SOC_UART_HANDLE_DATA_DIVISOR_VALUE_GET(handle->data));

}

/*!
 * Stops the UART. While UART configuration can be done while enabled, it is
 * not recommended.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_disable(SOC_UART_HANDLE_t * handle)
{
    /* Write 0 to the divisor the divisor registers. This will effectively */
    /* disable the UART. */
    return soc_uart_write_divisor_helper(handle, 0);
}


/*!
 * Reads a single character from the UART receiver buffer. This API should
 * only be used when FIFOs are disabled.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       item
 *              [out] Pointer to an output parameter that contains the in
 *              receiver buffer of the UART.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_read(SOC_UART_HANDLE_t * handle,
                               char * item)
{
    /* Verify that the UART is enabled */
    if (!(handle->data & SOC_UART_HANDLE_DATA_UART_ENABLED_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    /* Verify that the FIFO is disabled */
    if (handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK)
    {
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Read the RBR (Receive Buffer Register) into *item. */
        *item = soc_UART_RBR_THR_DLL_VALUE_GET(soc_read_word(soc_UART_RBR_THR_DLL_ADDR(handle->location)));
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }
    return SOC_S_SUCCESSFULLY;
}

/*!
 * Writes a single character to the UART transmitter buffer. This API should
 * only be used when FIFOs are disabled.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       item
 *              The character to write to the transmitter buffer of the UART.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_write(SOC_UART_HANDLE_t * handle,
                                char item)
{
    /* Verify that the UART is enabled */
    if (!(handle->data & SOC_UART_HANDLE_DATA_UART_ENABLED_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    /* Verify that the FIFO is disabled */
    if (handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK)
    {
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Write the buffer into the THR (Transmit Holding Register) */
        soc_write_word(soc_UART_RBR_THR_DLL_ADDR(handle->location), item);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Enables FIFO on the UART. This will enable both the receiver FIFO and
 * transmitter FIFO. Both FIFOs will be cleared.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_enable(SOC_UART_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Set FCR::FIFOE (FIFO Control Register :: FIFO Enable) bit. */
        handle->fcr |= SOC_UART_FCR_FIFOE_SET_MSK | SOC_UART_FCR_RFIFOR_SET_MSK | SOC_UART_FCR_XFIFOR_SET_MSK;
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    /* No need to reset / clear the FIFOs. This is done automatically when */
    /* FCR::FIFOE is changed. */
    return SOC_S_SUCCESSFULLY;
}

/*!
 * Disables FIFOs on the UART. This will disable both the receiver FIFO and
 * transmitter FIFO. Any data left in the FIFOs will be lost.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_disable(SOC_UART_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Clear FCR::FIFOE (FIFO Control Register :: FIFO Enable) bit. */
        handle->fcr &= ~SOC_UART_FCR_FIFOE_SET_MSK;
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Reads the given buffer from the receiver FIFO in the UART.
 *
 * The available characters in the FIFO can be determined by a few ways. Users
 * can determine the number of items by calling soc_uart_fifo_level_get_rx().
 *
 * Another way is by using the RX trigger and RX interrupt. First determine the
 * RX FIFO size by calling soc_uart_fifo_size_get_rx(). Then set the desired
 * trigger level by calling soc_uart_fifo_trigger_set_rx(). Calculate the
 * triggering point by applying trigger description on the FIFO size. Enable RX
 * interrupts by calling soc_uart_int_enable_rx(). When the RX interrupt fires
 * due to the SOC_UART_INT_STATUS_RX_DATA condition, the calculated triggering
 * point value can be used to determine the RX FIFO level. If the interrupt
 * fires due to the SOC_UART_INT_STATUS_RX_TIMEOUT, the RX FIFO can be
 * completely emptied by repeatedly polling the Line Status
 * SOC_UART_LINE_STATUS_DR condition by calling soc_uart_line_status_get().
 * These steps are necessary if the UART does not implement FIFO level query
 * functionality. As of 13.0sp1, this applies to the socera 16550 Compatible
 * Soft UART.
 *
 * Reading more data than that which is available can result in invalid data
 * appearing like valid data.
 *
 * The FIFO must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       buffer
 *              [out] Pointer to a buffer where the specified count of
 *              characters from the receiver FIFO will be copied to.
 *
 * \param       count
 *              The count of characters from the receiver FIFO to be copied.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_read(SOC_UART_HANDLE_t * handle,
                                    char * buffer,
                                    size_t count)
{
    size_t i;
    /* Verify that the UART is enabled */
    if (!(handle->data & SOC_UART_HANDLE_DATA_UART_ENABLED_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Read the RBR (Receive Buffer Register) into the buffer */
        for (i = 0; i < count; ++i)
        {
            buffer[i] = soc_UART_RBR_THR_DLL_VALUE_GET(soc_read_word(soc_UART_RBR_THR_DLL_ADDR(handle->location)));
        }
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Writes the given buffer to the transmitter FIFO in the UART.
 *
 * The available space in the FIFO can be determined by a few ways. Users can
 * determine the number of items by calculating the FIFO capacity minus the
 * FIFO level. This can be done by calling  soc_uart_fifo_size_get_tx() and
 * soc_uart_fifo_level_get_tx() respectively.
 *
 * Another way is by using the TX trigger and TX interrupt. First determine the
 * TX FIFO size by calling soc_uart_fifo_size_get_tx(). The set the desired
 * trigger level by calling soc_uart_fifo_trigger_set_tx(). Calculate the
 * triggering point by applying the trigger description on the FIFO size.
 * Enable TX interrupts by calling soc_uart_int_enable_tx(). When the TX
 * interrupt fires, calculate the empty entries in the FIFO by subtracting the
 * TX FIFO size and the calculated value. These steps are necessary if the UART
 * does not implement FIFO level query functionality. As of 13.0sp1, this
 * applies to the socera 16550 Compatible Soft UART.
 *
 * Writing more data that there is space can result in data lost due to
 * overflowing.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       buffer
 *              Pointer to a buffer from where the specified count of
 *              characters will be copied to the transmitter FIFO.
 *
 * \param       count
 *              The count of characters from the given buffer to be copied.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_write_safe(SOC_UART_HANDLE_t * handle,
                                     const char * buffer,
                                     size_t count,
                                     bool safe)
{
    size_t i;
    /* Verify that the UART is enabled */
    if (!(handle->data & SOC_UART_HANDLE_DATA_UART_ENABLED_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
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
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Writes the given buffer to the transmitter FIFO in the UART, blocking
 * if the fifo is full until there is enough space to write the string
 *
 * Writing more data that there is space can result in data lost due to
 * overflowing.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       buffer
 *              Pointer to a buffer from where the specified count of
 *              characters will be copied to the transmitter FIFO.
 *
 * \param       count
 *              The count of characters from the given buffer to be copied.
 *
 * \param       safe
 *              true = block when the buffer is full until space is available
 *              false = do not block. This causes this function to act
 *                      in the same way as soc_uart_fifo_write()
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_write(SOC_UART_HANDLE_t * handle,
                                     const char * buffer,
                                     size_t count)
{
    return soc_uart_fifo_write_safe(handle, buffer, count, false);
}

/*!
 * Clears the contents of the receiver FIFO. Any characters which were
 * previously contained in that FIFO will be discarded.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_clear_rx(SOC_UART_HANDLE_t * handle)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
        /* Write SRR::RFR (Shadow Reset Register :: Receiver FIFO Reset) bit. */
        soc_write_word(soc_UART_SRR_ADDR(handle->location), SOC_UART_SRR_RFR_SET_MSK);
        break;
    case SOC_UART_DEVICE_INTEL_IP:
        /* Write FCR::RFIFOR (FIFO Control Register :: Receiver FIFO Reset) bit. */
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr | SOC_UART_FCR_RFIFOR_SET_MSK);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Clears the contents of the transmitter FIFO. Any characters which were
 * previously contained in that FIFO will be discarded.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_clear_tx(SOC_UART_HANDLE_t * handle)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
        /* Write SRR::XFR (Shadow Reset Register :: Xmitter FIFO Reset) bit. */
        soc_write_word(soc_UART_SRR_ADDR(handle->location), SOC_UART_SRR_XFR_SET_MSK);
        break;
    case SOC_UART_DEVICE_INTEL_IP:
        /* Write FCR::XFIFOR (FIFO Control Register :: Xmitter FIFO Reset) bit. */
        soc_write_word(soc_UART_FCR_ADDR(handle->location), handle->fcr | SOC_UART_FCR_XFIFOR_SET_MSK);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Clears the contents of the receiver and transmitter FIFO. Any characters
 * which were previously contained on those FIFOs will be discarded.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_clear_all(SOC_UART_HANDLE_t * handle)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
        /* Write SRR::(RFR | XFR) */
        /*   (Shadow Reset Register :: (Receiver FIFO Reset | Xmitter FIFO Reset)) bits. */
        soc_write_word(SOC_UART_SRR_ADDR(handle->location),
                       SOC_UART_SRR_RFR_SET_MSK | SOC_UART_SRR_XFR_SET_MSK);
        break;
    case SOC_UART_DEVICE_INTEL_IP:
        /* Write FCR::(RFIFOR |XFIFOR) */
        /*   (FIFO Control Register :: (Receiver FIFO Reset | Xmitter FIFO Reset)) bits. */
        soc_write_word(SOC_UART_FCR_ADDR(handle->location),
                       handle->fcr | SOC_UART_FCR_RFIFOR_SET_MSK | SOC_UART_FCR_XFIFOR_SET_MSK);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Queries the size of the receiver FIFO.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       size
 *              [out] Pointer to an output parameter that contains the size of
 *              the receiver FIFO.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_size_get_rx(SOC_UART_HANDLE_t * handle,
                                           uint32_t * size)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
        /* Read the CPR::FIFO_Mod (Component Parameter Register :: FIFO Mode). */
        /* The FIFO size is 16x this value. */
        *size = SOC_UART_CPR_FIFO_MOD_GET(soc_read_word(SOC_UART_CPR_ADDR(handle->location))) << 4;
        break;
    case SOC_UART_DEVICE_INTEL_IP:
        /* socera 16550 Compatible Soft UARTs have a configurable size and is */
        /* stored in the CPR::FIFO_Mode (Component Parameter Register :: FIFO Depth). */
        *size = SOC_UART_CPR_FIFO_MODE_GET(soc_read_word(SOC_UART_CPR_ADDR(handle->location))) << 4;
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}


/*!
 * Queries the size of the transmitter FIFO.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       size
 *              [out] Pointer to an output parameter that contains the size of
 *              the transmitter FIFO.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_size_get_tx(SOC_UART_HANDLE_t * handle,
                                           uint32_t * size)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
        /* Read the CPR::FIFO_Mod (Component Parameter Register :: FIFO Mode). */
        /* The FIFO size is 16x this value. */
        *size = SOC_UART_CPR_FIFO_MOD_GET(soc_read_word(\
        		soc_UART_CPR_ADDR(handle->location))) << 4;
        break;
    case SOC_UART_DEVICE_INTEL_IP:
        /* socera 16550 Compatible Soft UARTs have a configurable size and is */
        /* stored in the CPR::FIFO_Mode (Component Parameter Register :: FIFO Depth). */
        /* The FIFO size is 16x this value. */
        *size = SOC_UART_CPR_FIFO_MODE_GET(soc_read_word( \
        		SOC_UART_CPR_FIFO_MODE_GET(handle->location))) << 4;
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Queries the current level of the receiver FIFO.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * For the socera 16550 Compatible UART, it may not be possible to read the
 * FIFO level and this function may always report 0. For more information on
 * interacting with the FIFO in this situation, see documentation for
 * soc_uart_fifo_read().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       level
 *              [out] Pointer to an output parameter that contains the level
 *              or number of characters in the receiver FIFO.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_level_get_rx(SOC_UART_HANDLE_t * handle,
                                            uint32_t * level)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
        /* Read RFL (Receive FIFO Level). */
        *level = soc_read_word(SOC_UART_RFL_ADDR(handle->location));
        break;
    case SOC_UART_DEVICE_INTEL_IP:
        /* RFL not implemented. Return 0. */
        *level = 0;
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}


/*!
 * Queries the current level of the transmitter FIFO.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * For the socera 16550 Compatible UART, it may not be possible to read the
 * FIFO level and this function may always report 0. For more information on
 * interacting with the FIFO in this situation, see documentation for
 * soc_uart_fifo_write().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       level
 *              [out] Pointer to an output parameter that contains the level
 *              or number of characters in the transmitter FIFO.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_level_get_tx(SOC_UART_HANDLE_t * handle,
                                            uint32_t * level)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
        /* Read TFL (Transmit FIFO Level). */
        *level = soc_read_word(soc_UART_TFL_ADDR(handle->location));
        break;
    case SOC_UART_DEVICE_INTEL_IP:
        /* TFL not implemented. Return 0. */
        *level = 0;
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Sets the receiver FIFO level which will trigger the receiver FIFO to issue
 * receiver FIFO full event. For the list of available receiver FIFO trigger
 * levels, see the documentation for SOC_UART_FIFO_TRIGGER_RX_t.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       trigger
 *              The level of the receiver FIFO which is needed to trigger a
 *              receiver FIFO full event.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_trigger_set_rx(SOC_UART_HANDLE_t * handle,
                                              SOC_UART_FIFO_TRIGGER_RX_t trigger)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    /* Verify triggering parameter */
    switch (trigger)
    {
    case SOC_UART_FIFO_TRIGGER_RX_ANY:
    case SOC_UART_FIFO_TRIGGER_RX_QUARTER_FULL:
    case SOC_UART_FIFO_TRIGGER_RX_HALF_FULL:
    case SOC_UART_FIFO_TRIGGER_RX_ALMOST_FULL:
        break;
    default:
        return SOC_S_WRONG_INPUT;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Update FCR::RT (FIFO Control Register :: Receiver Trigger) */
        handle->fcr &= ~SOC_UART_FCR_RT_SET_MSK;
        handle->fcr |= SOC_UART_FCR_RT_SET(trigger);
        soc_write_word(SOC_UART_FCR_ADDR(handle->location), handle->fcr);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Sets the transmitter FIFO level which will trigger the transmitter FIFO to
 * transmitter FIFO empty event. For the list of available transmitter FIFO
 * trigger levels, see the documentation for SOC_UART_FIFO_TRIGGER_TX_t.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_uart_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       trigger
 *              The level of the transmitter FIFO which is needed to trigger a
 *              transmitter FIFO empty event.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_fifo_trigger_set_tx(SOC_UART_HANDLE_t * handle,
                                              SOC_UART_FIFO_TRIGGER_TX_t trigger)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    /* Verify triggering parameter */
    switch (trigger)
    {
    case SOC_UART_FIFO_TRIGGER_TX_EMPTY:
    case SOC_UART_FIFO_TRIGGER_TX_ALMOST_EMPTY:
    case SOC_UART_FIFO_TRIGGER_TX_QUARTER_FULL:
    case SOC_UART_FIFO_TRIGGER_TX_HALF_FULL:
        break;
    default:
        return SOC_S_WRONG_INPUT;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Update FCR::TET (FIFO Control Register :: Transmit Empty Trigger) */
        handle->fcr &= ~SOC_UART_FCR_TET_SET_MSK;
        handle->fcr |= SOC_UART_FCR_TET_SET(trigger);
        soc_write_word(SOC_UART_FCR_ADDR(handle->location), handle->fcr);
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Gets the baudrate for the UART.
 *
 * This is done by calculating the baudrate from the divisor and the serial
 * clock. The reported baudrate may not correspond exactly to the request
 * baudrate.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       baudrate
 *              [out] Pointer to an output paramter that contains the current
 *              baudrate of the UART.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_baudrate_get(SOC_UART_HANDLE_t * handle,
                                       uint32_t * baudrate)
{
    /* Query the divisor cached in the handle data */
    uint32_t divisor = SOC_UART_HANDLE_DATA_DIVISOR_VALUE_GET(handle->data);

    /* The divisor should never be zero. It is set to allow for a baud of 57600
    // on initialization and a valid value is checked at
    // soc_uart_divisor_set(). We do not check for users socering the data in
    // the handle structure.

    // Formula for calculating the baudrate:
    //    baudrate = clock / (16 * divisor) */

    *baudrate = (handle->clock_freq >> 4) / divisor;

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Sets the baudrate for the UART. This change will take effect when the UART
 * moves from disabled to enabled.
 *
 * This is done by calculating the correct divisor using the request baudrate
 * and the known serial clock.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       baudrate
 *              The requested baudrate for the UART.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 * \retval      SOC_S_WRONG_INPUT The given baudrate is not possible due to
 *                              limitations of the baudrate divisor and/or
 *                              serial clock.
 */
soc_status_t soc_uart_baudrate_set(SOC_UART_HANDLE_t * handle,
                                       uint32_t baudrate)
{
    uint32_t divisor;
    if (baudrate == 0)
    {
        return SOC_S_WRONG_INPUT;
    }

    /* Formula for calculating the divisor:
    //    baudrate = clock / (16 * divisor)
    // => baudrate * 16 * divisor = clock
    // => divisor = clock / (baudrate * 16)
    // => divisor = (clock / 16) / baudrate */

    /* Add half of the denominator to address rounding errors. */
    divisor = ((handle->clock_freq + (8 * baudrate)) / (16 * baudrate));

    /* Check for divisor range is in soc_uart_divisor_set(). */
    return soc_uart_divisor_set(handle, divisor);
}

/*!
 * Gets the baudrate divisor for the UART.
 *
 * The baudrate is determined by the following formula:
 *  * Baudrate = (serial clock frequency) / (16 * divisor)
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       divisor
 *              [out] Pointer to an output parameter that contains the current
 *              divisor used for baudrate generation.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_divisor_get(SOC_UART_HANDLE_t * handle,
                                      uint32_t * divisor)
{
    /* Just read the divisor portion of the handle data. */
    *divisor = SOC_UART_HANDLE_DATA_DIVISOR_VALUE_GET(handle->data);

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Sets the baudrate divisor for the UART. This change will take effect when
 * the UART moves from disabled to enabled.
 *
 * The baudrate is determined by the following formula:
 *  * Baudrate = (serial clock frequency) / (16 * divisor)
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       divisor
 *              The specified divisor value to use for baudrate generation.
 *              Valid values are 1 - 65535.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART identifier is invalid or the
 *                              specified divisor is not supported by the
 *                              UART.
 */
soc_status_t soc_uart_divisor_set(SOC_UART_HANDLE_t * handle,
                                      uint32_t divisor)
{
    /* Verify divisor value is in range. */
    if ((divisor > 0xffff) || (divisor == 0))
    {
        return SOC_S_WRONG_INPUT;
    }

    /* Set the divisor portion of the handle data. */
    handle->data &= ~(0xffff);
    handle->data |= divisor;

    /* Even if the UART is enabled, don't do anything. It is documented that */
    /* the change will take effect when the UART move to the enabled state. */

    return SOC_S_SUCCESSFULLY;
}


/*!
 * Enables the receiver FIFO to generate interrupts. Enabling this interrupt
 * allows for the following interrupt signal(s):
 *  * SOC_UART_INT_STATUS_RX_DATA
 *  * SOC_UART_INT_STATUS_RX_TIMEOUT
 *
 * This interrupt is disabled by default.
 *
 * The FIFOs must also be enabled for this interrupt to actually be generated.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_enable_rx(SOC_UART_HANDLE_t * handle)
{
    /* Set the IER::ERBFI (Interrupt Enable Register :: Enable Receive Buffer Full Interrupt) bit. */
    return soc_uart_ier_mask_set_helper(handle, SOC_UART_IER_DLH_ERBFI_DLH0_SET_MSK);
}


/*!
 * Disables the receiver FIFO from generating interrupts.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_disable_rx(SOC_UART_HANDLE_t * handle)
{
    /* Clear the IER::ERBFI (Interrupt Enable Register :: Enable Receive Buffer Full Interrupt) bit. */
    return soc_uart_ier_mask_clr_helper(handle, SOC_UART_IER_DLH_ERBFI_DLH0_SET_MSK);
}

/*!
 * Enables the transmitter FIFO to generate interrupts. Enabling this
 * interrupt allows for the following interrupt signal(s):
 *  * SOC_UART_INT_STATUS_TX_IDLE
 *
 * This interrupt is disabled by default.
 *
 * The FIFOs must also be enabled for this interrupt to actually be generated.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_enable_tx(SOC_UART_HANDLE_t * handle)
{
    /* Set the IER::ETBEI (Interrupt Enable Register :: Enable Transmit Buffer Empty Interrupt) bit. */
    return soc_uart_ier_mask_set_helper(handle, SOC_UART_IER_DLH_ETBEI_DLH1_SET_MSK);
}

/*!
 * Disables the transmitter FIFO from generating interrupts.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_disable_tx(SOC_UART_HANDLE_t * handle)
{
    /* Clear the IER::ETBEI (Interrupt Enable Register :: Enable Transmit Buffer Empty Interrupt) bit. */
    return soc_uart_ier_mask_clr_helper(handle, SOC_UART_IER_DLH_ETBEI_DLH1_SET_MSK);
}

/*!
 * Enables the receiver to generate line status interrupts. Enabling this
 * interrupt allows for the following interrupt signal(s):
 *  * SOC_UART_INT_STATUS_LINE
 *
 * This interrupt is disabled by default.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_enable_line(SOC_UART_HANDLE_t * handle)
{
    /* Set the IER::ELSI (Interrupt Enable Register :: Enable Line Status Interrupt) bit. */
    return soc_uart_ier_mask_set_helper(handle, SOC_UART_IER_DLH_ELSI_DHL2_SET_MSK);
}

/*!
 * Disables the receiver from generating line status interrupts.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_disable_line(SOC_UART_HANDLE_t * handle)
{
    /* Clear the IER::ELSI (Interrupt Enable Register :: Enable Line Status Interrupt) bit. */
    return soc_uart_ier_mask_clr_helper(handle, SOC_UART_IER_DLH_ELSI_DHL2_SET_MSK);
}

/*!
 * Enables the UART to generate modem status interrupts. Enabling this
 * interrupt allows for the following interrupt signal(s):
 *  * SOC_UART_INT_STATUS_MODEM
 *
 * This interrupt is disabled by default.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_enable_modem(SOC_UART_HANDLE_t * handle)
{
    /* Set the IER::EDSSI (Interrupt Enable Register :: Enable Modem Status Interrupt) bit. */
    return soc_uart_ier_mask_set_helper(handle, SOC_UART_IER_DLH_EDSSI_DHL3_SET_MSK);
}


/*!
 * Disables the UART from generate modem status interrupts.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_disable_modem(SOC_UART_HANDLE_t * handle)
{
    /* Clear the IER::EDSSI (Interrupt Enable Register :: Enable Modem Status Interrupt) bit. */
    return soc_uart_ier_mask_clr_helper(handle, SOC_UART_IER_DLH_EDSSI_DHL3_SET_MSK);
}

/*!
 * Disables all interrupts on the UART.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_disable_all(SOC_UART_HANDLE_t * handle)
{
    /* Clear the IER::(ERBFI | ETBEI | ELSI | EDSSI)
    //   (Interrupt Enable Register :: (Enable Receive Buffer Full Interrupt   |
    //                                  Enable Transmit Buffer Empty Interrupt |
    //                                  Enable Line Status Interrupt           |
    //                                  Enable Modem Status Interrupt)) bits   */
    return soc_uart_ier_mask_clr_helper(handle,  SOC_UART_IER_DLH_ERBFI_DLH0_SET_MSK |
                                                 SOC_UART_IER_DLH_ETBEI_DLH1_SET_MSK |
                                                 SOC_UART_IER_DLH_ELSI_DHL2_SET_MSK  |
                                                 SOC_UART_IER_DLH_EDSSI_DHL3_SET_MSK);
}

/*!
 * Queries the interrupt status of the UART. This returns the highest priority
 * interrupt pending. The appropriate interrupts must be enabled for them be
 * generated in the UART.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       status
 *              [out] Pointer to an output parameter that contains the current
 *              interrupt status of the UART.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_int_status_get(SOC_UART_HANDLE_t * handle,
                                         SOC_UART_INT_STATUS_t * status)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Read IIR::IID (Interrupt Identity Register :: Interrupt ID) */
        *status = (SOC_UART_INT_STATUS_t) SOC_UART_IIR_ID_GET(soc_read_word(SOC_UART_IIR_ADDR(handle->location)));
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}



/*!
 * Enables automatic flow control in the UART modem. When in this mode, the
 * rts_n is gated with the threshold trigger condition of the receiver FIFO.
 *
 * The socera 16550 Compatible Soft IP UART may not have this option enabled.
 *
 * The FIFOs must be enabled for flow control to be used.
 *
 * The recommended bring up for flow control is as follows:
 *  * Enable automatic flow control by calling soc_uart_flowcontrol_enable().
 *    This will allow both the receiver FIFO and user RTS to control the rts_n
 *    output. Because the user RTS is not enabled, the rts_n will be inactive
 *    high.
 *  * Enable RTS by calling soc_uart_modem_enable_rts(). This will give the
 *    receiver FIFO to have full control of the rts_n output.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_flowcontrol_enable(SOC_UART_HANDLE_t * handle)
{
    /* Verify that the FIFO is enabled */
    if (!(handle->fcr & SOC_UART_FCR_FIFOE_SET_MSK))
    {
        return SOC_S_GENERAL_ERROR;
    }

    /* For the socera 16550 Compatible Soft UART, check that Hardware Flowcontrol is enabled. */
    if (handle->device == SOC_UART_DEVICE_INTEL_IP)
    {
        /* Read the CPR::AFCE_Mode (Component Parameter Register :: Auto Flow Control mode) bit. */
        uint32_t cpr = soc_read_word(SOC_UART_CPR_ADDR(handle->location));
        if (!(SOC_UART_CPR_AFCE_MODE_SET_MSK & cpr))
        {
            return SOC_S_GENERAL_ERROR;
        }
    }

    /* Set MCR::AFCE (Modem Control Register :: Automatic FlowControl Enable) bit. */
    return soc_uart_mcr_mask_set_helper(handle, SOC_UART_MCR_AFCE_SET_MSK);
}

/*!
 * Disables automatic flow control in the UART modem.
 *
 * The recommended bring down for flow control is as follows:
 *  * Disable RTS by calling SOC_UART_modem_disable_rts(). This will disable
 *    generation of the rts_n ouput.
 *  * Disable automatic flow control by calling
 *    soc_uart_flowcontrol_disable().
 *
 * The receiver FIFO will still be active after these steps.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_flowcontrol_disable(SOC_UART_HANDLE_t * handle)
{
    /* Clear MCR::AFCE (Modem Control Register :: Automatic FlowControl Enable) bit. */
    return soc_uart_mcr_mask_clr_helper(handle, SOC_UART_MCR_AFCE_SET_MSK);
}

/*!
 * Puts the UART in loopback mode. This is used for diagnostic and test
 * purposes.
 *
 * The SoCFPGA UARTs does not support automatic flow control when in loopback
 * mode.
 *
 * The socera 16550 Compatible Soft IP UART implements this in 13.0sp1 and
 * later. Setting this has no effect with 13.0.
 *
 * When in this mode, the modem control inputs (dsr_n, cts_n, ri_n, dcd_n) are
 * disconnected and the modem control outputs (dtr_n, rts_n, out1_n, out2_n)
 * are held inactive high externally and internally looped back to the inputs.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_loopback_enable(SOC_UART_HANDLE_t * handle)
{
    /* Loopback is not implemented in the socera 16550 Compatible Soft UART. */
    if (handle->device == SOC_UART_DEVICE_INTEL_IP)
    {
        return SOC_S_GENERAL_ERROR;
    }

    /* Set MCR::Loopback (Modem Control Register :: Loopback) bit. */
    return soc_uart_mcr_mask_set_helper(handle, SOC_UART_MCR_LOOPBACK_SET_MSK);
}


/*!
 * Takes the UART out of loopback mode.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_loopback_disable(SOC_UART_HANDLE_t * handle)
{
    /* Clear MCR::Loopback (Modem Control Register :: Loopback) bit. */
    return soc_uart_mcr_mask_clr_helper(handle, SOC_UART_MCR_LOOPBACK_SET_MSK);
}

/*!
 * Asserts the OUT1 output. OUT1 is inverted then driven out to out1_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_uart_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_modem_enable_out1(SOC_UART_HANDLE_t * handle)
{
    /* Set MCR::Out1 (Modem Control Register :: Out1) bit. */
    return soc_uart_mcr_mask_set_helper(handle, SOC_UART_MCR_OUT1_SET_MSK);
}

/*!
 * Unasserts the OUT1 output.  OUT1 is inverted then driven out to out1_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_uart_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_modem_disable_out1(SOC_UART_HANDLE_t * handle)
{
    /* Clear MCR::Out1 (Modem Control Register :: Out1) bit. */
    return soc_uart_mcr_mask_clr_helper(handle, SOC_UART_MCR_OUT1_SET_MSK);
}

/*!
 * Asserts the OUT2 output. OUT2 is inverted then driven out to out2_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_uart_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_modem_enable_out2(SOC_UART_HANDLE_t * handle)
{
    /* Set MCR::Out2 (Modem Control Register :: Out2) bit. */
    return soc_uart_mcr_mask_set_helper(handle, SOC_UART_MCR_OUT2_SET_MSK);
}

/*!
 * Unasserts the OUT2 output. OUT2 is inverted then driven out to out2_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_uart_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_modem_disable_out2(SOC_UART_HANDLE_t * handle)
{
    /* Clear MCR::Out2 (Modem Control Register :: Out2) bit. */
    return soc_uart_mcr_mask_clr_helper(handle, SOC_UART_MCR_OUT2_SET_MSK);
}

/*!
 * Asserts the RTS (Request To Send) output. RTS is inverted then driven out
 * to rts_n. RTS is used to inform the modem that the UART is ready to receive
 * data.
 *
 * There are special considerations when the UART is in automatic flow control
 * mode. See soc_uart_flowcontrol_enable() for more information.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_uart_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_modem_enable_rts(SOC_UART_HANDLE_t * handle)
{
    /* Set MCR::RTS (Modem Control Register :: Request To Send) bit. */
    return soc_uart_mcr_mask_set_helper(handle, SOC_UART_MCR_RTS_SET_MSK);
}

/*!
 * Deaserts the RTS (Request To Send) output. RTS is inverted then driven out
 * to rts_n.
 *
 * There are special considerations when the UART is in automatic flow control
 * mode. See soc_uart_flowcontrol_enable() for more information.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_uart_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t SOC_UART_modem_disable_rts(SOC_UART_HANDLE_t * handle)
{
    /* Clear MCR::RTS (Modem Control Register :: Request To Send) bit. */
    return soc_uart_mcr_mask_clr_helper(handle, SOC_UART_MCR_RTS_SET_MSK);
}

/*!
 * Asserts the DTR (Data Terminal Ready) output. DTR is inverted then driven
 * out to dtr_n. DTR is used to inform the modem that UART is ready to
 * establish communications.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_uart_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_modem_enable_dtr(SOC_UART_HANDLE_t * handle)
{
    /* Set MCR::DTR (Modem Control Register :: Data Terminal Ready) bit. */
    return soc_uart_mcr_mask_set_helper(handle, SOC_UART_MCR_DTR_SET_MSK);
}

/*!
 * Deasserts the DTR (Data Terminal Ready) output. DTR is inverted then driven
 * out to dtr_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_uart_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_modem_disable_dtr(SOC_UART_HANDLE_t * handle)
{
    /* Clear MCR::DTR (Modem Control Register :: Data Terminal Ready) bit. */
    return soc_uart_mcr_mask_clr_helper(handle, SOC_UART_MCR_DTR_SET_MSK);
}

/*!
 * Reads the modem status from the UART.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       status
 *              [out] Pointer to an output parameter that contains the current
 *              modem status of the UART as a register mask.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_modem_status_get(SOC_UART_HANDLE_t * handle,
                                          uint32_t * status)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Read the MSR (Modem Status Register). */
        *status = soc_read_word(soc_UART_MSR_ADDR(handle->location));
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Sets the configuration for a given character frame.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       databits
 *              The number of databits for each character frame.
 *
 * \param       parity
 *              The parity to use for each character frame.
 *
 * \param       stopbits
 *              The number of stopbits for each character frame.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_line_config_set( SOC_UART_HANDLE_t * handle,
                                          SOC_UART_DATABITS_t databits,
                                          SOC_UART_PARITY_t parity,
                                          SOC_UART_STOPBITS_t stopbits)
{
    /* LCR (Line Control Register) cache. */
    uint32_t lcr = 0;

    /* Validate the databits parameter. */
    switch (databits)
    {
    case SOC_UART_DATABITS_5:
    case SOC_UART_DATABITS_6:
    case SOC_UART_DATABITS_7:
    case SOC_UART_DATABITS_8:
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    /* Validate the parity parameter. */
    switch (parity)
    {
    case SOC_UART_PARITY_DISABLE:
    case SOC_UART_PARITY_ODD:
    case SOC_UART_PARITY_EVEN:
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    /* Validate the stopbits parameter. */
    switch (stopbits)
    {
    case SOC_UART_STOPBITS_1:
    case SOC_UART_STOPBITS_2:
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:

        /* Configure the number of databits */
        lcr |= soc_UART_LCR_DLS_SET(databits);

        /* Configure the number of stopbits */
        lcr |= soc_UART_LCR_STOP_SET(stopbits);

        /* Configure the parity */
        if (parity != SOC_UART_PARITY_DISABLE)
        {
            /* Enable parity in LCR */
            lcr |= SOC_UART_LCR_PEN_SET_MSK;

            if (parity == SOC_UART_PARITY_EVEN)
            {
                /* Enable even parity in LCR; otherwise it's odd parity. */
                lcr |= SOC_UART_LCR_EPS_SET_MSK;
            }
        }

        /* Update LCR (Line Control Register) */
        soc_replbits_word(SOC_UART_LCR_ADDR(handle->location),
                          SOC_UART_LCR_DLS_SET_MSK
                        | SOC_UART_LCR_STOP_SET_MSK
                        | SOC_UART_LCR_PEN_SET_MSK
                        | SOC_UART_LCR_EPS_SET_MSK,
                        lcr);

        break;

    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Starts transmitting a break condition by transmitting a logic 0 state
 * longer than a frame would normally be transmitted.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_line_break_enable(SOC_UART_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Set the LCR::Break (Line Control Register :: Break) bit. */
        soc_setbits_word(SOC_UART_LCR_ADDR(handle->location), SOC_UART_LCR_BREAK_SET_MSK);
        break;

    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}

/*!
 * Stops transmitting a break condition.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_line_break_disable(SOC_UART_HANDLE_t * handle)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Clear the LCR::Break (Line Control Register :: Break) bit. */
        soc_clrbits_word(SOC_UART_LCR_ADDR(handle->location), SOC_UART_LCR_BREAK_SET_MSK);
        break;

    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}


/*!
 * Reads the line status from the UART.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \param       status
 *              [out] Pointer to an output parameter that contains the current
 *              line status of the UART.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_status_t soc_uart_line_status_get(SOC_UART_HANDLE_t * handle,
                                          uint32_t * status)
{
    switch (handle->device)
    {
    case SOC_UART_DEVICE_SOCFPGA_UART0:
    case SOC_UART_DEVICE_SOCFPGA_UART1:
    case SOC_UART_DEVICE_INTEL_IP:
        /* Read the LSR (Line Status Register). */
        *status = soc_read_word(SOC_UART_LSR_ADDR(handle->location));
        break;
    default:
        return SOC_S_GENERAL_ERROR;
    }

    return SOC_S_SUCCESSFULLY;
}
