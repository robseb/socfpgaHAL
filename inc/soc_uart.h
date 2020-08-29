
#ifndef __SOC__UART_H__
#define __SOC__UART_H__

//#include "hwlib.h"
//#include "soc_clock_manager.h"
#include "soc_uart.h"

#ifdef __cplusplus
extern "C"
{
#endif



/*!
 * The type definition for status codes returned by the HWLIB.
 */
typedef int32_t             soc_STATUS_CODE;

/*! Definitions of status codes returned by the HWLIB. */

/*! The operation was successful. */
#define soc_E_SUCCESS               0

/*! The operation failed. */
#define soc_E_ERROR                 (-1)
/*! FPGA configuration error detected.*/
#define soc_E_FPGA_CFG              (-2)
/*! FPGA CRC error detected. */
#define soc_E_FPGA_CRC              (-3)
/*! An error occurred on the FPGA configuration bitstream input source. */
#define soc_E_FPGA_CFG_STM          (-4)
/*! The FPGA is powered off. */
#define soc_E_FPGA_PWR_OFF          (-5)
/*! The SoC does not currently control the FPGA. */
#define soc_E_FPGA_NO_SOC_CTRL      (-6)
/*! The FPGA is not in USER mode. */
#define soc_E_FPGA_NOT_USER_MODE    (-7)
/*! An argument violates a range constraint. */
#define soc_E_ARG_RANGE             (-8)
/*! A bad argument value was passed. */
#define soc_E_BAD_ARG               (-9)
/*! The operation is invalid or illegal. */
#define soc_E_BAD_OPERATION         (-10)
/*! An invalid option was selected. */
#define soc_E_INV_OPTION            (-11)
/*! An operation or response timeout period expired. */
#define soc_E_TMO                   (-12)
/*! The argument value is reserved or unavailable. */
#define soc_E_RESERVED              (-13)
/*! A clock is not enabled or violates an operational constraint. */
#define soc_E_BAD_CLK               (-14)
/*! The version ID is invalid. */
#define soc_E_BAD_VERSION           (-15)
/*! The buffer does not contain enough free space for the operation. */
#define soc_E_BUF_OVF               (-20)


typedef uint32_t soc_freq_t;














/*!
 * \addtogroup UART UART Driver API
 *
 * This module defines the Universal Asynchronous Receiver/Transmitter (UART)
 * API for accessing and using the UART resources. The API allows for general
 * control of a 16550 compatible UART controller.
 *
 * This implementation can control the following UARTs:
 *  * SoCFPGA On-board UARTs
 *  * socera 16550 Compatible Soft IP UART
 *
 * The following reference materials were used in the design of this API:
 *  * Synopsys&reg; DesignWare DW_apb_uart Databook v3.10a
 *
 * @{
 */

/*!
 * \addtogroup UART_BASIC UART Basic
 *
 * This group of APIs provides basic access to the UART to initialize,
 * uninitialize, read, write, and reset the UART.
 *
 * @{
 */

/*!
 * This type definition enumerates the list of UARTs available on the system.
 */
typedef enum soc_16550_DEVICE_e
{
    /*!
     * This option selects UART0 in the SoC FPGA.
     */
    soc_16550_DEVICE_SOCFPGA_UART0 = 0,

    /*!
     * This option selects UART1 in the SoC FPGA.
     */
    soc_16550_DEVICE_SOCFPGA_UART1 = 1,

    /*!
     * This option selects an socera 16550 Compatible soft IP UART. The memory
     * location of the device must be provided as part of the initialization.
     */
    soc_16550_DEVICE_socERA_16550_UART = 0x100
}
soc_16550_DEVICE_t;

/*!
 * This structure is used to represent a handle to a specific UART on the
 * system. The internal members are undocumented and should be not socered
 * outside of this API.
 */
typedef struct soc_16550_HANDLE_s
{
    soc_16550_DEVICE_t device;
    void *             location;
    soc_freq_t         clock_freq;
    uint32_t           data;
    uint32_t           fcr;
}
soc_16550_HANDLE_t;

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
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device identifier is invalid.
 * \retval      soc_E_BAD_CLK   The required clock is not yet setup.
 */
soc_STATUS_CODE soc_16550_init(soc_16550_DEVICE_t device,
                               void * location,
                               soc_freq_t clock_freq,
                               soc_16550_HANDLE_t * handle);

/*!
 * Performs the uninitialization steps for the UART. This should be the last
 * API call made to cleanup the UART.
 *
 * After calling this function, the handle will need to be initialized again
 * before being used by calling soc_16550_init().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_uninit(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_reset(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_enable(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_disable(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_read(soc_16550_HANDLE_t * handle,
                               char * item);

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
soc_STATUS_CODE soc_16550_write(soc_16550_HANDLE_t * handle,
                                char item);

/*!
 * @}
 */

/*!
 * \addtogroup UART_FIFO UART FIFO Interface
 *
 * This group of APIs provides access, configuration, and control of the UART
 * FIFO. The FIFO allows the UART to buffer received data and data to be
 * transmitted.
 *
 * @{
 */

/*!
 * This type definition enumerates the receiver FIFO level conditions that
 * will trigger the receiver FIFO to issue a receiver FIFO full event.
 */
typedef enum soc_16550_FIFO_TRIGGER_RX_e
{
    /*!
     * 1 or more character(s) in the receiver FIFO will trigger an event.
     */
    soc_16550_FIFO_TRIGGER_RX_ANY = 0,

    /*!
     * 25% or higher capacity usage in the receiver FIFO will trigger an
     * event.
     */
    soc_16550_FIFO_TRIGGER_RX_QUARTER_FULL = 1,

    /*!
     * 50% or higher capacity usage in the receiver FIFO will trigger an
     * event.
     */
    soc_16550_FIFO_TRIGGER_RX_HALF_FULL = 2,

    /*!
     * 2 characters less than the receiver FIFO capacity will trigger an
     * event.
     */
    soc_16550_FIFO_TRIGGER_RX_ALMOST_FULL = 3
}
soc_16550_FIFO_TRIGGER_RX_t;

/*!
 * This type definition enumerates the transmitter FIFO level conditions that
 * will trigger the transmitter FIFO to issue a transmitter FIFO empty event.
 */
typedef enum soc_16550_FIFO_TRIGGER_TX_e
{
    /*!
     * Transmitter FIFO being completely empty will trigger an event.
     */
    soc_16550_FIFO_TRIGGER_TX_EMPTY = 0,

    /*!
     * 2 or less character(s) in the transmitter FIFO will trigger an event.
     */
    soc_16550_FIFO_TRIGGER_TX_ALMOST_EMPTY = 1,

    /*!
     * 25% or less capacity usage in the transmitter FIFO will trigger an
     * event.
     */
    soc_16550_FIFO_TRIGGER_TX_QUARTER_FULL = 2,

    /*!
     * 50% or less capacity usage in the transmitter FIFO will trigger an
     * event.
     */
    soc_16550_FIFO_TRIGGER_TX_HALF_FULL = 3
}
soc_16550_FIFO_TRIGGER_TX_t;

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
soc_STATUS_CODE soc_16550_fifo_enable(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_fifo_disable(soc_16550_HANDLE_t * handle);

/*!
 * Reads the given buffer from the receiver FIFO in the UART.
 *
 * The available characters in the FIFO can be determined by a few ways. Users
 * can determine the number of items by calling soc_16550_fifo_level_get_rx().
 *
 * Another way is by using the RX trigger and RX interrupt. First determine the
 * RX FIFO size by calling soc_16550_fifo_size_get_rx(). Then set the desired
 * trigger level by calling soc_16550_fifo_trigger_set_rx(). Calculate the
 * triggering point by applying trigger description on the FIFO size. Enable RX
 * interrupts by calling soc_16550_int_enable_rx(). When the RX interrupt fires
 * due to the soc_16550_INT_STATUS_RX_DATA condition, the calculated triggering
 * point value can be used to determine the RX FIFO level. If the interrupt
 * fires due to the soc_16550_INT_STATUS_RX_TIMEOUT, the RX FIFO can be
 * completely emptied by repeatedly polling the Line Status
 * soc_16550_LINE_STATUS_DR condition by calling soc_16550_line_status_get().
 * These steps are necessary if the UART does not implement FIFO level query
 * functionality. As of 13.0sp1, this applies to the socera 16550 Compatible
 * Soft UART.
 *
 * Reading more data than that which is available can result in invalid data
 * appearing like valid data.
 *
 * The FIFO must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
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
soc_STATUS_CODE soc_16550_fifo_read(soc_16550_HANDLE_t * handle,
                                    char * buffer,
                                    size_t count);

/*!
 * Writes the given buffer to the transmitter FIFO in the UART.
 *
 * The available space in the FIFO can be determined by a few ways. Users can
 * determine the number of items by calculating the FIFO capacity minus the
 * FIFO level. This can be done by calling  soc_16550_fifo_size_get_tx() and
 * soc_16550_fifo_level_get_tx() respectively.
 *
 * Another way is by using the TX trigger and TX interrupt. First determine the
 * TX FIFO size by calling soc_16550_fifo_size_get_tx(). The set the desired
 * trigger level by calling soc_16550_fifo_trigger_set_tx(). Calculate the
 * triggering point by applying the trigger description on the FIFO size.
 * Enable TX interrupts by calling soc_16550_int_enable_tx(). When the TX
 * interrupt fires, calculate the empty entries in the FIFO by subtracting the
 * TX FIFO size and the calculated value. These steps are necessary if the UART
 * does not implement FIFO level query functionality. As of 13.0sp1, this
 * applies to the socera 16550 Compatible Soft UART.
 *
 * Writing more data that there is space can result in data lost due to
 * overflowing.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
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
soc_STATUS_CODE soc_16550_fifo_write(soc_16550_HANDLE_t * handle,
                                     const char * buffer,
                                     size_t count);


/*!
 * Writes the given buffer to the transmitter FIFO in the UART, blocking 
 * if the fifo is full until there is enough space to write the string
 *
 * Writing more data that there is space can result in data lost due to
 * overflowing.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
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
 *                      in the same way as soc_16550_fifo_write()
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_fifo_write_safe(soc_16550_HANDLE_t * handle,
                                     const char * buffer,
                                     size_t count,
                                     bool safe);
/*!
 * Clears the contents of the receiver FIFO. Any characters which were
 * previously contained in that FIFO will be discarded.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_fifo_clear_rx(soc_16550_HANDLE_t * handle);

/*!
 * Clears the contents of the transmitter FIFO. Any characters which were
 * previously contained in that FIFO will be discarded.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_fifo_clear_tx(soc_16550_HANDLE_t * handle);

/*!
 * Clears the contents of the receiver and transmitter FIFO. Any characters
 * which were previously contained on those FIFOs will be discarded.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_fifo_clear_all(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_fifo_size_get_rx(soc_16550_HANDLE_t * handle,
                                           uint32_t * size);

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
soc_STATUS_CODE soc_16550_fifo_size_get_tx(soc_16550_HANDLE_t * handle,
                                           uint32_t * size);

/*!
 * Queries the current level of the receiver FIFO.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
 *
 * For the socera 16550 Compatible UART, it may not be possible to read the
 * FIFO level and this function may always report 0. For more information on
 * interacting with the FIFO in this situation, see documentation for
 * soc_16550_fifo_read().
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
soc_STATUS_CODE soc_16550_fifo_level_get_rx(soc_16550_HANDLE_t * handle,
                                            uint32_t * level);

/*!
 * Queries the current level of the transmitter FIFO.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
 *
 * For the socera 16550 Compatible UART, it may not be possible to read the
 * FIFO level and this function may always report 0. For more information on
 * interacting with the FIFO in this situation, see documentation for
 * soc_16550_fifo_write().
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
soc_STATUS_CODE soc_16550_fifo_level_get_tx(soc_16550_HANDLE_t * handle,
                                            uint32_t * level);

/*!
 * Sets the receiver FIFO level which will trigger the receiver FIFO to issue
 * receiver FIFO full event. For the list of available receiver FIFO trigger
 * levels, see the documentation for soc_16550_FIFO_TRIGGER_RX_t.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
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
soc_STATUS_CODE soc_16550_fifo_trigger_set_rx(soc_16550_HANDLE_t * handle,
                                              soc_16550_FIFO_TRIGGER_RX_t trigger);

/*!
 * Sets the transmitter FIFO level which will trigger the transmitter FIFO to
 * transmitter FIFO empty event. For the list of available transmitter FIFO
 * trigger levels, see the documentation for soc_16550_FIFO_TRIGGER_TX_t.
 *
 * The FIFOs must first be enabled before calling this function by calling
 * soc_16550_fifo_enable().
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
soc_STATUS_CODE soc_16550_fifo_trigger_set_tx(soc_16550_HANDLE_t * handle,
                                              soc_16550_FIFO_TRIGGER_TX_t trigger);

/*!
 * @}
 */

/*!
 * \addtogroup UART_BAUD UART Baudrate Interface
 *
 * This group of APIs allows for the configuration of the UART's baudrate
 * generation related functions.
 *
 * The UART baudrate is determined by dividing the soc_CLK_L4_SP clock with
 * the configured divisor.
 *
 * @{
 */

/*!
 * This enumeration lists out the common baudrates used with modem and serial
 * ports. Not every baudrate is available for the UART due to the limits of
 * the serial clock frequency and divisor value.
 */
typedef enum soc_16550_BAUDRATE_e
{
    soc_16550_BAUDRATE_50     =     50, /*!< 50 bps baudrate. */
    soc_16550_BAUDRATE_75     =     75, /*!< 75 bps baudrate. */
    soc_16550_BAUDRATE_150    =    150, /*!< 150 bps baudrate. */
    soc_16550_BAUDRATE_300    =    300, /*!< 300 bps baudrate. */
    soc_16550_BAUDRATE_600    =    600, /*!< 600 bps baudrate. */
    soc_16550_BAUDRATE_900    =    900, /*!< 900 bps baudrate. */
    soc_16550_BAUDRATE_1200   =   1200, /*!< 1200 bps baudrate. */
    soc_16550_BAUDRATE_1800   =   1800, /*!< 1800 bps baudrate. */
    soc_16550_BAUDRATE_2400   =   2400, /*!< 2400 bps baudrate. */
    soc_16550_BAUDRATE_3600   =   3600, /*!< 3600 bps baudrate. */
    soc_16550_BAUDRATE_4800   =   4800, /*!< 4800 bps baudrate. */
    soc_16550_BAUDRATE_7200   =   7200, /*!< 7200 bps baudrate. */
    soc_16550_BAUDRATE_9600   =   9600, /*!< 9600 bps baudrate. */
    soc_16550_BAUDRATE_14400  =  14400, /*!< 14400 bps baudrate. */
    soc_16550_BAUDRATE_19200  =  19200, /*!< 19200 bps baudrate. */
    soc_16550_BAUDRATE_28800  =  28800, /*!< 28800 bps baudrate. */
    soc_16550_BAUDRATE_38400  =  38400, /*!< 38400 bps baudrate. */
    soc_16550_BAUDRATE_57600  =  57600, /*!< 57600 bps baudrate. */
    soc_16550_BAUDRATE_115200 = 115200  /*!< 115200 bps baudrate. */
}
soc_16550_BAUDRATE_t;

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
soc_STATUS_CODE soc_16550_baudrate_get(soc_16550_HANDLE_t * handle,
                                       uint32_t * baudrate);

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
 * \retval      soc_E_ARG_RANGE The given baudrate is not possible due to
 *                              limitations of the baudrate divisor and/or
 *                              serial clock.
 */
soc_STATUS_CODE soc_16550_baudrate_set(soc_16550_HANDLE_t * handle,
                                       uint32_t baudrate);

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
soc_STATUS_CODE soc_16550_divisor_get(soc_16550_HANDLE_t * handle,
                                      uint32_t * divisor);

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
soc_STATUS_CODE soc_16550_divisor_set(soc_16550_HANDLE_t * handle,
                                      uint32_t divisor);

/*!
 * @}
 */

/*!
 * \addtogroup UART_INT UART Interrupt Interface
 *
 * This group of APIs provides access, configuration, and control of the 
 * UART interrupts. 
 *
 * @{
 */

/*!
 * This type definition enumerates the different interrupt conditions that can
 * be generated by the UART controller.
 *
 * Interrupts are listed in highest to lowest priority order.
 */
typedef enum soc_16550_INT_STATUS_e
{
    /*!
     * This interrupt signals that a overrun, parity, or framing error 
     * occurred, or a break event occured. The interrupt is cleared by reading
     * the line status by calling soc_16550_line_status_get() or by disabling
     * line status interrupts by calling soc_16550_int_disable_line().
     */
    soc_16550_INT_STATUS_LINE = 0x6,

    /*!
     * This interrupt signals that some data is available to be read from the
     * UART. The definition of some depends on whether FIFOs are enabled or
     * not.
     *
     * If FIFOs are disabled, this interrupt signals that the receiver
     * contains data. In this case, the interrupt is cleared by reading the
     * data from the UART by calling soc_16550_read().
     *
     * If FIFOs are enabled, this interrupt signals that the receiver FIFO
     * level is above the receiver trigger level specified. In this case, the
     * interrupt is cleared by reading a sufficiently large buffer from the
     * receiver FIFO such that the FIFO is filled below the receiver trigger
     * level specified by calling soc_16550_fifo_read() or by adjusting the
     * receiver trigger level appropriately by calling
     * soc_16550_fifo_trigger_set_rx().
     *
     * In either case, this interrupt can also be cleared by disabling
     * receiver interrupts by calling soc_16550_int_disable_rx().
     */
    soc_16550_INT_STATUS_RX_DATA = 0x4,

    /*!
     * This interrupt signals that data is available in the receiver FIFO and
     * that there has been no activity with the receiver FIFO for the last 4
     * character frames. In essence, the receiver FIFO has temporarily settled
     * thus it may be a good time to empty the receiver FIFO. This interrupt
     * is only available if FIFOs are enabled. The interrupt is cleared by
     * reading from the receiver FIFO by calling soc_16550_fifo_read() or by
     * disabling receiver interrupts by calling soc_16550_int_disable_rx().
     */
    soc_16550_INT_STATUS_RX_TIMEOUT = 0xC,

    /*!
     * This interrupt signals that the transmitter is idling. The definition
     * of idling depends on whether FIFOs are enabled or not.
     *
     * If FIFOs are disabled, this interrupt signals that the transmitter
     * shift register is empty. In this case, the interrupt is cleared by
     * writing data to the UART by calling soc_16550_write().
     *
     * If FIFO are enabled, this interrupt signals that the transmitter FIFO
     * level is below the transmitter trigger level specified. In this case,
     * the interrupt is cleared by writing a sufficiently large buffer to the
     * transmitter FIFO such that the FIFO is filled above the transmitter
     * trigger level specified by calling soc_16550_fifo_write() or by
     * adjusting the transmitter trigger level appropriately by calling
     * soc_16550_fifo_trigger_set_tx().
     *
     * In either case, this interrupt can also be cleared by disabling
     * transmitter interrupts by calling soc_16550_int_disable_tx().
     */
    soc_16550_INT_STATUS_TX_IDLE = 0x2,

    /*!
     * Modem status interrupt pending. The interrupt is cleared by reading the
     * modem status by calling soc_16550_modem_status_get() or by disabling
     * modem status interrupts by calling soc_16550_int_disable_modem().
     */
    soc_16550_INT_STATUS_MODEM = 0x0,

    /*!
     * No interrupts pending.
     */
    soc_16550_INT_STATUS_NONE = 0x1
}
soc_16550_INT_STATUS_t;

/*!
 * Enables the receiver FIFO to generate interrupts. Enabling this interrupt
 * allows for the following interrupt signal(s):
 *  * soc_16550_INT_STATUS_RX_DATA
 *  * soc_16550_INT_STATUS_RX_TIMEOUT
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
soc_STATUS_CODE soc_16550_int_enable_rx(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_int_disable_rx(soc_16550_HANDLE_t * handle);

/*!
 * Enables the transmitter FIFO to generate interrupts. Enabling this
 * interrupt allows for the following interrupt signal(s):
 *  * soc_16550_INT_STATUS_TX_IDLE
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
soc_STATUS_CODE soc_16550_int_enable_tx(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_int_disable_tx(soc_16550_HANDLE_t * handle);

/*!
 * Enables the receiver to generate line status interrupts. Enabling this
 * interrupt allows for the following interrupt signal(s):
 *  * soc_16550_INT_STATUS_LINE
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
soc_STATUS_CODE soc_16550_int_enable_line(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_int_disable_line(soc_16550_HANDLE_t * handle);

/*!
 * Enables the UART to generate modem status interrupts. Enabling this
 * interrupt allows for the following interrupt signal(s):
 *  * soc_16550_INT_STATUS_MODEM
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
soc_STATUS_CODE soc_16550_int_enable_modem(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_int_disable_modem(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_int_disable_all(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE  soc_16550_int_status_get(soc_16550_HANDLE_t * handle,
                                          soc_16550_INT_STATUS_t * status);

/*!
 * @}
 */

/*!
 * \addtogroup UART_MODEM UART Modem Interface
 *
 * This group of APIs provides access, configuration, and control of the UART
 * Modem interface.
 *
 * @{
 */

/*!
 * This type definition enumerates the set of UART modem status conditions as
 * register mask values.
 */
typedef enum soc_16550_MODEM_STATUS_e
{
    /*!
     * Data Carrier Detect. This status indicates that the carrier has been
     * detected by the modem. It corresponds to an inverted dcd_n input. DCD
     * is unasserted when dcd_n is logic 1 and asserted when dcd_n is logic 0.
     */
    soc_16550_MODEM_STATUS_DCD = 1 << 7,

    /*!
     * Ring Indicator. This status indicates that the telephone ringing signal
     * has been redeived by the modem. It corresponds to an inverted ri_n
     * input. RI is unasserted when ri_n is logic 1 and asserted when ri_n is
     * logic 0.
     */
    soc_16550_MODEM_STATUS_RI = 1 << 6,

    /*!
     * Data Set Ready. This status indicates that the modem is ready to
     * establish communications with the UART. It corresponds to an inverted
     * dsr_n input. DSR is unasserted when dsr_n is logic 1 and asserted when
     * dsr_n is logic 0.
     */
    soc_16550_MODEM_STATUS_DSR = 1 << 5,

    /*!
     * Clear To Send. This status indicates the current state of the modem
     * cts_n line. It corresponds to an inverted cts_n input. CTS is
     * unasserted when cts_n is logic 1 and asserted when cts_n is logic 0.
     */
    soc_16550_MODEM_STATUS_CTS = 1 << 4,

    /*!
     * Delta Data Carrier Detect. This status condition indicates that the
     * Data Carrier Detect has changed since the last time the modem status
     * was read. Reading the modem status clears this status. For more
     * information about the Data Carrier Detect status, see
     * soc_16550_MODEM_STATUS_DCD.
     */
    soc_16550_MODEM_STATUS_DDCD = 1 << 3,

    /*!
     * Trailing Edge of Ring Indicator. This status indicates that the Ring
     * Indicator has changed from asserted to unasserted. Reading the modem
     * status will clear this status. For more information about the Ring
     * Indicator status, reference soc_16550_MODEM_STATUS_RI.
     */
    soc_16550_MODEM_STATUS_TERI = 1 << 2,

    /*!
     * Delta Data Set Ready. This status condition indicates that the Data Set
     * Ready has changed since the last time the modem status was read.
     * Reading the modem status will clear this status. For more information
     * about the Data Set Ready status, see soc_16550_MODEM_STATUS_DSR.
     */
    soc_16550_MODEM_STATUS_DDSR = 1 << 1,

    /*!
     * Delta Clear To Send. This status condition indicates that the Clear To
     * Send has changed since the last time the modem status was read. Reading
     * the modem status will clear this status. For more information about the
     * Clear To Send status, see soc_16550_MODEM_STATUS_CTS.
     */
    soc_16550_MODEM_STATUS_DCTS = 1 << 0
}
soc_16550_MODEM_STATUS_t;

/*!
 * Enables automatic flow control in the UART modem. When in this mode, the
 * rts_n is gated with the threshold trigger condition of the receiver FIFO.
 *
 * The socera 16550 Compatible Soft IP UART may not have this option enabled.
 *
 * The FIFOs must be enabled for flow control to be used.
 *
 * The recommended bring up for flow control is as follows:
 *  * Enable automatic flow control by calling soc_16550_flowcontrol_enable().
 *    This will allow both the receiver FIFO and user RTS to control the rts_n
 *    output. Because the user RTS is not enabled, the rts_n will be inactive
 *    high.
 *  * Enable RTS by calling soc_16550_modem_enable_rts(). This will give the
 *    receiver FIFO to have full control of the rts_n output.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_flowcontrol_enable(soc_16550_HANDLE_t * handle);

/*!
 * Disables automatic flow control in the UART modem.
 *
 * The recommended bring down for flow control is as follows:
 *  * Disable RTS by calling soc_16550_modem_disable_rts(). This will disable
 *    generation of the rts_n ouput.
 *  * Disable automatic flow control by calling
 *    soc_16550_flowcontrol_disable().
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
soc_STATUS_CODE soc_16550_flowcontrol_disable(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_loopback_enable(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_loopback_disable(soc_16550_HANDLE_t * handle);

/*!
 * Asserts the OUT1 output. OUT1 is inverted then driven out to out1_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_16550_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_modem_enable_out1(soc_16550_HANDLE_t * handle);

/*!
 * Unasserts the OUT1 output.  OUT1 is inverted then driven out to out1_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_16550_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_modem_disable_out1(soc_16550_HANDLE_t * handle);

/*!
 * Asserts the OUT2 output. OUT2 is inverted then driven out to out2_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_16550_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_modem_enable_out2(soc_16550_HANDLE_t * handle);

/*!
 * Unasserts the OUT2 output. OUT2 is inverted then driven out to out2_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_16550_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_modem_disable_out2(soc_16550_HANDLE_t * handle);

/*!
 * Asserts the RTS (Request To Send) output. RTS is inverted then driven out
 * to rts_n. RTS is used to inform the modem that the UART is ready to receive
 * data.
 *
 * There are special considerations when the UART is in automatic flow control
 * mode. See soc_16550_flowcontrol_enable() for more information.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_16550_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_modem_enable_rts(soc_16550_HANDLE_t * handle);

/*!
 * Deaserts the RTS (Request To Send) output. RTS is inverted then driven out
 * to rts_n.
 *
 * There are special considerations when the UART is in automatic flow control
 * mode. See soc_16550_flowcontrol_enable() for more information.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_16550_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_modem_disable_rts(soc_16550_HANDLE_t * handle);

/*!
 * Asserts the DTR (Data Terminal Ready) output. DTR is inverted then driven
 * out to dtr_n. DTR is used to inform the modem that UART is ready to
 * establish communications.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_16550_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_modem_enable_dtr(soc_16550_HANDLE_t * handle);

/*!
 * Deasserts the DTR (Data Terminal Ready) output. DTR is inverted then driven
 * out to dtr_n.
 *
 * There are special considerations when the UART is in loopback mode. See
 * soc_16550_loopback_enable() for more information.
 *
 * \param       handle
 *              The UART device handle.
 *
 * \retval      soc_E_SUCCESS   The operation was successful.
 * \retval      soc_E_ERROR     The operation failed.
 * \retval      soc_E_BAD_ARG   The given UART device handle is invalid.
 */
soc_STATUS_CODE soc_16550_modem_disable_dtr(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_modem_status_get(soc_16550_HANDLE_t * handle,
                                           uint32_t * status);

/*!
 * @}
 */

/*!
 * \addtogroup UART_LINE UART Line Interface
 *
 * This group of APIs provides access, configuration, and control of the UART
 * Line interface.
 *
 * @{
 */

/*!
 * This type definition enumerates the supported databits per frame.
 */
typedef enum soc_16550_DATABITS_e
{
    /*!
     * This option selects 5 databits per frame.
     */
    soc_16550_DATABITS_5 = 0,

    /*!
     * This option selects 6 databits per frame.
     */
    soc_16550_DATABITS_6 = 1,

    /*!
     * This option selects 7 databits per frame.
     */
    soc_16550_DATABITS_7 = 2,

    /*!
     * This option selects 8 databits per frame.
     */
    soc_16550_DATABITS_8 = 3
}
soc_16550_DATABITS_t;

/*!
 * This type definition enumerates the supported stopbits per frame.
 */
typedef enum soc_16550_STOPBITS_e
{
    /*!
     * This options specifies 1 stopbit per frame.
     */
    soc_16550_STOPBITS_1 = 0,

    /*!
     * This options specifies 2 stopbits per frame. If the frame is
     * configured with 5 databits, 1.5 stopbits is used instead.
     */
    soc_16550_STOPBITS_2 = 1
}
soc_16550_STOPBITS_t;

/*!
 * This type definition enumerates the possible parity to use per frame.
 */
typedef enum soc_16550_PARITY_e
{
    /*!
     * This option disables the parity error detection bit in the data frame.
     */
    soc_16550_PARITY_DISABLE = 0,

    /*!
     * This option enables the odd parity error detection bit in the data
     * frame.
     */
    soc_16550_PARITY_ODD = 1,

    /*!
     * This option enables the even parity error detection bit in the data
     * frame.
     */
    soc_16550_PARITY_EVEN = 2
}
soc_16550_PARITY_t;

/*!
 * This type definition enumerates the set of UART line status conditions as
 * register mask values.
 */
typedef enum soc_16550_LINE_STATUS_e
{
    /*!
     * Receiver FIFO Error. This status indicates that one or more parity
     * error, framing error, or break indication exists in the receiver FIFO.
     * It is only set when FIFO is enabled. This status cleared when line
     * status is read, the character with the issue is at the top of the FIFO,
     * and when no other issues exist in the FIFO.
     */
    soc_16550_LINE_STATUS_RFE = 1 << 7,

    /*!
     * Transmitter EMpTy (Empty). This status indicates that transmitter shift
     * register is empty. If FIFOs are enabled, the status is set when the
     * transmitter FIFO is also empty. This status is cleared when the
     * transmitter shift registers is loaded by writing to the UART
     * transmitter buffer or transmitter FIFO if FIFOs are enabled. This is
     * done by calling soc_16550_write() and soc_16550_fifo_write()
     * respectively.
     */
    soc_16550_LINE_STATUS_TEMT = 1 << 6,

    /*!
     * Transmitter Holding Register Empty. This status indicates that the 
     * transmitter will run out of data soon. The definition of soon depends
     * on whether the FIFOs are enabled.
     *
     * If FIFOs are disabled, this status indicates that the transmitter will
     * run out of data to send after the current transmit shift register
     * completes. In this case, this status is cleared when the data is
     * written to the UART. This can be done by calling soc_16550_write().
     *
     * If FIFOs are enabled, this status indicates that the transmitter FIFO
     * level is below the transmitter trigger level specified. In this case,
     * this status is cleared by writing a sufficiently large buffer to the
     * transmitter FIFO such that the FIFO is filled above the transmitter
     * trigger level specified by calling soc_16550_fifo_write() or by
     * adjusting the transmitter trigger level appropriately by calling 
     * soc_16550_fifo_trigger_set_tx().
     *
     * \internal
     * The implementation of the UART driver always ensures that IER[7] is
     * set. This means that the UART always has Programmable THRE (Transmitter
     * Holding Register Empty) Interrupt Mode Enable (PTIME) enabled.
     * \endinternal
     */
    soc_16550_LINE_STATUS_THRE = 1 << 5,

    /*!
     * Break Interrupt. This status indicates that a break interrupt sequence
     * is detected in the incoming serial data. This happens when the the data
     * is 0 for longer than a frame would normally be transmitted. The break
     * interrupt status is cleared by reading the line status by calling
     * soc_16550_line_status_get().
     *
     * If FIFOs are enabled, this status will be set when the character with
     * the break interrupt status is at the top of the receiver FIFO.
     */
    soc_16550_LINE_STATUS_BI = 1 << 4,

    /*!
     * Framing Error. This status indicates that a framing error occurred in
     * the receiver. This happens when the receiver detects a missing or
     * incorrect number of stopbit(s).
     *
     * If FIFOs are enabled, this status will be set when the character with
     * the framing error is at the top of the FIFO. When a framing error
     * occurs, the UART attempts to resynchronize with the transmitting UART.
     * This status is also set if break interrupt occurred.
     */
    soc_16550_LINE_STATUS_FE = 1 << 3,

    /*!
     * Parity Error. This status indicates that a parity error occurred in the
     * receiver.
     *
     * If FIFOs are enabled, this status will be set when the character with
     * the parity error is at the top of the receiver FIFO. This status is
     * also set if a break interrupt occurred.
     */
    soc_16550_LINE_STATUS_PE = 1 << 2,

    /*!
     * Overrun Error. This status indicates that an overrun occurred in the
     * receiver.
     *
     * If FIFOs are disabled, the arriving character will overwrite the
     * existing character in the receiver. Any previously existing
     * character(s) will be lost.
     *
     * If FIFOs are disabled, the arriving character will be discarded. The
     * buffer will continue to contain the preexisting characters.
     */
    soc_16550_LINE_STATUS_OE = 1 << 1,

    /*!
     * Data Ready. This status indicates that the receiver or receiver FIFO
     * contains at least one character.
     */
    soc_16550_LINE_STATUS_DR = 1 << 0
}
soc_16550_LINE_STATUS_t;

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
soc_STATUS_CODE soc_16550_line_config_set(soc_16550_HANDLE_t * handle,
                                          soc_16550_DATABITS_t databits,
                                          soc_16550_PARITY_t parity,
                                          soc_16550_STOPBITS_t stopbits);

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
soc_STATUS_CODE soc_16550_line_break_enable(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_line_break_disable(soc_16550_HANDLE_t * handle);

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
soc_STATUS_CODE soc_16550_line_status_get(soc_16550_HANDLE_t * handle,
                                          uint32_t * status);

/*!
 * @}
 */

/*!
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* __soc_16550_UART_H__ */
