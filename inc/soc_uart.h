
#ifndef __SOC__UART_H__
#define __SOC__UART_H__



#include "soc_uart.h"
#include "soc_clockManager.h"
#include "socfpgaHAL.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define SOC_UART_CAST(type, ptr) ((type) (ptr))

/*
 *
 *  Base Registers Address
 *
 */
#ifdef SOC_CY_AV
	#define SOC_UART0_BASE (0xFFC02000+FPGA2HPS_BASE_OFFSET-SUB_WINDOW_OFFSET)
	#define SOC_UART1_BASE (0xFFC03000+FPGA2HPS_BASE_OFFSET-SUB_WINDOW_OFFSET)

#endif

/*
 *
 *  Base Registers Masks
 *
 */
#define SOC_UART_LCR_DLAB_SET_MSK    		   0x00000080
#define SOC_UART_IER_DLH_ETBEI_DLHL_SET_MSK    0x00000002
#define SOC_UART_SRR_UR_SET_MSK				   0x00000001
#define SOC_UART_IER_DLH_PTIME_DLH7_SET_MSK    0x00000080
#define SOC_UART_FCR_FIFOE_SET_MSK             0x00000001
#define SOC_UART_FCR_RFIFOR_SET_MSK            0x00000002
#define SOC_UART_FCR_XFIFOR_SET_MSK            0x00000004
#define SOC_UART_SRR_RFR_SET_MSK               0x00000002
#define SOC_UART_SRR_XFR_SET_MSK			   0x00000004
#define SOC_UART_FCR_RT_SET_MSK				   0x000000c0
#define SOC_UART_FCR_TET_SET_MSK			   0x00000030
#define SOC_UART_IER_DLH_ERBFI_DLH0_SET_MSK    0x00000001
#define SOC_UART_IER_DLH_ETBEI_DLH1_SET_MSK    SOC_UART_IER_DLH_ETBEI_DLHL_SET_MSK
#define SOC_UART_IER_DLH_ELSI_DHL2_SET_MSK	   0x00000004
#define SOC_UART_IER_DLH_EDSSI_DHL3_SET_MSK    0x00000008
#define SOC_UART_LCR_PEN_SET_MSK			   0x00000008
#define SOC_UART_LCR_EPS_SET_MSK               0x00000010
#define SOC_UART_LCR_DLS_SET_MSK			   0x00000003
#define SOC_UART_LCR_STOP_SET_MSK              0x00000004
#define SOC_UART_LCR_BREAK_SET_MSK             0x00000040
#define SOC_UART_MCR_AFCE_SET_MSK              0x00000020
#define SOC_UART_MCR_LOOPBACK_SET_MSK		   0x00000010
#define SOC_UART_MCR_OUT1_SET_MSK              0x00000004
#define SOC_UART_MCR_OUT2_SET_MSK              0x00000008
#define SOC_UART_MCR_RTS_SET_MSK               0x00000002
#define SOC_UART_MCR_DTR_SET_MSK			   0x00000001


/*
 *
 *  Register Addresses
 *
 */
#define SOC_UART_LCR_ADDR(base)  SOC_CAST(void *, (SOC_UART_CAST(char *, (base)) + 0xc))
#define SOC_UART_RBR_THR_DLL_VALUE_SET(value) (((value) << 0) & 0x000000ff)
#define SOC_UART_RBR_THR_DLL_ADDR(base)  SOC_UART_CAST(void *, (SOC_UART_CAST(char *, (base)) + 0x0))
#define SOC_UART_IER_DLH_ADDR(base)  SOC_UART_CAST(void *, (SOC_UART_CAST(char *, (base)) + 0x4s))
#define SOC_UART_SRR_ADDR(base)  SOC_UART_CAST(void *, (SOC_UART_CAST(char *, (base)) + 0x88))
#define SOC_UART_FCR_ADDR(base)  SOC_UART_CAST(void *, (SOC_UART_CAST(char *, (base)) + 0x8))

#define SOC_UART_RFL_ADDR(base)  SOC_UART_CAST(void *, (SOC_UART_CAST(char *, (base)) + 0x84))
#define SOC_UART_MCR_ADDR(base)  ALT_CAST(void *, (ALT_CAST(char *, (base)) + 0x10))

/*
 *
 * Const values and Reset values
 *
 */
#define SOC_UART_UCV_UART_COMPONENT_VER_RESET      0x3331312a


/*!
 * This type definition enumerates the list of UARTs available on the system.
 */
typedef enum
{
 // Select the HPS UART0 (offten used for Linux Console output)
#if SOCFPGAHAL_ENABLE_UART0 == 1
    SOC_UART_DEVICE_SOCFPGA_UART0 = 0,
#endif

// Select the HPS UART1
#if SOCFPGAHAL_ENABLE_UART1 == 1
    SOC_UART_DEVICE_SOCFPGA_UART1 = 1,
#endif
// Use a Intel (ALTERA) UART Soft-IP module
#if SOCFPGAHAL_ENABLE_UART_SOFTIP ==1
    SOC_UART_DEVICE_INTEL_IP = 0x100
#endif
}
SOC_UART_DEVICE_t;


typedef struct
{
    SOC_UART_DEVICE_t  device;
    void *             location;
    soc_freq_t         clock_freq;
    uint32_t           data;
    uint32_t           fcr;
}
SOC_UART_HANDLE_t;

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
typedef enum SOC_UART_FIFO_TRIGGER_RX_e
{
    /*!
     * 1 or more character(s) in the receiver FIFO will trigger an event.
     */
    SOC_UART_FIFO_TRIGGER_RX_ANY = 0,

    /*!
     * 25% or higher capacity usage in the receiver FIFO will trigger an
     * event.
     */
    SOC_UART_FIFO_TRIGGER_RX_QUARTER_FULL = 1,

    /*!
     * 50% or higher capacity usage in the receiver FIFO will trigger an
     * event.
     */
    SOC_UART_FIFO_TRIGGER_RX_HALF_FULL = 2,

    /*!
     * 2 characters less than the receiver FIFO capacity will trigger an
     * event.
     */
    SOC_UART_FIFO_TRIGGER_RX_ALMOST_FULL = 3
}
SOC_UART_FIFO_TRIGGER_RX_t;

/*!
 * This type definition enumerates the transmitter FIFO level conditions that
 * will trigger the transmitter FIFO to issue a transmitter FIFO empty event.
 */
typedef enum SOC_UART_FIFO_TRIGGER_TX_e
{
    /*!
     * Transmitter FIFO being completely empty will trigger an event.
     */
    SOC_UART_FIFO_TRIGGER_TX_EMPTY = 0,

    /*!
     * 2 or less character(s) in the transmitter FIFO will trigger an event.
     */
    SOC_UART_FIFO_TRIGGER_TX_ALMOST_EMPTY = 1,

    /*!
     * 25% or less capacity usage in the transmitter FIFO will trigger an
     * event.
     */
    SOC_UART_FIFO_TRIGGER_TX_QUARTER_FULL = 2,

    /*!
     * 50% or less capacity usage in the transmitter FIFO will trigger an
     * event.
     */
    SOC_UART_FIFO_TRIGGER_TX_HALF_FULL = 3
}
SOC_UART_FIFO_TRIGGER_TX_t;
/*
*!
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
typedef enum SOC_UART_BAUDRATE_e
{
    SOC_UART_BAUDRATE_50     =     50, /*!< 50 bps baudrate. */
    SOC_UART_BAUDRATE_75     =     75, /*!< 75 bps baudrate. */
    SOC_UART_BAUDRATE_150    =    150, /*!< 150 bps baudrate. */
    SOC_UART_BAUDRATE_300    =    300, /*!< 300 bps baudrate. */
    SOC_UART_BAUDRATE_600    =    600, /*!< 600 bps baudrate. */
    SOC_UART_BAUDRATE_900    =    900, /*!< 900 bps baudrate. */
    SOC_UART_BAUDRATE_1200   =   1200, /*!< 1200 bps baudrate. */
    SOC_UART_BAUDRATE_1800   =   1800, /*!< 1800 bps baudrate. */
    SOC_UART_BAUDRATE_2400   =   2400, /*!< 2400 bps baudrate. */
    SOC_UART_BAUDRATE_3600   =   3600, /*!< 3600 bps baudrate. */
    SOC_UART_BAUDRATE_4800   =   4800, /*!< 4800 bps baudrate. */
    SOC_UART_BAUDRATE_7200   =   7200, /*!< 7200 bps baudrate. */
    SOC_UART_BAUDRATE_9600   =   9600, /*!< 9600 bps baudrate. */
    SOC_UART_BAUDRATE_14400  =  14400, /*!< 14400 bps baudrate. */
    SOC_UART_BAUDRATE_19200  =  19200, /*!< 19200 bps baudrate. */
    SOC_UART_BAUDRATE_28800  =  28800, /*!< 28800 bps baudrate. */
    SOC_UART_BAUDRATE_38400  =  38400, /*!< 38400 bps baudrate. */
    SOC_UART_BAUDRATE_57600  =  57600, /*!< 57600 bps baudrate. */
    SOC_UART_BAUDRATE_115200 = 115200  /*!< 115200 bps baudrate. */
}SOC_UART_BAUDRATE_t;

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
typedef enum SOC_UART_INT_STATUS_e
{
    /*!
     * This interrupt signals that a overrun, parity, or framing error 
     * occurred, or a break event occured. The interrupt is cleared by reading
     * the line status by calling soc_uart_line_status_get() or by disabling
     * line status interrupts by calling soc_uart_int_disable_line().
     */
    SOC_UART_INT_STATUS_LINE = 0x6,

    /*!
     * This interrupt signals that some data is available to be read from the
     * UART. The definition of some depends on whether FIFOs are enabled or
     * not.
     *
     * If FIFOs are disabled, this interrupt signals that the receiver
     * contains data. In this case, the interrupt is cleared by reading the
     * data from the UART by calling soc_uart_read().
     *
     * If FIFOs are enabled, this interrupt signals that the receiver FIFO
     * level is above the receiver trigger level specified. In this case, the
     * interrupt is cleared by reading a sufficiently large buffer from the
     * receiver FIFO such that the FIFO is filled below the receiver trigger
     * level specified by calling soc_uart_fifo_read() or by adjusting the
     * receiver trigger level appropriately by calling
     * soc_uart_fifo_trigger_set_rx().
     *
     * In either case, this interrupt can also be cleared by disabling
     * receiver interrupts by calling soc_uart_int_disable_rx().
     */
    SOC_UART_INT_STATUS_RX_DATA = 0x4,

    /*!
     * This interrupt signals that data is available in the receiver FIFO and
     * that there has been no activity with the receiver FIFO for the last 4
     * character frames. In essence, the receiver FIFO has temporarily settled
     * thus it may be a good time to empty the receiver FIFO. This interrupt
     * is only available if FIFOs are enabled. The interrupt is cleared by
     * reading from the receiver FIFO by calling soc_uart_fifo_read() or by
     * disabling receiver interrupts by calling soc_uart_int_disable_rx().
     */
    SOC_UART_INT_STATUS_RX_TIMEOUT = 0xC,

    /*!
     * This interrupt signals that the transmitter is idling. The definition
     * of idling depends on whether FIFOs are enabled or not.
     *
     * If FIFOs are disabled, this interrupt signals that the transmitter
     * shift register is empty. In this case, the interrupt is cleared by
     * writing data to the UART by calling soc_uart_write().
     *
     * If FIFO are enabled, this interrupt signals that the transmitter FIFO
     * level is below the transmitter trigger level specified. In this case,
     * the interrupt is cleared by writing a sufficiently large buffer to the
     * transmitter FIFO such that the FIFO is filled above the transmitter
     * trigger level specified by calling soc_uart_fifo_write() or by
     * adjusting the transmitter trigger level appropriately by calling
     * soc_uart_fifo_trigger_set_tx().
     *
     * In either case, this interrupt can also be cleared by disabling
     * transmitter interrupts by calling soc_uart_int_disable_tx().
     */
    SOC_UART_INT_STATUS_TX_IDLE = 0x2,

    /*!
     * Modem status interrupt pending. The interrupt is cleared by reading the
     * modem status by calling soc_uart_modem_status_get() or by disabling
     * modem status interrupts by calling soc_uart_int_disable_modem().
     */
    SOC_UART_INT_STATUS_MODEM = 0x0,

    /*!
     * No interrupts pending.
     */
    SOC_UART_INT_STATUS_NONE = 0x1
}
SOC_UART_INT_STATUS_t;


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
typedef enum SOC_UART_MODEM_STATUS_e
{
    /*!
     * Data Carrier Detect. This status indicates that the carrier has been
     * detected by the modem. It corresponds to an inverted dcd_n input. DCD
     * is unasserted when dcd_n is logic 1 and asserted when dcd_n is logic 0.
     */
    SOC_UART_MODEM_STATUS_DCD = 1 << 7,

    /*!
     * Ring Indicator. This status indicates that the telephone ringing signal
     * has been redeived by the modem. It corresponds to an inverted ri_n
     * input. RI is unasserted when ri_n is logic 1 and asserted when ri_n is
     * logic 0.
     */
    SOC_UART_MODEM_STATUS_RI = 1 << 6,

    /*!
     * Data Set Ready. This status indicates that the modem is ready to
     * establish communications with the UART. It corresponds to an inverted
     * dsr_n input. DSR is unasserted when dsr_n is logic 1 and asserted when
     * dsr_n is logic 0.
     */
    SOC_UART_MODEM_STATUS_DSR = 1 << 5,

    /*!
     * Clear To Send. This status indicates the current state of the modem
     * cts_n line. It corresponds to an inverted cts_n input. CTS is
     * unasserted when cts_n is logic 1 and asserted when cts_n is logic 0.
     */
    SOC_UART_MODEM_STATUS_CTS = 1 << 4,

    /*!
     * Delta Data Carrier Detect. This status condition indicates that the
     * Data Carrier Detect has changed since the last time the modem status
     * was read. Reading the modem status clears this status. For more
     * information about the Data Carrier Detect status, see
     * SOC_UART_MODEM_STATUS_DCD.
     */
    SOC_UART_MODEM_STATUS_DDCD = 1 << 3,

    /*!
     * Trailing Edge of Ring Indicator. This status indicates that the Ring
     * Indicator has changed from asserted to unasserted. Reading the modem
     * status will clear this status. For more information about the Ring
     * Indicator status, reference SOC_UART_MODEM_STATUS_RI.
     */
    SOC_UART_MODEM_STATUS_TERI = 1 << 2,

    /*!
     * Delta Data Set Ready. This status condition indicates that the Data Set
     * Ready has changed since the last time the modem status was read.
     * Reading the modem status will clear this status. For more information
     * about the Data Set Ready status, see SOC_UART_MODEM_STATUS_DSR.
     */
    SOC_UART_MODEM_STATUS_DDSR = 1 << 1,

    /*!
     * Delta Clear To Send. This status condition indicates that the Clear To
     * Send has changed since the last time the modem status was read. Reading
     * the modem status will clear this status. For more information about the
     * Clear To Send status, see SOC_UART_MODEM_STATUS_CTS.
     */
    SOC_UART_MODEM_STATUS_DCTS = 1 << 0
}
SOC_UART_MODEM_STATUS_t;

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
typedef enum SOC_UART_DATABITS_e
{
    /*!
     * This option selects 5 databits per frame.
     */
    SOC_UART_DATABITS_5 = 0,

    /*!
     * This option selects 6 databits per frame.
     */
    SOC_UART_DATABITS_6 = 1,

    /*!
     * This option selects 7 databits per frame.
     */
    SOC_UART_DATABITS_7 = 2,

    /*!
     * This option selects 8 databits per frame.
     */
    SOC_UART_DATABITS_8 = 3
}
SOC_UART_DATABITS_t;

/*!
 * This type definition enumerates the supported stopbits per frame.
 */
typedef enum SOC_UART_STOPBITS_e
{
    /*!
     * This options specifies 1 stopbit per frame.
     */
    SOC_UART_STOPBITS_1 = 0,

    /*!
     * This options specifies 2 stopbits per frame. If the frame is
     * configured with 5 databits, 1.5 stopbits is used instead.
     */
    SOC_UART_STOPBITS_2 = 1
}
SOC_UART_STOPBITS_t;

/*!
 * This type definition enumerates the possible parity to use per frame.
 */
typedef enum SOC_UART_PARITY_e
{
    /*!
     * This option disables the parity error detection bit in the data frame.
     */
    SOC_UART_PARITY_DISABLE = 0,

    /*!
     * This option enables the odd parity error detection bit in the data
     * frame.
     */
    SOC_UART_PARITY_ODD = 1,

    /*!
     * This option enables the even parity error detection bit in the data
     * frame.
     */
    SOC_UART_PARITY_EVEN = 2
}
SOC_UART_PARITY_t;

/*!
 * This type definition enumerates the set of UART line status conditions as
 * register mask values.
 */
typedef enum SOC_UART_LINE_STATUS_e
{
    /*!
     * Receiver FIFO Error. This status indicates that one or more parity
     * error, framing error, or break indication exists in the receiver FIFO.
     * It is only set when FIFO is enabled. This status cleared when line
     * status is read, the character with the issue is at the top of the FIFO,
     * and when no other issues exist in the FIFO.
     */
    SOC_UART_LINE_STATUS_RFE = 1 << 7,

    /*!
     * Transmitter EMpTy (Empty). This status indicates that transmitter shift
     * register is empty. If FIFOs are enabled, the status is set when the
     * transmitter FIFO is also empty. This status is cleared when the
     * transmitter shift registers is loaded by writing to the UART
     * transmitter buffer or transmitter FIFO if FIFOs are enabled. This is
     * done by calling soc_uart_write() and soc_uart_fifo_write()
     * respectively.
     */
    SOC_UART_LINE_STATUS_TEMT = 1 << 6,

    /*!
     * Transmitter Holding Register Empty. This status indicates that the 
     * transmitter will run out of data soon. The definition of soon depends
     * on whether the FIFOs are enabled.
     *
     * If FIFOs are disabled, this status indicates that the transmitter will
     * run out of data to send after the current transmit shift register
     * completes. In this case, this status is cleared when the data is
     * written to the UART. This can be done by calling soc_uart_write().
     *
     * If FIFOs are enabled, this status indicates that the transmitter FIFO
     * level is below the transmitter trigger level specified. In this case,
     * this status is cleared by writing a sufficiently large buffer to the
     * transmitter FIFO such that the FIFO is filled above the transmitter
     * trigger level specified by calling soc_uart_fifo_write() or by
     * adjusting the transmitter trigger level appropriately by calling 
     * soc_uart_fifo_trigger_set_tx().
     *
     * \internal
     * The implementation of the UART driver always ensures that IER[7] is
     * set. This means that the UART always has Programmable THRE (Transmitter
     * Holding Register Empty) Interrupt Mode Enable (PTIME) enabled.
     * \endinternal
     */
    SOC_UART_LINE_STATUS_THRE = 1 << 5,

    /*!
     * Break Interrupt. This status indicates that a break interrupt sequence
     * is detected in the incoming serial data. This happens when the the data
     * is 0 for longer than a frame would normally be transmitted. The break
     * interrupt status is cleared by reading the line status by calling
     * soc_uart_line_status_get().
     *
     * If FIFOs are enabled, this status will be set when the character with
     * the break interrupt status is at the top of the receiver FIFO.
     */
    SOC_UART_LINE_STATUS_BI = 1 << 4,

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
    SOC_UART_LINE_STATUS_FE = 1 << 3,

    /*!
     * Parity Error. This status indicates that a parity error occurred in the
     * receiver.
     *
     * If FIFOs are enabled, this status will be set when the character with
     * the parity error is at the top of the receiver FIFO. This status is
     * also set if a break interrupt occurred.
     */
    SOC_UART_LINE_STATUS_PE = 1 << 2,

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
    SOC_UART_LINE_STATUS_OE = 1 << 1,

    /*!
     * Data Ready. This status indicates that the receiver or receiver FIFO
     * contains at least one character.
     */
    SOC_UART_LINE_STATUS_DR = 1 << 0
}
SOC_UART_LINE_STATUS_t;

///////////////////////////////////////////////////////////////////////////////////
/*
 *
 *  FUNCTIONS
 *
 */
///////////////////////////////////////////////////////////////////////////////////

soc_status_t soc_uart_init(SOC_UART_DEVICE_t device,
                               void * location,
                               soc_freq_t clock_freq,
                               SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_uninit(SOC_UART_HANDLE_t * handle);

soc_status_t soc_uart_reset(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_enable(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_disable(SOC_UART_HANDLE_t * handle);


soc_status_t soc_uart_read(SOC_UART_HANDLE_t * handle,
                               char * item);
soc_status_t soc_uart_write(SOC_UART_HANDLE_t * handle,
                                char item);


soc_status_t soc_uart_fifo_enable(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_fifo_disable(SOC_UART_HANDLE_t * handle);


soc_status_t soc_uart_fifo_read(SOC_UART_HANDLE_t * handle,
                                    char * buffer,
                                    size_t count);
soc_status_t soc_uart_fifo_write(SOC_UART_HANDLE_t * handle,
                                     const char * buffer,
                                     size_t count);
soc_status_t soc_uart_fifo_write_safe(SOC_UART_HANDLE_t * handle,
                                     const char * buffer,
                                     size_t count,
                                     bool safe);

soc_status_t soc_uart_fifo_clear_rx(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_fifo_clear_tx(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_fifo_clear_all(SOC_UART_HANDLE_t * handle);


soc_status_t soc_uart_fifo_size_get_rx(SOC_UART_HANDLE_t * handle,
                                           uint32_t * size);
soc_status_t soc_uart_fifo_size_get_tx(SOC_UART_HANDLE_t * handle,
                                           uint32_t * size);


soc_status_t soc_uart_fifo_level_get_rx(SOC_UART_HANDLE_t * handle,
                                            uint32_t * level);
soc_status_t soc_uart_fifo_level_get_tx(SOC_UART_HANDLE_t * handle,
                                            uint32_t * level);


soc_status_t soc_uart_fifo_trigger_set_rx(SOC_UART_HANDLE_t * handle,
                                              SOC_UART_FIFO_TRIGGER_RX_t trigger);
soc_status_t soc_uart_fifo_trigger_set_tx(SOC_UART_HANDLE_t * handle,
                                              SOC_UART_FIFO_TRIGGER_TX_t trigger);

soc_status_t soc_uart_baudrate_get(SOC_UART_HANDLE_t * handle,
                                       uint32_t * baudrate);
soc_status_t soc_uart_baudrate_set(SOC_UART_HANDLE_t * handle,
                                       uint32_t baudrate);

soc_status_t soc_uart_divisor_get(SOC_UART_HANDLE_t * handle,
                                      uint32_t * divisor);
soc_status_t soc_uart_divisor_set(SOC_UART_HANDLE_t * handle,
                                      uint32_t divisor);

soc_status_t soc_uart_int_enable_rx(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_int_disable_rx(SOC_UART_HANDLE_t * handle);


soc_status_t soc_uart_int_enable_tx(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_int_disable_tx(SOC_UART_HANDLE_t * handle);

soc_status_t soc_uart_int_enable_line(SOC_UART_HANDLE_t * handle);


soc_status_t soc_uart_int_disable_line(SOC_UART_HANDLE_t * handle);


soc_status_t soc_uart_int_enable_modem(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_int_disable_modem(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_int_disable_all(SOC_UART_HANDLE_t * handle);


soc_status_t  soc_uart_int_status_get(SOC_UART_HANDLE_t * handle,
                                          SOC_UART_INT_STATUS_t * status);

soc_status_t soc_uart_flowcontrol_enable(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_flowcontrol_disable(SOC_UART_HANDLE_t * handle);

soc_status_t soc_uart_loopback_enable(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_loopback_disable(SOC_UART_HANDLE_t * handle);

soc_status_t soc_uart_modem_enable_out1(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_modem_disable_out1(SOC_UART_HANDLE_t * handle);

soc_status_t soc_uart_modem_enable_out2(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_modem_disable_out2(SOC_UART_HANDLE_t * handle);

soc_status_t soc_uart_modem_enable_rts(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_modem_disable_rts(SOC_UART_HANDLE_t * handle);


soc_status_t soc_uart_modem_enable_dtr(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_modem_disable_dtr(SOC_UART_HANDLE_t * handle);

soc_status_t soc_uart_modem_status_get(SOC_UART_HANDLE_t * handle,
                                           uint32_t * status);

soc_status_t soc_uart_line_config_set(SOC_UART_HANDLE_t * handle,
                                          SOC_UART_DATABITS_t databits,
                                          SOC_UART_PARITY_t parity,
                                          SOC_UART_STOPBITS_t stopbits);


soc_status_t soc_uart_line_break_enable(SOC_UART_HANDLE_t * handle);
soc_status_t soc_uart_line_break_disable(SOC_UART_HANDLE_t * handle);

soc_status_t soc_uart_line_status_get(SOC_UART_HANDLE_t * handle,
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

#endif /* __SOC_UART_UART_H__ */
