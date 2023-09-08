/*
    Copyright (C) 2023 Dimitris Mantzouranis

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    UART/hal_uart_lld.h
 * @brief   SN32 low level UART driver header.
 *
 * @addtogroup UART
 * @{
 */

#ifndef HAL_UART_LLD_H
#define HAL_UART_LLD_H

#if HAL_USE_UART || defined(__DOXYGEN__)

#include "sn32_uart.h"

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/** @defgroup UART_Exported_Constants
  * @{
  */

/** @defgroup UART_LineControl
  * @{
  */
#define UART_Break_Control_Disable             (0x0<<6)
#define UART_Break_Control_Enable              (0x1<<6)
#define UART_Divisor_Latch_Access_Disable      (0x0<<7)
#define UART_Divisor_Latch_Access_Enable       (0x1<<7)
#define UART_Parity_None                       (0x0<<3)
#define UART_Parity_Enable                     (0x1<<3)
#define UART_Parity_Odd                        (0x00<<4)
#define UART_Parity_Even                       (0x01<<4)
#define UART_Parity_Mark                       (0x02<<4)
#define UART_Parity_Space                      (0x03<<4)
#define UART_StopBits_One                      (0x0<<2)
#define UART_StopBits_Two                      (0x1<<2)
#define UART_WordLength_5b                     (0x00)
#define UART_WordLength_6b                     (0x01)
#define UART_WordLength_7b                     (0x02)
#define UART_WordLength_8b                     (0x03)
/**
  * @}
  */


/** @defgroup UART_AutoBaudControl
  * @{
  */
#define UART_AutoBaudControl_Start             (0x01)
#define UART_AutoBaudControl_Restart           (UART_AutoBaudControl_Start <<2)
#define UART_AutoBaudControl_End               (UART_AutoBaudControl_Start <<8)
#define UART_AutoBaudControl_Timeout           (UART_AutoBaudControl_Start <<9)
#define UART_ABCTRL_MODE(x)                    ((0x00 + x) << 1)
/**
  * @}
  */


/** @defgroup UART_FIFOControl
  * @{
  */
#define UART_FIFO_Enable                       (0x01)
#define UART_RxFIFO_Reset                      (0x01<<1)
#define UART_TxFIFO_Reset                      (0x01<<2)
#define UART_RxFIFOThreshold_1                 (0x00<<6)
#define UART_RxFIFOThreshold_4                 (0x01<<6)
#define UART_RxFIFOThreshold_8                 (0x02<<6)
#define UART_RxFIFOThreshold_14                (0x03<<6)
/**
  * @}
  */


/** @defgroup UART_FractionalDivider
  * @{
  */
#define UART_Oversample_8                      (0x1<<8)
#define UART_Oversample_16                     (0x0<<8)
#define UART_FD_MULVAL(x)                      ((0x0000 + x) << 4)
#define UART_FD_DIVADDVAL(x)                   (0x000 +x)
/**
  * @}
  */

/** @defgroup UART_InterruptEnable
  * @{
  */
#define UART_InterruptEnable                   (0x01)
#define UART_TxError                           (UART_InterruptEnable <<10)
#define UART_AutoBaudTimeout                   (UART_InterruptEnable <<9)
#define UART_AutoBaudEnd                       (UART_InterruptEnable <<8)
#define UART_TransmitterEmpty                  (UART_InterruptEnable <<4)
#define UART_ModemStatus                       (UART_InterruptEnable <<3)
#define UART_ReceiveLine                       (UART_InterruptEnable <<2)
#define UART_TransmitterHoldingEmpty           (UART_InterruptEnable <<1)
#define UART_ReceiveDataAvailable              (UART_InterruptEnable <<0)
/**
  * @}
  */

/** @defgroup UART_InterruptIdentification
  * @{
  */
#define UART_Interrupt_Pending                 0
#define UART_InterruptID_THRE                  1
#define UART_InterruptID_RDA                   2
#define UART_InterruptID_RLS                   3
#define UART_InterruptID_CTI                   6
#define UART_InterruptID_TEMT                  7
#define UART_Interrupt_TxError                 (0x01<<10)
#define UART_Interrupt_ABTO                    (0x01<<9)
#define UART_Interrupt_ABEO                    (0x01<<8)
#define UART_Interrupt_Status                  (0x01)
#define UART_InterruptID_Status                7

/**
  * @}
  */

/** @defgroup UART_LineStatus
  * @{
  */
#define UART_LineStatus_TxError                (0x01<<8)
#define UART_LineStatus_RxError                (0x01<<7)
#define UART_LineStatus_TEMT                   (0x01<<6)
#define UART_LineStatus_THRE                   (0x01<<5)
#define UART_LineStatus_BI                     (0x01<<4)
#define UART_LineStatus_FE                     (0x01<<3)
#define UART_LineStatus_PE                     (0x01<<2)
#define UART_LineStatus_OE                     (0x01<<1)
#define UART_LineStatus_RDR                    (0x01)

/**
  * @}
  */

/** @defgroup UART_Control
  * @{
  */
#define UART_Enable                            (0x01)
#define UART_RxEnable                          (UART_Enable <<6)
#define UART_TxEnable                          (UART_Enable <<7)
/**
  * @}
  */

/**
 * @}
 */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/**
 * @name    Configuration options
 * @{
 */
/**
 * @brief   UART driver on UART0 enable switch.
 * @details If set to @p TRUE the support for UART0 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SN32_UART_USE_UART0) || defined(__DOXYGEN__)
#define SN32_UART_USE_UART0                    FALSE
#endif

/**
 * @brief   UART driver on UART1 enable switch.
 * @details If set to @p TRUE the support for UART1 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SN32_UART_USE_UART1) || defined(__DOXYGEN__)
#define SN32_UART_USE_UART1                    FALSE
#endif

/**
 * @brief   UART driver on UART2 enable switch.
 * @details If set to @p TRUE the support for UART2 is included.
 * @note    The default is @p FALSE.
 */
#if !defined(SN32_UART_USE_UART2) || defined(__DOXYGEN__)
#define SN32_UART_USE_UART2                    FALSE
#endif

/**
 * @brief   UART0 interrupt priority level setting.
 */
#if !defined(SN32_UART_UART0_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SN32_UART_UART0_IRQ_PRIORITY          3
#endif

/**
 * @brief   UART1 interrupt priority level setting.
 */
#if !defined(SN32_UART_UART1_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SN32_UART_UART1_IRQ_PRIORITY          3
#endif

/**
 * @brief   UART2 interrupt priority level setting.
 */
#if !defined(SN32_UART_UART2_IRQ_PRIORITY) || defined(__DOXYGEN__)
#define SN32_UART_UART2_IRQ_PRIORITY          3
#endif

/** @} */

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

#if SN32_UART_USE_UART0 && !SN32_HAS_UART0
#error "UART0 not present in the selected device"
#endif

#if SN32_UART_USE_UART1 && !SN32_HAS_UART1
#error "UART1 not present in the selected device"
#endif

#if SN32_UART_USE_UART2 && !SN32_HAS_UART2
#error "UART2 not present in the selected device"
#endif

#if !SN32_UART_USE_UART0 && !SN32_UART_USE_UART1 &&                         \
    !SN32_UART_USE_UART2
#error "UART driver activated but no UART/UART peripheral assigned"
#endif

#if SN32_UART_USE_UART0 &&                                                  \
    !OSAL_IRQ_IS_VALID_PRIORITY(SN32_UART_UART0_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to UART0"
#endif

#if SN32_UART_USE_UART1 &&                                                  \
    !OSAL_IRQ_IS_VALID_PRIORITY(SN32_UART_UART1_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to UART1"
#endif

#if SN32_UART_USE_UART2 &&                                                  \
    !OSAL_IRQ_IS_VALID_PRIORITY(SN32_UART_UART2_IRQ_PRIORITY)
#error "Invalid IRQ priority assigned to UART2"
#endif

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

#if SN32_HAS_UART0
#define SN32_UART0_BASE  SN_UART0_BASE
#define SN32_UART0       ((sn32_uart_t *)SN_UART0_BASE)
#endif

#if SN32_HAS_UART1
#define SN32_UART1_BASE  SN_UART1_BASE
#define SN32_UART1       ((sn32_uart_t *)SN_UART1_BASE)
#endif

#if SN32_HAS_UART2
#define SN32_UART2_BASE  SN_UART2_BASE
#define SN32_UART2       ((sn32_uart_t *)SN_UART2_BASE)
#endif

/**
 * @brief   UART driver condition flags type.
 */
typedef uint32_t uartflags_t;

/**
 * @brief   Structure representing an UART driver.
 */
typedef struct UARTDriver UARTDriver;

/**
 * @brief   Generic UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 */
typedef void (*uartcb_t)(UARTDriver *uartp);

/**
 * @brief   Character received UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] c         received character
 */
typedef void (*uartccb_t)(UARTDriver *uartp, uint16_t c);

/**
 * @brief   Receive error UART notification callback type.
 *
 * @param[in] uartp     pointer to the @p UARTDriver object
 * @param[in] e         receive error mask
 */
typedef void (*uartecb_t)(UARTDriver *uartp, uartflags_t e);

typedef struct {
  const uint8_t             *tx_buf;
  volatile uint32_t         tx_len;
  uint8_t                   *rx_buf;
  volatile uint32_t         rx_len;
  uint32_t                  tx_abrt_source;
} uart_xfer_info_t;

/**
 * @brief   Driver configuration structure.
 * @note    It could be empty on some architectures.
 */
typedef struct {
  /**
   * @brief End of transmission buffer callback.
   */
  uartcb_t                  txend1_cb;
  /**
   * @brief Physical end of transmission callback.
   */
  uartcb_t                  txend2_cb;
  /**
   * @brief Receive buffer filled callback.
   */
  uartcb_t                  rxend_cb;
  /**
   * @brief Character received while out if the @p UART_RECEIVE state.
   */
  uartccb_t                 rxchar_cb;
  /**
   * @brief Receive error callback.
   */
  uartecb_t                 rxerr_cb;
  /* End of the mandatory fields.*/
  /**
   * @brief   Receiver timeout callback.
   * @details Handles idle interrupts depending on configured
   *          flags in CR registers and supported hardware features.
   */
  uartcb_t                  timeout_cb;
  /**
   * @brief Controls the clock prescaler for baud rate generation.
   */
  uint32_t                  UART_BaudRate;
  /**
   * @brief Specifies the number of data bits transmitted or received in a frame.
   *        This parameter can be a value of @ref UART_Word_Length
   */
  uint8_t                   UART_WordLength;
  /**
  * @brief Specifies the number of stop bits transmitted.
  *        This parameter can be a value of @ref UART_Stop_Bits
  */
  uint8_t                   UART_StopBits;
  /**
   * @brief Specifies the parity mode.
   *        This parameter can be a value of @ref UART_Parity
   */
  uint8_t                   UART_Parity;
  /**
   * @brief Controls the operation of the UART RX and TX FIFOs.
   */
  uint32_t                  UART_FIFOControl;
  /**
  * @brief Specifies the auto flow control mode and configures functionality.
  */
  uint32_t                  UART_AutoBaudControl;
} UARTConfig;

/**
 * @brief   Structure representing an UART driver.
 */
struct UARTDriver {
  /**
   * @brief Driver state.
   */
  uartstate_t               state;
  /**
   * @brief Transmitter state.
   */
  uarttxstate_t             txstate;
  /**
   * @brief Receiver state.
   */
  uartrxstate_t             rxstate;
  /**
   * @brief Current configuration data.
   */
  const UARTConfig          *config;
#if (UART_USE_WAIT == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Synchronization flag for transmit operations.
   */
  bool                      early;
  /**
   * @brief   Waiting thread on RX.
   */
  thread_reference_t        threadrx;
  /**
   * @brief   Waiting thread on TX.
   */
  thread_reference_t        threadtx;
#endif /* UART_USE_WAIT */
#if (UART_USE_MUTUAL_EXCLUSION == TRUE) || defined(__DOXYGEN__)
  /**
   * @brief   Mutex protecting the peripheral.
   */
  mutex_t                   mutex;
#endif /* UART_USE_MUTUAL_EXCLUSION */
#if defined(UART_DRIVER_EXT_FIELDS)
  UART_DRIVER_EXT_FIELDS
#endif
  /* End of the mandatory fields.*/
  /**
   * @brief Pointer to the UART registers block.
   */
  sn32_uart_t              *uart;
  /**
   * @brief Default receive buffer while into @p UART_RX_IDLE state.
   */
  volatile uint16_t         rxbuf;
  uart_xfer_info_t          xfer;
};

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#if SN32_UART_USE_UART0 && !defined(__DOXYGEN__)
extern UARTDriver UARTD0;
#endif

#if SN32_UART_USE_UART1 && !defined(__DOXYGEN__)
extern UARTDriver UARTD1;
#endif

#if SN32_UART_USE_UART2 && !defined(__DOXYGEN__)
extern UARTDriver UARTD2;
#endif

#ifdef __cplusplus
extern "C" {
#endif
  void uart_lld_init(void);
  void uart_lld_start(UARTDriver *uartp);
  void uart_lld_stop(UARTDriver *uartp);
  void uart_lld_start_send(UARTDriver *uartp, size_t n, const void *txbuf);
  size_t uart_lld_stop_send(UARTDriver *uartp);
  void uart_lld_start_receive(UARTDriver *uartp, size_t n, void *rxbuf);
  size_t uart_lld_stop_receive(UARTDriver *uartp);
#ifdef __cplusplus
}
#endif

#endif /* HAL_USE_UART */

#endif /* HAL_UART_LLD_H */

/** @} */