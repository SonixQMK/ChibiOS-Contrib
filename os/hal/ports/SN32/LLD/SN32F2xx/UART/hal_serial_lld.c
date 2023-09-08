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
 * @file    UART/hal_serial_lld.c
 * @brief   SN32 low level serial driver code.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "hal.h"

#if HAL_USE_SERIAL || defined(__DOXYGEN__)

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/** @brief UART0 serial driver identifier.*/
#if SN32_SERIAL_USE_UART0 || defined(__DOXYGEN__)
SerialDriver SD0;
#endif

/** @brief UART1 serial driver identifier.*/
#if SN32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
SerialDriver SD1;
#endif

/** @brief UART2 serial driver identifier.*/
#if SN32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
SerialDriver SD2;
#endif

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/* Driver default configuration.*/
static const SerialConfig default_config = {SERIAL_DEFAULT_BITRATE,
                                            UART_WordLength_8b,
                                            UART_StopBits_One,
                                            UART_Parity_None,
                                            (UART_FIFO_Enable | UART_RxFIFOThreshold_1),
                                            UART_AutoBaudControl_None};

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

/**
 * @brief   UART initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration
 */
static void uart_init(SerialDriver *sdp, const SerialConfig *config) {
  uint32_t divider, apbclock;
  uint8_t dlm, dll, divaddval, mulval, oversampling;
  UART_TypeDef *u = sdp->uart;

  u->ABCTRL = config->UART_AutoBaudControl;

  apbclock = SN32_HCLK;

#if defined(UART_OVER8)
  oversampling = 8;
#else
  oversampling = 16;
#endif

  // Check constraints based on oversampling value
  if (oversampling == OVERSAMPLING_8) {
      chDbgAssert(oversampling * config->speed <= apbclock / 8,
                  "Invalid oversampling configuration for requested baud rate");
  } else if (oversampling == OVERSAMPLING_16) {
      chDbgAssert(oversampling * config->speed <= apbclock / 16,
                  "Invalid oversampling configuration for requested baud rate");
  }

  // Calculate divider
  uint32_t rounded_sum = apbclock + (config->speed >> 1);
  divider = rounded_sum / (config->speed * oversampling);

  // Calculate DLM and DLL
  dlm = (uint8_t)(divider >> 8);
  dll = (uint8_t)(divider & 0xFF);

  // Check and adjust DLL value if needed
  if (divaddval > 0 && dlm == 0 && dll < 3) {
      dll = 3;  // Set to the minimum value
  }

  // Calculate DIVADDVAL and MULVAL
  uint32_t fractional_part = (rounded_sum % (baudrate * oversampling)) *
            (config->UART_WordLength + 1) * oversampling;
  divaddval = (uint8_t)((fractional_part >> 4) & 0x0F);

  // Check and adjust MULVAL if needed
  mulval = (uint8_t)(fractional_part & 0x0F);
  if (mulval - divaddval == 2) {
      mulval++;  // Adjust mulval to satisfy the condition
  }

  // Calculate fractional part
  uint32_t fractional_part = (rounded_sum % (config->speed *
         oversampling)) * (config->UART_WordLength + 1) * oversampling;
  divaddval = (uint8_t)((fractional_part >> 4) & 0x0F);
  mulval = (uint8_t)(fractional_part & 0x0F);

  // Update the registers
  u->LC |= UART_Divisor_Latch_Access_Enable;
  u->DLL = dll;
  u->DLM = dlm;
  u->FD_b.DIVADDVAL = divaddval;
  u->FD_b.MULVAL = mulval;
  u->FD_b.OVER8 = (oversampling == UART_Oversample_8) ? 1 : 0;
  u->LC &= ~(UART_Divisor_Latch_Access_Enable);

  u->LC = (config->UART_WordLength | config->UART_StopBits |
             config->UART_Parity);

  // Set RX trigger level
  u->FIFOCTRL |= config->UART_FIFOControl;

  // Reset FIFO and enable
  u->FIFOCTRL |= (UART_TxFIFO_Reset | UART_RxFIFO_Reset | UART_FIFO_Enable);

  // Enable UART
  u->CTRL = (UART_TxEnable | UART_RxEnable | UART_Enable);

  /* Note that some bits are enforced.*/
  u->IE |= (UART_ReceiveDataAvailable | UART_ReceiveLine);

  /* Deciding mask to be applied on the data register on receive,
     this is required in order to mask out the parity bit.*/
  sdp->rxmask = 0xFF;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        UART SR register value
 */
static void set_error(SerialDriver *sdp, uint32_t ls) {
  eventflags_t sts = 0;

  if (ls & UART_LineStatus_OE)
    sts |= SD_OVERRUN_ERROR;
  if (ls & UART_LineStatus_PE)
    sts |= SD_PARITY_ERROR;
  if (ls & UART_LineStatus_FE)
    sts |= SD_FRAMING_ERROR;
  if (ls & UART_LineStatus_BI)
    sts |= SD_BREAK_DETECTED;
  if (sr & UART_LineStatus_RDR)
    sts |= SD_NOISE_ERROR;
  chnAddFlagsI(sdp, sts);
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
#define UART_LS_STATUS (UART_LineStatus_PE | UART_LineStatus_FE | UART_LineStatus_OE | UART_LineStatus_RDR)
  UART_TypeDef *u = sdp->uart;
  uint8_t ls;

  ls = (uint8_t)u->LS;

  /* Special case, LIN break detection.*/
  if ((ls & UART_LineStatus_BI) != RESET) {
    osalSysLockFromISR();
    chnAddFlagsI(sdp, SD_BREAK_DETECTED);
    osalSysUnlockFromISR();
  }

  /* Data available.*/
  osalSysLockFromISR();
  while ((ls & UART_SR_STATUS) | ((ls & UART_LineStatus_RDR) != RESET)) {
    uint8_t b;

    /* Error condition detection.*/
    if (ls & UART_SR_STATUS)
      set_error(sdp, ls);
    b = (uint8_t)u->RB & sdp->rxmask;
    if ((ls & UART_LineStatus_RDR) != RESET)
      sdIncomingDataI(sdp, b);
    ls = (uint32_t)u->LS;
  }
  osalSysUnlockFromISR();

  /* Transmission buffer empty.*/
  if (((u->IE & UART_TransmitterHoldingEmpty) != RESET)
      && (ls & UART_LineStatus_THRE) != RESET) {
    msg_t b;
    osalSysLockFromISR();
    b = oqGetI(&sdp->oqueue);
    if (b < MSG_OK) {
      chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
      u->IE &= ~(UART_TransmitterHoldingEmpty);
    }
    else
      u->TH = b;
    osalSysUnlockFromISR();
  }
  /* Physical transmission end.*/
  if (((u->IE & UART_TransmitterHoldingEmpty) != RESET)
      && (ls & UART_LineStatus_TEMT) != RESET) {
    osalSysLockFromISR();
    if (oqIsEmptyI(&sdp->oqueue)) {
      chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
    }
    osalSysUnlockFromISR();
  }
}

#if SN32_SERIAL_USE_UART0 || defined(__DOXYGEN__)
static void notify0(io_queue_t *qp) {

  (void)qp;
  UART0->IE |= UART_TransmitterHoldingEmpty;
}
#endif

#if SN32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  UART1->IE |= UART_TransmitterHoldingEmpty;
}
#endif

#if SN32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  UART2->IE |= UART_TransmitterHoldingEmpty;
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SN32_SERIAL_USE_UART0 || defined(__DOXYGEN__)
#if !defined(SN32_UART0_IRQ_VECTOR)
#error "SN32_UART0_IRQ_VECTOR not defined"
#endif
/**
 * @brief   UART0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_UART0_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SN32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
#if !defined(SN32_UART1_IRQ_VECTOR)
#error "SN32_UART1_IRQ_VECTOR not defined"
#endif
/**
 * @brief   UART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_UART1_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SN32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
#if !defined(SN32_UART2_IRQ_VECTOR)
#error "SN32_UART2_IRQ_VECTOR not defined"
#endif
/**
 * @brief   UART2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_UART2_IRQ_VECTOR) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD2);

  OSAL_IRQ_EPILOGUE();
}
#endif

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

#if SN32_SERIAL_USE_UART0
  sdObjectInit(&SD0, NULL, notify0);
  SD0.uart = UART0;
#endif

#if SN32_SERIAL_USE_UART1
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = UART1;
#endif

#if SN32_SERIAL_USE_UART2
  sdObjectInit(&SD2, NULL, notify2);
  SD2.uart = UART2;
#endif
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if SN32_SERIAL_USE_UART0
    if (&SD0 == sdp) {
      /* UART0 clock enable.*/
      sys1EnableUART0();
      nvicClearPending(SN32_UART0_NUMBER);
      nvicEnableVector(SN32_UART0_NUMBER, SN32_SERIAL_UART0_PRIORITY);
    }
#endif
#if SN32_SERIAL_USE_UART1
    if (&SD1 == sdp) {
      /* UART1 clock enable.*/
      sys1EnableUART1();
      nvicClearPending(SN32_UART1_NUMBER);
      nvicEnableVector(SN32_UART1_NUMBER, SN32_SERIAL_UART1_PRIORITY);
    }
#endif
#if SN32_SERIAL_USE_UART2
    if (&SD2 == sdp) {
      /* UART2 clock enable.*/
      sys1EnableUART2();
      nvicClearPending(SN32_UART2_NUMBER);
      nvicEnableVector(SN32_UART2_NUMBER, SN32_SERIAL_UART2_PRIORITY);
    }
#endif
  }
  uart_init(sdp, config);
}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the UART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
#if SN32_SERIAL_USE_UART0
    if (&SD0 == sdp) {
      /* UART0 DeInit.*/
      sys1DisableUART0();
      nvicDisableVector(SN32_UART0_NUMBER);
      return;
    }
#endif
#if SN32_SERIAL_USE_UART1
    if (&SD1 == sdp) {
      /* UART1 DeInit.*/
      sys1DisableUART1();
      nvicDisableVector(SN32_UART1_NUMBER);
      return;
    }
#endif
#if SN32_SERIAL_USE_UART2
    if (&SD2 == sdp) {
      /* UART2 DeInit.*/
      sys1DisableUART2();
      nvicDisableVector(SN32_UART2_NUMBER);
      return;
    }
#endif
  }
}

#endif /* HAL_USE_SERIAL */

/** @} */
