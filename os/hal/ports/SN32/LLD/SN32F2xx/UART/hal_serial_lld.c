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
#include "matrix.h"
#include "print.h"
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
  //uint32_t divider, apbclock;
  uint32_t apbclock;
  uint8_t dlm, dll, divaddval, mulval, oversampling;
  sn32_uart_t *u = sdp->uart;

  u->ABCTRL = config->UART_AutoBaudControl;

  apbclock = SN32_HCLK;

#if defined(UART_OVER8)
  oversampling = 8;
#else
  oversampling = 16;
#endif

  // Check constraints based on oversampling value
  if (oversampling == 8) {
      chDbgAssert(oversampling * config->speed <= apbclock / 8,
                  "Invalid oversampling configuration for requested baud rate");
  } else if (oversampling == 16) {
      chDbgAssert(oversampling * config->speed <= apbclock / 16,
                  "Invalid oversampling configuration for requested baud rate");
  }
/*
  // Calculate divider
  uint32_t rounded_sum = apbclock + (config->speed >> 1);
  divider = rounded_sum / (config->speed * oversampling);

  // Calculate DLM and DLL
  dlm = (uint8_t)(divider >> 8);
  dll = (uint8_t)(divider & 0xFF);

  // Calculate fractional part
  uint32_t fractional_part = (rounded_sum % (config->speed * oversampling));

  // Calculate DIVADDVAL and MULVAL
  divaddval = (uint8_t)((fractional_part >> 4) & 0x0F);
  mulval = (uint8_t)(fractional_part & 0x0F);

  // Check and adjust DLL value if needed
  if (divaddval > 0 && dlm == 0 && dll < 3) {
      dll = 3;  // Set to the minimum value
  }

  // Check and adjust MULVAL if needed
  if (mulval - divaddval == 2) {
      mulval++;  // Adjust mulval to satisfy the condition
  }
*/
  dll=200;
  dlm=0;
  divaddval=1;
  mulval=5;
  // Update the registers
  //u->LC |= UART_Divisor_Latch_Access_Enable;
  u->LC_b.DLAB = 1;
//  if(u->LC_b.DLAB) writePinHigh(B1);
  u->DLL_b.DLL = dll;
  u->DLM_b.DLM = dlm;
  u->FD = (UART_FD_MULVAL(mulval) | UART_FD_DIVADDVAL(divaddval));
  u->FD_b.OVER8 = (oversampling == 8) ? 1 : 0;
  //u->LC &= ~(UART_Divisor_Latch_Access_Enable);
  u->LC_b.DLAB = 0;

  u->LC = (config->UART_WordLength | config->UART_StopBits |
             config->UART_Parity);

  // Set RX trigger level
  u->FIFOCTRL_b.RXTL = config->UART_FIFOControl;
  u->ABCTRL |= (UART_AutoBaudControl_Timeout | UART_AutoBaudControl_End);

  // Reset FIFO and enable
  u->FIFOCTRL |= (UART_TxFIFO_Reset | UART_RxFIFO_Reset | UART_FIFO_Enable);

  // Enable UART
  //u->CTRL = (UART_TxEnable | UART_RxEnable | UART_Enable);

  /* Note that some bits are enforced.*/
  u->IE |= (UART_ReceiveDataAvailable | UART_ReceiveLine);
  u->CTRL_b.TXEN = 1;
  u->CTRL_b.RXEN = 1;
  u->CTRL_b.UARTEN = 1;
  u->IE_b.TEMTIE = 1;
  u->IE_b.RLSIE = 1;
  u->IE_b.THREIE = 1;
  u->IE_b.RDAIE = 1;
  u->IE_b.ABTOIE = 0;
  u->IE_b.ABEOIE = 0;

  /* Deciding mask to be applied on the data register on receive,
     this is required in order to mask out the parity bit.*/
  sdp->rxmask = 0x7F; // 8E2
}

/**
 * @brief   UART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(sn32_uart_t *u) {

  u->FIFOCTRL_b.FIFOEN =0;
  //u->LC_b.BC=1;
  u->CTRL =0;
}

/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        UART SR register value
 */
static void set_error(SerialDriver *sdp, uint8_t ls) {
  eventflags_t sts = 0;

  if (ls & UART_LineStatus_PE)
    sts |= SD_PARITY_ERROR;
  if (ls & UART_LineStatus_FE)
    sts |= SD_FRAMING_ERROR;
  if (ls & UART_LineStatus_OE)
    sts |= SD_OVERRUN_ERROR;
  chnAddFlagsI(sdp, sts);
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  writePinHigh(B8);
#define UART_LS_STATUS (UART_LineStatus_PE | UART_LineStatus_FE | UART_LineStatus_OE)
  sn32_uart_t *u = sdp->uart;
  uint8_t ls = 0;
  uint32_t ii_buf= u->II;

    if ((ii_buf & UART_Interrupt_Status) == UART_Interrupt_Pending) writePinHigh(B0);
    // Get Interrupt ID
    uint8_t int_id = ((ii_buf >> 1) & UART_InterruptID_Status);
    // Debug
    uprintf("int_id %d \n",int_id);
    // Get Line Status
    ls = (uint8_t)u->LS;
    // Debug
    uprintf("ls %d \n",ls);

    while (int_id  == UART_InterruptID_RLS || int_id == UART_InterruptID_RDA) {
      //Debug
      if (int_id  == UART_InterruptID_RLS) writePinHigh(B10);
      if (int_id  == UART_InterruptID_RDA) writePinHigh(B9);

      /* Special case, LIN break detection.*/
      if (ls & UART_LineStatus_BI) {
        writePinHigh(B2);
        osalSysLockFromISR();
        chnAddFlagsI(sdp, SD_BREAK_DETECTED);
        ls = (uint8_t)u->LS;
        osalSysUnlockFromISR();
      }
      // Debug
      uprintf("ls1 %d \n",ls);

      /* Error condition detection.*/
      if (ls & UART_LS_STATUS) {
        writePinHigh(B4);
        osalSysLockFromISR();
        set_error(sdp, ls);
        ls = (uint8_t)u->LS;
        osalSysUnlockFromISR();
      }

      /* Data available.*/
      if (ls & UART_LineStatus_RDR) {
        uint8_t b;
        writePinHigh(B3);
        osalSysLockFromISR();
        b = (uint8_t)u->RB_b.RB & sdp->rxmask;
        sdIncomingDataI(sdp, b);
        osalSysUnlockFromISR();
      }
      ii_buf= u->II;
      int_id = ((ii_buf >> 1) & UART_InterruptID_Status);
      // Debug
      uprintf("ls3 %d \n",ls);
    }
    if ((int_id == UART_InterruptID_THRE) && (ls & UART_LineStatus_THRE)) {
      /* Transmission buffer empty.*/
      if (ls & UART_LineStatus_THRE) {
        // Debug
        uprintf("ls4 %d \n",ls);

        writePinHigh(B5);
        msg_t b;
        osalSysLockFromISR();
        b = oqGetI(&sdp->oqueue);
        if (b < MSG_OK) {
          chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
          u->IE &= ~(UART_TransmitterHoldingEmpty);
        }
        else
          u->TH_b.TH = b;
        osalSysUnlockFromISR();
      }
    }
    if ((int_id == UART_InterruptID_TEMT) && (ls & UART_LineStatus_TEMT)) {
      /* Physical transmission end.*/
      if (ls & UART_LineStatus_TEMT) {
        writePinHigh(B6);
        osalSysLockFromISR();
        if (oqIsEmptyI(&sdp->oqueue)) {
          chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
          u->IE &= ~(UART_TransmitterEmpty);
        }
        osalSysUnlockFromISR();
      }
    }
    // Update interrupt buffer
    ii_buf= u->II;

  writePinLow(B0);
  if (!(ls & UART_LineStatus_RDR)) writePinLow(B3);
  if (!(ls & UART_LineStatus_BI)) writePinLow(B2);
  if (!(ls & UART_LS_STATUS)) writePinLow(B4);
  if (!(ls & UART_LineStatus_OE)) writePinLow(B1);
  if(!(ls & UART_LineStatus_THRE)) writePinLow(B5);
  if(!(ls & UART_LineStatus_TEMT)) writePinLow(B6);
  if (((ii_buf >> 1) & UART_InterruptID_Status) != UART_InterruptID_RLS) writePinLow(B10);
  if (((ii_buf >> 1) & UART_InterruptID_Status) != UART_InterruptID_RDA) writePinLow(B9);
  writePinLow(B8);

}

#if SN32_SERIAL_USE_UART0 || defined(__DOXYGEN__)
static void notify0(io_queue_t *qp) {

  (void)qp;
  SN32_UART0->IE |= (UART_TransmitterHoldingEmpty | UART_TransmitterEmpty);
}
#endif

#if SN32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  SN32_UART1->IE |= (UART_TransmitterHoldingEmpty | UART_TransmitterEmpty);
}
#endif

#if SN32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  SN32_UART2->IE |= (UART_TransmitterHoldingEmpty | UART_TransmitterEmpty);
}
#endif

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

#if SN32_SERIAL_USE_UART0 || defined(__DOXYGEN__)
#if !defined(SN32_UART0_HANDLER)
#error "SN32_UART0_HANDLER not defined"
#endif
/**
 * @brief   UART0 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_UART0_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD0);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SN32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
#if !defined(SN32_UART1_HANDLER)
#error "SN32_UART1_HANDLER not defined"
#endif
/**
 * @brief   UART1 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_UART1_HANDLER) {

  OSAL_IRQ_PROLOGUE();

  serve_interrupt(&SD1);

  OSAL_IRQ_EPILOGUE();
}
#endif

#if SN32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
#if !defined(SN32_UART2_HANDLER)
#error "SN32_UART2_HANDLER not defined"
#endif
/**
 * @brief   UART2 interrupt handler.
 *
 * @isr
 */
OSAL_IRQ_HANDLER(SN32_UART2_HANDLER) {

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
  SD0.uart = SN32_UART0;
#endif

#if SN32_SERIAL_USE_UART1
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = SN32_UART1;
#endif

#if SN32_SERIAL_USE_UART2
  sdObjectInit(&SD2, NULL, notify2);
  SD2.uart = SN32_UART2;
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
  setPinOutput(B0);
  setPinOutput(B1);
  setPinOutput(B2);
  setPinOutput(B3);
  setPinOutput(B4);
  setPinOutput(B5);
  setPinOutput(B6);
  setPinOutput(B7);
  setPinOutput(B8);
  setPinOutput(B9);
  setPinOutput(B10);

  writePinLow(B0);
  writePinLow(B1);
  writePinLow(B2);
  writePinLow(B3);
  writePinLow(B4);
  writePinLow(B5);
  writePinLow(B6);
  writePinLow(B7);
  writePinLow(B8);
  writePinLow(B9);
  writePinLow(B10);


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
  //SN32_UART0->TH=UINT8_MAX;
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
    uart_deinit(sdp->uart);
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
