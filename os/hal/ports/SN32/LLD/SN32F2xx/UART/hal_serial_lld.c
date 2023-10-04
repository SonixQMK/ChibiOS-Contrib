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
  uint32_t apbclock;
  uint8_t dlm, dll, divaddval, mulval, oversampling;
  sn32_uart_t *u = sdp->uart;


  apbclock = SN32_HCLK;

#if defined(UART_OVER8)
  oversampling = 8;
#else
  oversampling = 16;
#endif

  // Calculate divider
  uint32_t uart_clock = config->speed * oversampling;

  // Check constraints based on oversampling value
  chDbgAssert(uart_clock <= apbclock / oversampling,
              "Invalid oversampling configuration for requested baud rate");

  uint32_t divider = (apbclock + uart_clock/2) / uart_clock;
  if ((divider < 1) || (divider > 0xFFFF)) { /*error*/ }
  dlm = (uint8_t)(divider >> 8);
  dll = (uint8_t)(divider & 0xFF);
  divaddval = 0; // disable fractional divider
  mulval = 0;

  // Update the registers
  u->LC = (config->UART_WordLength
          | config->UART_StopBits
          | config->UART_Parity
          | UART_Break_Control_Disable
          | UART_Divisor_Latch_Access_Enable);
  u->FD = (UART_FD_MULVAL(mulval) | UART_FD_DIVADDVAL(divaddval));
  u->FD_b.OVER8 = (oversampling == 8) ? 1 : 0;
  u->DLM = dlm;
  u->DLL = dll;

  u->LC &= ~(UART_Divisor_Latch_Access_Enable);

//  u->ABCTRL = config->UART_AutoBaudControl;

  // Set RX trigger level
  u->FIFOCTRL = (UART_FIFO_Enable
                | UART_RxFIFO_Reset
                | UART_TxFIFO_Reset
                | config->UART_FIFOControl);
  //u->ABCTRL |= (UART_AutoBaudControl_Timeout | UART_AutoBaudControl_End);

  // Reset FIFO and enable
  //u->FIFOCTRL |= (UART_FIFO_Enable);

  /* Note that some bits are enforced.*/
  u->IE = (UART_ReceiveDataAvailable | UART_ReceiveLine);

  // Enable UART
  //u->CTRL &= ~UART_TxEnable; //keep TX disabled
  //u->CTRL &= ~UART_RxEnable; //keep RX disabled
  u->CTRL = (UART_Enable| UART_RxEnable | UART_TxEnable);

  /* Deciding mask to be applied on the data register on receive,
     this is required in order to mask out the parity bit.*/
 // sdp->rxmask = 0x7F; // 8E2
}

/**
 * @brief   UART de-initialization.
 * @details This function must be invoked with interrupts disabled.
 *
 * @param[in] u         pointer to an UART I/O block
 */
static void uart_deinit(sn32_uart_t *u) {
  u->FIFOCTRL_b.FIFOEN =0;
  u->CTRL =0;
}
//static void debug_shit(uint8_t int_id, uint8_t ls);
/**
 * @brief   Error handling routine.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] sr        UART SR register value
*/
static void set_error(SerialDriver *sdp, uint8_t ls) {
  eventflags_t sts = 0;

 // debug_shit(0,ls);
  if(ls & UART_LineStatus_BI)
    sts |= SD_BREAK_DETECTED;
  if(ls & UART_LineStatus_OE)
    sts |= SD_OVERRUN_ERROR;
  if (ls & UART_LineStatus_PE)
    sts |= SD_PARITY_ERROR;
  if (ls & UART_LineStatus_FE)
    sts |= SD_FRAMING_ERROR;
  chnAddFlagsI(sdp, sts);
}

static void debug_ii(uint8_t int_id) {
//  uprintf("id_p %d \n",int_id);
  switch (int_id) {
  case UART_InterruptID_RDA:
      writePinHigh(B9);
      //uprintf("RDA \n");
      break;
  case UART_InterruptID_RLS:
      writePinHigh(B10);
      //uprintf("RLS \n");
      break;
  case UART_InterruptID_TEMT:
      writePinHigh(B11);
      //uprintf("TEMT \n");
      break;
  case UART_InterruptID_THRE:
      writePinHigh(B12);
      //uprintf("THRE \n");
      break;
  case UART_InterruptID_CTI:
      writePinHigh(B4);
      //uprintf("CTI \n");
      break;
  default:
     // writePinLow(B6);
     // writePinLow(B7);
      writePinLow(B4);
      writePinLow(B9);
      writePinLow(B10);
      writePinLow(B10);
      writePinLow(B11);
      writePinLow(B12);
      break;
  }
}

static void debug_shit(uint8_t int_id, uint8_t ls) {
 // uprintf("ls_p %d \n",ls);
  debug_ii(int_id);
  if (ls & UART_LineStatus_RDR) {
    writePinHigh(B2);
  } else {
    writePinLow(B2);
  }
  if (ls & UART_LineStatus_THRE) {
    writePinHigh(B3);
  } else {
    writePinLow(B3);
  }
  if (ls & UART_LineStatus_TEMT) {
    writePinHigh(B13);
  } else {
    writePinLow(B13);
  }
  if (ls & UART_LineStatus_RxError) {
      writePinHigh(B6);
    } else {
      writePinLow(B6);
  }
  if (ls & UART_LineStatus_TxError) {
    uprintf("TxError \n");
  }
  if (ls & UART_LineStatus_BI) {
    writePinHigh(B5);
  } else {
    writePinLow(B5);
  }
  if (ls & UART_LineStatus_PE) {
    writePinHigh(B1);
  } else {
    writePinLow(B1);
  }
  if (ls & UART_LineStatus_FE) {
    writePinHigh(B7);
  } else {
    writePinLow(B7);
  }
  if (ls & UART_LineStatus_OE) {
    writePinHigh(B8);
  } else {
    writePinLow(B8);
  }
}

/**
 * @brief   Common IRQ handler.
 *
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  writePinHigh(B0);
#define UART_LS_STATUS (UART_LineStatus_PE | UART_LineStatus_FE | UART_LineStatus_OE )//| UART_LineStatus_BI | UART_LineStatus_RxError)
  sn32_uart_t *u = sdp->uart;
  uint8_t ls = 0;
  uint32_t ii_buf= u->II;

  // Get Interrupt ID
  uint8_t int_id = ((ii_buf >> 1) & UART_InterruptID_Status);
  //while(int_id){
  ls = (uint8_t)u->LS;
  //debug
  debug_shit(int_id,ls);
  if((int_id == UART_InterruptID_RDA) || (int_id == UART_InterruptID_RLS)) {
  if (ls & UART_LineStatus_BI) {
    osalSysLockFromISR();
    chnAddFlagsI(sdp,SD_BREAK_DETECTED);
    uint8_t b = u->RB;
    (void)b;
    ii_buf= u->II;
    int_id = ((ii_buf >> 1) & UART_InterruptID_Status);
    debug_shit(int_id,ls);
    osalSysUnlockFromISR();
    writePinLow(B0);
    return;
  }

  //We have shit to receive
  if (ls & UART_LineStatus_RDR) {
    osalSysLockFromISR();
    //empty the FIFO ASAP
    uint8_t b = u->RB;
    //block any bad transfers
    if (ls & UART_LS_STATUS) {
      set_error(sdp, ls);

    } else {
      if(iqIsEmptyI(&sdp->iqueue)) chnAddFlagsI(sdp, CHN_INPUT_AVAILABLE);
      debug_shit(int_id,ls);
     // if(u->LS & UART_LineStatus_RDR) {
      if (iqPutI(&sdp->iqueue, b) < MSG_OK)
        chnAddFlagsI(sdp, SD_QUEUE_FULL_ERROR);
      //}
    }
    debug_shit(int_id,ls);
    osalSysUnlockFromISR();
    //ls = (uint8_t)u->LS;
  }
} else {
  //debug
  debug_shit(int_id,ls);

  /* Transmission buffer empty.*/
  if ((u->IE & UART_TransmitterHoldingEmpty) && (ls & UART_LineStatus_THRE)) {//&& !(ls & UART_LineStatus_BI)) {
      msg_t b;
      osalSysLockFromISR();
      debug_shit(int_id,ls);

      b = oqGetI(&sdp->oqueue);
      if (b < MSG_OK) {
        chnAddFlagsI(sdp, CHN_OUTPUT_EMPTY);
        u->IE &= ~(UART_TransmitterHoldingEmpty);
      }
      else {
        u->TH = b;
      debug_shit(int_id,ls); }
      debug_shit(int_id,ls);

      osalSysUnlockFromISR();
  }

  /* Physical transmission end.*/
  if ((u->IE & UART_TransmitterEmpty) && (ls & UART_LineStatus_TEMT)) {
    osalSysLockFromISR();
    debug_shit(int_id,ls);

    if (oqIsEmptyI(&sdp->oqueue)) {
      debug_shit(int_id,ls);

      chnAddFlagsI(sdp, CHN_TRANSMISSION_END);
      u->IE &= ~(UART_TransmitterEmpty);
    }
    osalSysUnlockFromISR();
  }
}

  ii_buf= u->II;
  int_id = ((ii_buf >> 1) & UART_InterruptID_Status);
  //debug
  debug_shit(int_id,ls);
  //}
  writePinLow(B0);
}

static void load(SerialDriver *sdp) {
  sn32_uart_t *u = sdp->uart;
  if (u->LS & UART_LineStatus_THRE) {
    msg_t b;
    osalSysLock();
    b = oqGetI(&sdp->oqueue);
    osalSysUnlock();
    if (b >= MSG_OK) {
      u->TH = b;
    }
  }
  u->IE |= (UART_TransmitterHoldingEmpty | UART_TransmitterEmpty);
}

#if SN32_SERIAL_USE_UART0 || defined(__DOXYGEN__)
static void notify0(io_queue_t *qp) {

  (void)qp;
  load(&SD0);
}
#endif

#if SN32_SERIAL_USE_UART1 || defined(__DOXYGEN__)
static void notify1(io_queue_t *qp) {

  (void)qp;
  load(&SD1);
}
#endif

#if SN32_SERIAL_USE_UART2 || defined(__DOXYGEN__)
static void notify2(io_queue_t *qp) {

  (void)qp;
  load(&SD2);
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
  setPinOutput(B11);
  setPinOutput(B12);
  setPinOutput(B13);

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
  writePinLow(B11);
  writePinLow(B12);
  writePinLow(B13);


  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
#if SN32_SERIAL_USE_UART0
    if (&SD0 == sdp) {
      /* UART0 clock enable.*/
      sys1EnableUART0();
      uart_init(sdp, config);
      //nvicClearPending(SN32_UART0_NUMBER);
      nvicEnableVector(SN32_UART0_NUMBER, SN32_SERIAL_UART0_PRIORITY);
    }
#endif
#if SN32_SERIAL_USE_UART1
    if (&SD1 == sdp) {
      /* UART1 clock enable.*/
      sys1EnableUART1();
      uart_init(sdp, config);
      nvicClearPending(SN32_UART1_NUMBER);
      nvicEnableVector(SN32_UART1_NUMBER, SN32_SERIAL_UART1_PRIORITY);
    }
#endif
#if SN32_SERIAL_USE_UART2
    if (&SD2 == sdp) {
      /* UART2 clock enable.*/
      sys1EnableUART2();
      uart_init(sdp, config);
      nvicClearPending(SN32_UART2_NUMBER);
      nvicEnableVector(SN32_UART2_NUMBER, SN32_SERIAL_UART2_PRIORITY);
    }
#endif
   // sdp->LC &= ~(UART_Break_Control_Enable);
  }
  //chThdSleepMicroseconds(50);
  //sdp->uart->CTRL |=(UART_TxEnable | UART_RxEnable);
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
