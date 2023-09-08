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

#ifndef SN32_UART_H
#define SN32_UART_H

typedef struct {
  union {
    union {
      uint32_t RB;
      struct {
        uint32_t RB       : 8;
        uint32_t          : 24;
      } RB_b;
    } ;

    union {
      uint32_t TH;      
      struct {
        uint32_t TH       : 8;
        uint32_t          : 24;
      } TH_b;
    } ;

    union {
      uint32_t DLL;
      struct {
        uint32_t DLL      : 8;
        uint32_t          : 24;
      } DLL_b;
    } ;
  };

  union {
    union {
      uint32_t DLM;
      struct {
        uint32_t DLM      : 8;
        uint32_t          : 24;
      } DLM_b;
    } ;

    union {
      uint32_t IE;
      struct {
        uint32_t RDAIE    : 1;
        uint32_t THREIE   : 1;
        uint32_t RLSIE    : 1;
        uint32_t          : 1;
        uint32_t TEMTIE   : 1;
        uint32_t          : 3;
        uint32_t ABEOIE   : 1;
        uint32_t ABTOIE   : 1;
        uint32_t          : 22;
      } IE_b;
    } ;
  };

  union {
    union {
      uint32_t II;
      struct {
        uint32_t INTSTATUS : 1;
        uint32_t INTID    : 3;
        uint32_t          : 2;
        uint32_t FIFOEN   : 2;
        uint32_t ABEOIF   : 1;
        uint32_t ABTOIF   : 1;
        uint32_t          : 22;
      } II_b;
    } ;
    
    union {
      uint32_t FIFOCTRL;
      struct {
        uint32_t FIFOEN   : 1;
        uint32_t          : 5;
        uint32_t RXTL     : 2;
        uint32_t          : 24;
      } FIFOCTRL_b;
    } ;
  };

  union {
    uint32_t LC;
    struct {
      uint32_t WLS        : 2;
      uint32_t SBS        : 1;
      uint32_t PE         : 1;
      uint32_t PS         : 2;
      uint32_t BC         : 1;
      uint32_t DLAB       : 1;
      uint32_t            : 24;
    } LC_b;
  } ;

  union {
    __IOM uint32_t MC;
    
    struct {
      uint32_t            : 1;
      uint32_t RTSCTRL    : 1;
      uint32_t            : 4;
      uint32_t RTSEN      : 1;
      uint32_t CTSEN      : 1;
      uint32_t            : 24;
    } MC_b;
  } ;

  union {
    uint32_t LS;
    struct {
      uint32_t RDR        : 1;
      uint32_t OE         : 1;
      uint32_t PE         : 1;
      uint32_t FE         : 1;
      uint32_t BI         : 1;
      uint32_t THRE       : 1;
      uint32_t TEMT       : 1;
      uint32_t RXFE       : 1;
      uint32_t            : 24;
    } LS_b;
  } ;

  union {
    __IM  uint32_t MS;
    struct {
      uint32_t DCTS       : 1;
      uint32_t            : 3;
      uint32_t CTS        : 1;
      uint32_t            : 27;
    } MS_b;
  } ;

  union {
    uint32_t SP;
    struct {
      uint32_t PAD        : 8;
      uint32_t            : 24;
    } SP_b;
  } ;

  union {
    uint32_t ABCTRL;
    struct {
      uint32_t START      : 1;
      uint32_t MODE       : 1;
      uint32_t AUTORESTART : 1;
      uint32_t            : 5;
      uint32_t ABEOIFC    : 1;
      uint32_t ABTOIFC    : 1;
      uint32_t            : 22;
    } ABCTRL_b;
  } ;
  uint32_t  RESERVED;

  union {
    uint32_t FD;
    struct {
      uint32_t DIVADDVAL  : 4;
      uint32_t MULVAL     : 4;
      uint32_t OVER8      : 1;
      uint32_t            : 23;
    } FD_b;
  } ;
  uint32_t  RESERVED1;

  union {
    uint32_t CTRL;
    struct {
      uint32_t UARTEN     : 1;
      uint32_t MODE       : 3;
      uint32_t            : 2;
      uint32_t RXEN       : 1;
      uint32_t TXEN       : 1;
      uint32_t            : 24;
    } CTRL_b;
  } ;

  union {
    uint32_t HDEN;
    struct {
      uint32_t HDEN       : 1;
      uint32_t            : 31;
    } HDEN_b;
  } ;
} sn32_uart_t;

#endif /* SN32_UART_H */

/** @} */
