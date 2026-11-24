#ifndef _Somfy_Rts_H
#define _Somfy_Rts_H

#include <stdint.h>
#include "board.h"

#define SOMFY_RTS_FRAME_SIZE 7

// Receiving Somfy RTS frames is enabled together with HAS_SOMFY_RTS (not in
// firmware built for 868MHz only). Define NO_SOMFY_RTS_RX in board.h to save
// program memory and RAM.
#if defined(HAS_SOMFY_RTS) && !defined(NO_SOMFY_RTS_RX) && !defined(_868MHZ)
#define HAS_SOMFY_RTS_RX
#endif

// Number of pulses kept for the Yp/Yh diagnostics (X25), RAM: 1 byte per
// pulse. A frame has up to 115 pulses. Off by default, for debugging define
// e.g. SOMFY_RTS_RX_CAPTURE 128 in board.h.
#ifndef SOMFY_RTS_RX_CAPTURE
#define SOMFY_RTS_RX_CAPTURE 0
#endif

/* public prototypes */
void somfy_rts_tunein(void);
void somfy_rts_func(char *in);

#ifdef HAS_SOMFY_RTS_RX
void    somfy_rts_rx_start(void);             // tune in and enable reception
void    somfy_rts_rx_off(void);               // clear the RX flag only
uint8_t somfy_rts_rx_active(void);            // used by the edge ISR
void    somfy_rts_edge(uint8_t pin, uint16_t duration_us);   // called from ISR
void    somfy_rts_task(void);                 // report received frames
uint16_t rf_receive_edge_elapsed_us(void);    // rf_receive.c: time since last edge
#endif

#endif
