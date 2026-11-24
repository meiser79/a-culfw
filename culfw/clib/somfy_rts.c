/*
 * Somfy RTS communication protocol
 *
 Copyright (c) 2014, Thomas Dankert <post@thomyd.de>
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <avr/interrupt.h>              // for cli, sei
#include <avr/pgmspace.h>               // for PROGMEM, __LPM
#include <avr/io.h>                     // for _BV
#include <stdint.h>                     // for int8_t
#include <stdlib.h>                     // for free, malloc
#include <string.h>                     // for memcpy

#include "board.h"                      // for HAS_ASKSIN, HAS_MORITZ, etc
#include "fband.h"                      // for checkFrequency
#include "stringfunc.h"                 // for fromhex, fromdec

#ifdef HAS_SOMFY_RTS

#include "cc1100.h"                     // for ccStrobe, cc1100_sendbyte, etc
#include "delay.h"                      // for my_delay_us, my_delay_ms
#include "display.h"                    // for DC, DNL, DU, display_hex2
#include "fncollection.h"               // for EE_CC1100_CFG_SIZE, erb, etc
#include "led.h"                        // for LED_OFF, LED_ON, SET_BIT
#include "rf_receive.h"                 // for set_txrestore, tx_report
#include "ttydata.h"                    // for TTY_Tx_Buffer
#include "somfy_rts.h"
#include "rf_mode.h"
#include "multi_CC.h"
#include "clock.h"                      // for ticks

#ifdef USE_HAL
#include "hal.h"
#endif

#ifndef USE_RF_MODE

#ifdef HAS_ASKSIN
#include "rf_asksin.h"                  // for asksin_on, rf_asksin_init
#endif

#ifdef HAS_MORITZ
#include "rf_moritz.h"                  // for rf_moritz_init, moritz_on
#endif

#endif
typedef uint8_t somfy_rts_frame_t;

const PROGMEM const uint8_t CC1100_SOMFY_RTS_CFG[EE_CC1100_CFG_SIZE] = {
// CULFW   IDX NAME     RESET STUDIO COMMENT
	0x0D,// 00 IOCFG2   *29   *0B    GDO2 as serial output
	0x2E,// 01 IOCFG1    2E    2E    Tri-State
	0x2D,// 02 IOCFG0   *3F   *0C    GDO0 for input
	0x07,// 03 FIFOTHR   07   *47
	0xD3,// 04 SYNC1     D3    D3
	0x91,// 05 SYNC0     91    91
	0x3D,// 06 PKTLEN   *FF    3D
	0x04,// 07 PKTCTRL1  04    04
	0x32,// 08 PKTCTRL0 *45    32
	0x00,// 09 ADDR      00    00
	0x00,// 0A CHANNR    00    00
	0x06,// 0B FSCTRL1  *0F    06    152kHz IF Frquency
	0x00,// 0C FSCTRL0   00    00
	0x10,// 0D FREQ2    *1E    21    433.42 (10,AB,85: Somfy RTS Frequency)
	0xAB,// 0E FREQ1    *C4    65
	0xA5,// 0F FREQ0    *EC    e8
	0xC5,// 10 MDMCFG4  *8C    55    bWidth 102kHz (RX, was 0x55: 325kHz)
	0x0A,// 11 MDMCFG3  *22   *43    Drate: 828 ((256+11)*2^5)*26000000/2^28
	0x30,// 12 MDMCFG2  *02   *B0    Modulation: ASK
	0x23,// 13 MDMCFG1  *22    23
	0xb9,// 14 MDMCFG0  *F8    b9    ChannelSpace: 350kHz
	0x00,// 15 DEVIATN  *47    00
	0x07,// 16 MCSM2     07    07
	0x00,// 17 MCSM1     30    30
	0x18,// 18 MCSM0    *04    18    Calibration: RX/TX->IDLE
	0x14,// 19 FOCCFG   *36    14
	0x6C,// 1A BSCFG     6C    6C
	0x07,// 1B AGCCTRL2 *03   *03    42 dB (0xC7, reduced gain: no reception
	     //                          below ca. -80dBm)
	0x00,// 1C AGCCTRL1 *40   *40
	0xB2,// 1D AGCCTRL0 *91   *92    12dB OOK decision boundary, AGC wait 32
	     //                          samples (RX; 0x90: 4dB lost hw sync pulses)
	0x87,// 1E WOREVT1   87    87
	0x6B,// 1F WOREVT0   6B    6B
	0xF8,// 20 WORCTRL   F8    F8
	0x56,// 21 FREND1    56    56
	0x11,// 22 FREND0   *16    17    0x11 for no PA ramping
	0xE9,// 23 FSCAL3   *A9    E9
	0x2A,// 24 FSCAL2   *0A    2A
	0x00,// 25 FSCAL1    20    00
	0x1F,// 26 FSCAL0    0D    1F
	0x41,// 27 RCCTRL1   41    41
	0x00,// 28 RCCTRL0   00    00
};

#ifndef USE_RF_MODE
uint8_t somfy_restore_asksin = 0;
uint8_t somfy_restore_moritz = 0;
#endif

uint8_t somfy_rts_repetition = 6;
uint16_t somfy_rts_interval = 1240; // symbol width in us -> ca. 828 Hz data rate
uint16_t somfy_rts_interval_half = 620;

#ifdef HAS_SOMFY_RTS_RX
/*
 * Somfy RTS receiver
 *
 * The CC1101 is tuned to 433.42 MHz (ASK, asynchronous serial mode) and every
 * edge on GDO2 is fed by the edge ISR (rf_receive.c) into somfy_rts_edge()
 * together with the duration of the level that just ended.
 *
 * Frame on air (symbol width ca. 1280us):
 *   wakeup (only first frame) | n x hw sync 2560us high/low |
 *   sw sync 4850us high + 640us low | 56 bit manchester (0 = high->low,
 *   1 = low->high), MSB first | inter-frame gap ca. 30ms low
 *
 * The decoded frame is "decrypted", the checksum is verified and the frame is
 * reported in the same format as the echo of the Ys command:
 *   Ys<key><ctrl+cks><rolling code><address a2 a1 a0>[<rssi>]
 */

#define SOMFY_RX_SYNC_MIN   3400        // us, software sync (nominal 4550-4850us,
                                        // received: 4326-4387us)
#define SOMFY_RX_SYNC_MAX   5900
#define SOMFY_RX_FLUSH_US   3000        // no edge for this long: frame ended
#define SOMFY_RX_YP_LINE    32          // pulses per Yp line
#define SOMFY_RX_HIST       8           // pulses before a rejected sw sync (Yh)
#define SOMFY_RX_HWSYNC_MIN 900         // hardware sync pulse (nominal 2416-2560us,
#define SOMFY_RX_HWSYNC_MAX 4300        // received: high 1328-1520, low 3072-3600us)
#define SOMFY_RX_HWSYNC_GAP 10000       // longer pulses end the hw sync
#define SOMFY_RX_HWSYNC_CNT 2           // min. hw sync pulses before the sw sync
                                        // (first frame: 2 x high/low = 4, but
                                        // dropouts were seen there, too)

#define SOMFY_RX_IDLE       0
#define SOMFY_RX_SYNC       1
#define SOMFY_RX_DATA       2

#define SOMFY_RX_HALF_NONE  0
#define SOMFY_RX_HALF_LOW   1
#define SOMFY_RX_HALF_HIGH  2

#define SOMFY_RX_BITS       (SOMFY_RTS_FRAME_SIZE*8)

typedef struct {
  uint8_t state;
  uint8_t bits;                         // number of decoded bits
  uint8_t half;                         // pending first half of a bit
  uint8_t data[SOMFY_RTS_FRAME_SIZE];   // raw ("encrypted") frame
  // mean length of a half symbol, tracked during the frame
  uint16_t hlen;
  uint8_t plevel;                       // level of the next data pulse
  uint16_t dprev;                       // length of the previous data pulse
  uint8_t nprev;                        // its number of half symbols
  uint8_t hwsync;                       // consecutive hardware sync pulses
  // glitch filter: the last pulse is held back until the next one has ended
  uint8_t hold_valid, hold_level, glitch;
  uint16_t hold_dur, glitch_dur;
  uint8_t last_pin;                     // pin level after the previous edge
#if SOMFY_RTS_RX_CAPTURE > 0
  uint8_t hist[SOMFY_RX_HIST];          // last pulses outside of a frame
  uint8_t hist_pos;
#endif
} somfy_rx_t;

// diagnostics (X command bit 2, e.g. X25): why a frame was aborted
typedef struct {
  volatile uint8_t ready;
  uint8_t kind;                         // 'd': aborted, 'h': sw sync rejected
  uint8_t bits, level;
  uint16_t raw;                         // pulse length as measured
  uint16_t hlen;                        // mean half symbol length
#if SOMFY_RTS_RX_CAPTURE > 0
  uint8_t n;                            // number of captured pulses
  uint8_t pos;                          // next pulse to report
  uint8_t pulse[SOMFY_RTS_RX_CAPTURE];  // pulses since the sw sync, 16us units
#endif
} somfy_rx_dbg_t;

extern void reset_input(void);          // rf_receive.c

static volatile uint8_t somfy_rx_on[NUM_SLOWRF];
static somfy_rx_t somfy_rx[NUM_SLOWRF];
static somfy_rx_dbg_t somfy_rx_dbg[NUM_SLOWRF];
static volatile uint8_t somfy_rx_ready[NUM_SLOWRF];
static uint8_t somfy_rx_frame[NUM_SLOWRF][SOMFY_RTS_FRAME_SIZE];
static uint8_t somfy_rx_last[NUM_SLOWRF][SOMFY_RTS_FRAME_SIZE];
static uint32_t somfy_rx_time[NUM_SLOWRF];

#define SOMFY_RX_ON (CC_INSTANCE < NUM_SLOWRF && somfy_rx_on[CC_INSTANCE])
#else
#define SOMFY_RX_ON 0
#endif

void somfy_rts_tunein(void) {
#ifdef USE_HAL
  hal_CC_GDO_init(CC_INSTANCE,INIT_MODE_OUT_CS_IN);
  hal_enable_CC_GDOin_int(CC_INSTANCE,FALSE); // disable INT - we'll poll...
#else
	EIMSK &= ~_BV(CC1100_INT);
	SET_BIT(CC1100_CS_DDR, CC1100_CS_PIN); // CS as output
#endif

	CC1100_DEASSERT;// Toggle chip select signal
	my_delay_us(30);
	CC1100_ASSERT;
	my_delay_us(30);
	CC1100_DEASSERT;
	my_delay_us(45);

	ccStrobe( CC1100_SRES);// Send SRES command
	my_delay_us(100);

	// load configuration
	CC1100_ASSERT;

	cc1100_sendbyte( 0 | CC1100_WRITE_BURST );
	for(uint8_t i = 0; i < EE_CC1100_CFG_SIZE; i++) {
		cc1100_sendbyte(__LPM(CC1100_SOMFY_RTS_CFG+i));
	}

	CC1100_DEASSERT;

	// setup PA table
	uint8_t *pa = EE_CC1100_PA;
	CC1100_ASSERT;
	cc1100_sendbyte( CC1100_PATABLE | CC1100_WRITE_BURST);
	for (uint8_t i = 0; i < 8; i++) {
		cc1100_sendbyte(erb(pa++));
	}
	CC1100_DEASSERT;

	// Set CC_ON
	ccStrobe( CC1100_SCAL);
	my_delay_ms(1);
#ifndef USE_RF_MODE
	cc_on = 1;
#endif
    checkFrequency(); 
}

#ifdef HAS_SOMFY_RTS_RX
uint8_t somfy_rts_rx_active(void) {
  return SOMFY_RX_ON;
}

void somfy_rts_rx_off(void) {
  if (SOMFY_RX_ON) {
    somfy_rx_on[CC_INSTANCE] = 0;
    reset_input();                      // hand the timer back to SlowRF
  }
}

// Stop decoding without touching the timer (used while sending)
static void somfy_rts_rx_pause(void) {
  if (CC_INSTANCE < NUM_SLOWRF)
    somfy_rx_on[CC_INSTANCE] = 0;
}

void somfy_rts_rx_start(void) {
  somfy_rts_tunein();                   // disables the edge interrupt

  if (!(CC_INSTANCE < NUM_SLOWRF))      // no edge interrupt / timer available
    return;

  reset_input();                        // abort a pending SlowRF reception
  memset(&somfy_rx[CC_INSTANCE], 0, sizeof(somfy_rx_t));
  somfy_rx_dbg[CC_INSTANCE].ready = 0;
  somfy_rx_ready[CC_INSTANCE] = 0;

  // let the timer run freely, the edge ISR measures the pulse widths
#ifdef USE_HAL
  hal_enable_CC_timer_int(CC_INSTANCE, FALSE);
  HAL_timer_set_reload_register(CC_INSTANCE, 0xFFFF);
  HAL_timer_reset_counter_value(CC_INSTANCE);
#else
  TIMSK1 = 0;
  OCR1A = 0xFFFF;
  TCNT1 = 0;
#endif

  somfy_rx_on[CC_INSTANCE] = 1;
  ccRX();                               // enables the edge interrupt
}

static void somfy_rx_complete(somfy_rx_t *r) {
  r->state = SOMFY_RX_IDLE;
  if (somfy_rx_ready[CC_INSTANCE])      // previous frame not yet reported
    return;
  memcpy(somfy_rx_frame[CC_INSTANCE], r->data, SOMFY_RTS_FRAME_SIZE);
  somfy_rx_ready[CC_INSTANCE] = 1;
}

static void somfy_rx_abort(somfy_rx_t *r, uint8_t level, uint16_t raw) {
  somfy_rx_dbg_t *g = &somfy_rx_dbg[CC_INSTANCE];
  if (r->state != SOMFY_RX_IDLE && (TX_REPORT & REP_BITS) && !g->ready) {
    g->kind = 'd';
    g->bits = r->bits;
    g->level = level;
    g->raw = raw;
    g->hlen = r->hlen;
    g->ready = 1;                       // freezes the captured pulses
  }
  r->state = SOMFY_RX_IDLE;
}

/*
 * Decode one (glitch filtered) pulse.
 * level: 1 = carrier on (as read from the pin), d: length in us
 *
 * Within a frame the level is not taken from the pin but from the position:
 * after the software sync (high) low and high pulses alternate.
 */
static void somfy_rx_pulse(somfy_rx_t *r, uint8_t level, uint16_t d) {
#if SOMFY_RTS_RX_CAPTURE > 0
  somfy_rx_dbg_t *g = &somfy_rx_dbg[CC_INSTANCE];
#endif
  uint8_t n;
  uint8_t swsync = level && d >= SOMFY_RX_SYNC_MIN && d <= SOMFY_RX_SYNC_MAX;

  // software sync starts (or restarts) a frame, if it follows the
  // hardware sync. Otherwise noise could start a frame.
  if (swsync && r->hwsync >= SOMFY_RX_HWSYNC_CNT) {
    r->state = SOMFY_RX_SYNC;
    r->hlen = somfy_rts_interval_half;
    r->plevel = 0;
#if SOMFY_RTS_RX_CAPTURE > 0
    if (!g->ready)
      g->n = 0;
#endif
  } else {
#if SOMFY_RTS_RX_CAPTURE > 0
    // diagnostics: a sw sync without enough hw sync pulses before it is
    // reported with the pulses before it (Yh)
    if (swsync && r->state == SOMFY_RX_IDLE && (TX_REPORT & REP_BITS) &&
        !g->ready) {
      uint8_t i;
      g->kind = 'h';
      g->bits = r->hwsync;
      g->level = level;
      g->raw = d;
      g->hlen = 0;
      for (i = 0; i < SOMFY_RX_HIST; i++)
        g->pulse[i] = r->hist[(r->hist_pos + i) % SOMFY_RX_HIST];
      g->n = SOMFY_RX_HIST;
      g->ready = 1;
    }
#endif
    swsync = 0;
  }

  if (d >= SOMFY_RX_HWSYNC_MIN && d <= SOMFY_RX_HWSYNC_MAX) {
    if (r->hwsync < 0xFF)
      r->hwsync++;
  } else if (d < SOMFY_RX_HWSYNC_MIN || d > SOMFY_RX_HWSYNC_GAP) {
    r->hwsync = 0;
  }                                     // else: hw sync pulses merged by a
                                        // dropout, or the sw sync itself

  if (r->state == SOMFY_RX_IDLE) {
#if SOMFY_RTS_RX_CAPTURE > 0
    r->hist[r->hist_pos] = (d >> 4) > 0xFF ? 0xFF : (d >> 4);
    r->hist_pos = (r->hist_pos + 1) % SOMFY_RX_HIST;
#endif
    return;
  }

#if SOMFY_RTS_RX_CAPTURE > 0
  if ((TX_REPORT & REP_BITS) && !g->ready && g->n < SOMFY_RTS_RX_CAPTURE)
    g->pulse[g->n++] = (d >> 4) > 0xFF ? 0xFF : (d >> 4);
#endif

  if (swsync)                           // the sw sync itself
    return;

  level = r->plevel;
  r->plevel ^= 1;

  // Number of half symbols. The ASK demodulator delays the rising or the
  // falling edge, so one level gets longer and the other shorter (received:
  // up to +-1000us in the hw sync, half symbols 128-816us), varying during
  // the frame. This cancels out in the sum of two adjacent pulses (a high
  // and a low): it is 2, 3 or 4 half symbols, minus those of the previous
  // pulse gives the half symbols of this pulse.
  uint16_t h = r->hlen;
  if (r->state == SOMFY_RX_SYNC) {
    // the first low: the last half symbol of the sync and the first half of
    // the first bit (always 1, key 0x8?-0xB?)
    if (d < h + (h >> 2) || d > 3 * h + (h >> 2)) {
      somfy_rx_abort(r, level, d);
      return;
    }
    n = 2;
  } else if (d < (h >> 1)) {
    // much too short (carrier lost or found too early): one damaged half
    // symbol, the missing time is in the neighbour pulse and thus still in
    // the sum with the next pulse
    n = 1;
  } else {
    uint16_t sum = r->dprev + (d > 0x7FFF ? 0x7FFF : d);
    uint8_t ns;
    if (sum < h + (h >> 1))
      ns = 0;                           // too short
    else if (sum < 2 * h + (h >> 1))
      ns = 2;
    else if (sum < 3 * h + (h >> 1))
      ns = 3;
    else if (sum < 4 * h + (h >> 1) + (h >> 2))
      ns = 4;
    else
      ns = 0;                           // too long
    if (ns <= r->nprev || ns - r->nprev > 2) {
      somfy_rx_abort(r, level, d);
      return;
    }
    n = ns - r->nprev;
    if (r->dprev >= (h >> 1))
      r->hlen = (7 * h + (ns == 3 ? sum / 3 : sum >> (ns >> 1))) >> 3;
  }
  r->dprev = d;
  r->nprev = n;

  if (r->state == SOMFY_RX_SYNC) {
    // the first half symbol of low belongs to the sync
    r->state = SOMFY_RX_DATA;
    r->bits = 0;
    r->half = SOMFY_RX_HALF_NONE;
    n--;
  }

  while (n--) {
    if (r->half == SOMFY_RX_HALF_NONE) {
      r->half = level ? SOMFY_RX_HALF_HIGH : SOMFY_RX_HALF_LOW;
      if (r->bits == SOMFY_RX_BITS-1) {
        // last bit: its value is known from the first half, the second
        // half may merge with the inter-frame gap
        uint8_t *p = &r->data[SOMFY_RTS_FRAME_SIZE-1];
        *p = (*p << 1) | (level ? 0 : 1);
        somfy_rx_complete(r);
        return;
      }
      continue;
    }

    // second half: manchester needs a transition in the middle of the bit
    if ((r->half == SOMFY_RX_HALF_HIGH) == (level != 0)) {
      somfy_rx_abort(r, level, d);
      return;
    }

    uint8_t *p = &r->data[r->bits >> 3];
    *p = (*p << 1) | (r->half == SOMFY_RX_HALF_LOW ? 1 : 0);
    r->half = SOMFY_RX_HALF_NONE;
    r->bits++;
  }
}

/*
 * Called from the edge ISR.
 * pin:         input level read in the ISR (1 = carrier on)
 * duration_us: time since the previous edge
 *
 * The level of the pulse which just ended is taken from the pin level read
 * at the previous edge: if a spike is shorter than the interrupt latency,
 * the pin may already be back when it is read, and the long pulse before
 * would get the wrong level.
 *
 * Short spikes (< ca. 1/4 half symbol) are merged into the surrounding pulse.
 * Therefore each pulse is held back until the next pulse has ended.
 */
static uint16_t somfy_rx_add(uint16_t a, uint16_t b) {
  return (0xFFFF - a < b) ? 0xFFFF : a + b;
}

void somfy_rts_edge(uint8_t pin, uint16_t duration_us) {
  somfy_rx_t *r = &somfy_rx[CC_INSTANCE];
  uint8_t level = r->last_pin;

  r->last_pin = pin;

  // spike(s): < ca. 1/4 half symbol (dropouts of 128-165us were seen)
  if (duration_us < (somfy_rts_interval_half >> 2) + (somfy_rts_interval_half >> 5)) {
    if (r->hold_valid) {
      r->glitch_dur = r->glitch ? somfy_rx_add(r->glitch_dur, duration_us)
                                : duration_us;
      r->glitch = 1;
    }
    return;
  }

  if (r->glitch) {
    r->glitch = 0;
    if (level == r->hold_level) {       // spike inside the held pulse
      r->hold_dur = somfy_rx_add(r->hold_dur,
                                 somfy_rx_add(r->glitch_dur, duration_us));
      return;
    }
    // spikes at the beginning of this pulse
    duration_us = somfy_rx_add(duration_us, r->glitch_dur);
  }

  if (r->hold_valid)
    somfy_rx_pulse(r, r->hold_level, r->hold_dur);

  r->hold_valid = 1;
  r->hold_level = level;
  r->hold_dur = duration_us;
}

void somfy_rts_task(void) {
  uint8_t inst = CC_INSTANCE;
  uint8_t f[SOMFY_RTS_FRAME_SIZE];
  uint8_t i, cks = 0;

  if (!(inst < NUM_SLOWRF))
    return;

  // no further edge (e.g. after the last frame): decode the held pulse
  if (somfy_rx[inst].hold_valid) {
    cli();
    if (somfy_rx[inst].hold_valid && !somfy_rx[inst].glitch &&
        rf_receive_edge_elapsed_us() > SOMFY_RX_FLUSH_US) {
      somfy_rx[inst].hold_valid = 0;
      somfy_rx_pulse(&somfy_rx[inst], somfy_rx[inst].hold_level,
                     somfy_rx[inst].hold_dur);
    }
    sei();
  }

  if (somfy_rx_dbg[inst].ready == 1) {
    somfy_rx_dbg_t *g = &somfy_rx_dbg[inst];
    // Yd<bits> <H|L> <us> <mean half symbol>: aborted frame
    // Yh<hw sync pulses> <H|L> <us> 0: sw sync without hw sync, the Yp line
    // has the pulses before it (oldest first, alternating levels)
    MULTICC_PREFIX();
    DC('Y'); DC(g->kind);
    DU(g->bits, 2);
    DC(' ');
    DC(g->level ? 'H' : 'L');
    DU(g->raw, 6);
    DU(g->hlen, 5);
    DNL();
#if SOMFY_RTS_RX_CAPTURE > 0
    g->pos = 0;
    g->ready = 2;
#else
    g->ready = 0;
#endif
  }

#if SOMFY_RTS_RX_CAPTURE > 0
  // Yp<pulses>: pulses since the sw sync (high first), 16us units, in
  // lines of up to SOMFY_RX_YP_LINE pulses, each one written only when it
  // fits into the TX buffer (the buffer drops what does not fit)
  if (somfy_rx_dbg[inst].ready == 2 &&
      TTY_Tx_Buffer.nbytes <= TTY_BUFSIZE - (2 * SOMFY_RX_YP_LINE + 8)) {
    somfy_rx_dbg_t *g = &somfy_rx_dbg[inst];
    MULTICC_PREFIX();
    DC('Y'); DC('p');
    for (i = 0; i < SOMFY_RX_YP_LINE && g->pos < g->n; i++)
      DH2(g->pulse[g->pos++]);
    DNL();
    if (g->pos >= g->n)
      g->ready = 0;
  }
#endif

  if (!somfy_rx_ready[inst])
    return;

  memcpy(f, somfy_rx_frame[inst], SOMFY_RTS_FRAME_SIZE);
  somfy_rx_ready[inst] = 0;

  // "decrypt"
  for (i = SOMFY_RTS_FRAME_SIZE-1; i > 0; i--)
    f[i] ^= f[i-1];

  // checksum: xor of all nibbles must be 0
  for (i = 0; i < SOMFY_RTS_FRAME_SIZE; i++)
    cks ^= f[i] ^ (f[i] >> 4);
  // checksum ok, address not 000000/FFFFFF, rolling code not 0 (except
  // for sensor frames, command E)
  if ((cks & 0x0F) ||
      (f[4] == 0x00 && f[5] == 0x00 && f[6] == 0x00) ||
      (f[4] == 0xFF && f[5] == 0xFF && f[6] == 0xFF) ||
      (f[2] == 0x00 && f[3] == 0x00 && (f[1] >> 4) != 0x0E)) {
    if (TX_REPORT & REP_BITS) {         // Yc<frame>: invalid frame
      MULTICC_PREFIX();
      DC('Y'); DC('c');
      for (i = 0; i < SOMFY_RTS_FRAME_SIZE; i++)
        DH2(f[i]);
      DNL();
    }
    return;
  }

  // a remote repeats the frame as long as the button is pressed
  if (!(TX_REPORT & REP_REPEATED) &&
      !memcmp(f, somfy_rx_last[inst], SOMFY_RTS_FRAME_SIZE) &&
      (ticks - somfy_rx_time[inst]) < REPTIME) {
    somfy_rx_time[inst] = ticks;
    return;
  }
  memcpy(somfy_rx_last[inst], f, SOMFY_RTS_FRAME_SIZE);
  somfy_rx_time[inst] = ticks;

  LED_ON();
  MULTICC_PREFIX();
  DC('Y');
  DC('s');
  for (i = 0; i < SOMFY_RTS_FRAME_SIZE; i++)
    DH2(f[i]);
  if (TX_REPORT & REP_RSSI)
    DH2(cc1100_readReg(CC1100_RSSI));
  DNL();
  LED_OFF();
}
#endif // HAS_SOMFY_RTS_RX

static void send_somfy_rts_bitZero(void) {
	// Somfy RTS bits are manchester encoded: 0 = high->low
	CC1100_SET_OUT;// High
	my_delay_us(somfy_rts_interval_half);
	CC1100_CLEAR_OUT;// Low
	my_delay_us(somfy_rts_interval_half);
}

static void send_somfy_rts_bitOne(void) {
	// Somfy RTS bits are manchester encoded: 1 = low->high
	CC1100_CLEAR_OUT;// Low
	my_delay_us(somfy_rts_interval_half);
	CC1100_SET_OUT;// High
	my_delay_us(somfy_rts_interval_half);
}

static void send_somfy_rts_frame(somfy_rts_frame_t *frame, int8_t hwPulses) {
	// send hardware sync (pulses of ca. double length)
	for (int8_t i = 0; i < hwPulses; i++) {
		CC1100_SET_OUT;		// High
		my_delay_us(2550);
		CC1100_CLEAR_OUT;// Low
		my_delay_us(2550);
	}

	// send software sync (4 x symbol width high, half symbol width low)
	CC1100_SET_OUT;// High
	my_delay_us(4860);
	CC1100_CLEAR_OUT;// Low
	my_delay_us(somfy_rts_interval_half);

	// Send the user data
	for (int8_t i = 0; i < SOMFY_RTS_FRAME_SIZE; i++) {
		uint16_t mask = 0x80; // mask to send bits (MSB first)
		uint8_t d = frame[i];
		for (int8_t j = 0; j < 8; j++) {
			if ((d & mask) == 0) {
				send_somfy_rts_bitZero();
			} else {
				send_somfy_rts_bitOne();
			}
			mask >>= 1; //get next bit
		} //end of byte
	} //end of data

	// send inter-frame gap
	// if last bit = 0, silence is 1/2 symbol longer
	CC1100_CLEAR_OUT;// Low
	my_delay_us(30415 + ((frame[SOMFY_RTS_FRAME_SIZE-1] & 1) ? 0 : somfy_rts_interval_half));
}

static uint8_t somfy_rts_calc_checksum(somfy_rts_frame_t *frame) {
	uint8_t checksum = 0;
	for (int8_t i = 0; i < 7; i++) {
		checksum = checksum ^ frame[i] ^ (frame[i] >> 4);
	}

	return checksum;
}

static void somfy_rts_send(char *in) {
	int8_t i, j;

	// input is in format: Ys_key_ctrl_cks_rollcode_a0_a1_a2
	// Ys ad 20 0ae3 a2 98 42

	// create somfy frame from the given input
	// 0   |    1     |   2     3    | 4   5   6
	// key | ctrl+cks | rolling_code | address

	uint8_t buf = 0;
#ifdef ARM
	somfy_rts_frame_t airdata[SOMFY_RTS_FRAME_SIZE];
#else
	somfy_rts_frame_t *airdata = malloc(SOMFY_RTS_FRAME_SIZE);
#endif

	fromhex(in+2, &buf, 1);// key
	airdata[0] = buf;

	fromhex(in+4, &buf, 1);// ctrl+cks (cks set to 0)
	airdata[1] = buf & 0xF0;

	fromhex(in+6, &buf, 1);// rolling code 1/2
	airdata[2] = buf;

	fromhex(in+8, &buf, 1);// rolling code 2/2
	airdata[3] = buf;

	fromhex(in+10, &buf, 1);// address a0
	airdata[6] = buf;

	fromhex(in+12, &buf, 1);// address a1
	airdata[5] = buf;

	fromhex(in+14, &buf, 1);// address a2
	airdata[4] = buf;

	// calculate checksum
	airdata[1] |= (somfy_rts_calc_checksum(airdata) & 0x0F);
	
	// save unencrypted data to return later
#ifdef ARM
	somfy_rts_frame_t unencrypted[SOMFY_RTS_FRAME_SIZE];
#else
	somfy_rts_frame_t *unencrypted = malloc(SOMFY_RTS_FRAME_SIZE);
#endif
	memcpy(unencrypted, airdata, SOMFY_RTS_FRAME_SIZE);

	// "encrypt"
	for (i = 1; i < SOMFY_RTS_FRAME_SIZE; i++) {
		airdata[i] = airdata[i] ^ airdata[i-1];
	}

	LED_ON();

#if defined (HAS_IRRX) || defined (HAS_IRTX) // Blockout IR_Reception for the moment
	cli();
#endif

#ifdef USE_RF_MODE
	change_RF_mode(RF_mode_somfy);
#ifdef HAS_SOMFY_RTS_RX
	somfy_rts_rx_pause();
#endif
#else
	uint8_t rx_was_on = SOMFY_RX_ON;
	// If NOT in Somfy receive mode
	if (!rx_was_on) {
	#ifdef HAS_ASKSIN
		if (asksin_on) {
			somfy_restore_asksin = 1;
			asksin_on = 0;
		}
	#endif
	#ifdef HAS_MORITZ
		if(moritz_on) {
			somfy_restore_moritz = 1;
			moritz_on = 0;
		}
	#endif
		somfy_rts_tunein();
		my_delay_ms(3);             // 3ms: Found by trial and error
	}
#ifdef HAS_SOMFY_RTS_RX
	else {
		somfy_rts_rx_pause();
	}
#endif
#endif
	ccStrobe(CC1100_SIDLE);
	ccStrobe(CC1100_SFRX);
	ccStrobe(CC1100_SFTX);

	// enable TX
	ccTX();

	// send wakeup pulse
	CC1100_SET_OUT;// High
	my_delay_us(9415);
	CC1100_CLEAR_OUT;// Low
	my_delay_ms(89);// should be 89565 us
	my_delay_us(565);

	// send the whole frame several times
	for(i = 0; i < somfy_rts_repetition; i++) {
		send_somfy_rts_frame(airdata, (i == 0) ? 2 : 7); // send 2 hw Sync pulses at first, and 7 for repeated frames
	}

#ifdef USE_RF_MODE
	if(!restore_RF_mode()) {
	  // we were already in Somfy mode: enable RX again
#ifdef HAS_SOMFY_RTS_RX
    somfy_rts_rx_start();
#else
    if (TX_REPORT) {
      ccRX();
    } else {
      ccStrobe(CC1100_SIDLE);
    }
#endif
	}
#else
	if (rx_was_on) {
		// enable Somfy RX again
#ifdef HAS_SOMFY_RTS_RX
		somfy_rts_rx_start();
#endif
	}
 	#ifdef HAS_ASKSIN
   	else if (somfy_restore_asksin) {
		somfy_restore_asksin = 0;
   		rf_asksin_init();
		asksin_on = 1;
   	 	ccRX();
  	}
  	#endif
	#ifdef HAS_MORITZ
	else if (somfy_restore_moritz) {
		somfy_restore_moritz = 0;
		rf_moritz_init();
	}
	#endif
  	else {
    	set_txrestore();
  	}
#endif

#if defined (HAS_IRRX) || defined (HAS_IRTX) //Activate IR_Reception again
	sei();
#endif

	LED_OFF();
	MULTICC_PREFIX();
	DC('Y');
	DC('s');
	for(j = 0; j < SOMFY_RTS_FRAME_SIZE; j++) {
		display_hex2(unencrypted[j]);
	}
	DNL();

#ifndef ARM
	free(unencrypted);
	free(airdata);
#endif
}

void somfy_rts_func(char *in) {
	if (in[1] == 's') {
		somfy_rts_send(in); // Send real data

	} else if (in[1] == 'r') { // Set repetition
		fromdec (in+2, (uint8_t *)&somfy_rts_repetition);
		MULTICC_PREFIX();
		DC('Y');DC('r');DC(':');
		DU(somfy_rts_repetition, 0);
		DNL();

	} else if (in[1] == 't') { // Set symbol width
		if (in[2] == '0' ) {
			somfy_rts_interval = 1240;
		} else {
			fromdec (in+2, (uint8_t *)&somfy_rts_interval);
		}
		somfy_rts_interval_half = somfy_rts_interval / 2;

		MULTICC_PREFIX();
		DC('Y');DC('t');DC(':');
		DU(somfy_rts_interval, 6);
		DNL();
#ifdef HAS_SOMFY_RTS_RX
	} else if (in[1] == 'e') {	// Enable/disable reception
		if (in[2] == '1') {
#ifdef USE_RF_MODE
			if (CC_INSTANCE < NUM_SLOWRF)
				set_RF_mode(RF_mode_somfy);
#else
			if (!SOMFY_RX_ON) {
	#ifdef HAS_ASKSIN
				if (asksin_on) {
					somfy_restore_asksin = 1;
					asksin_on = 0;
				}
	#endif
	#ifdef HAS_MORITZ
				if (moritz_on) {
					somfy_restore_moritz = 1;
					moritz_on = 0;
				}
	#endif
			}
			somfy_rts_rx_start();
#endif
		} else if (in[2] == '0') {
			if (SOMFY_RX_ON)
				somfy_rts_func("Yx");
		}
		MULTICC_PREFIX();
		DC('Y');DC('e');DC(':');
		DC(SOMFY_RX_ON ? '1' : '0');
		DNL();
#endif
	} else if (in[1] == 'x') { 	// Reset Frequency back to Eeprom value
#ifdef USE_RF_MODE
	  set_RF_mode(RF_mode_off);
#else
#ifdef HAS_SOMFY_RTS_RX
	  somfy_rts_rx_off();
#endif
	  if(0) { ;
		#ifdef HAS_ASKSIN
		} else if (somfy_restore_asksin) {
			somfy_restore_asksin = 0;
			rf_asksin_init();
			asksin_on = 1;
			ccRX();
		#endif
		#ifdef HAS_MORITZ
		} else if (somfy_restore_moritz) {
			somfy_restore_moritz = 0;
			rf_moritz_init();
		#endif
		} else {
			ccInitChip(EE_CC1100_CFG);										// Set back to Eeprom Values
			if(tx_report) {                               // Enable RX
				ccRX();
			} else {
				ccStrobe(CC1100_SIDLE);
			}
		}
#endif
	}
}

#endif
