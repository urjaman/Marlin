/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2019 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * Driver for the ledthing 4-ch LED driver
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(LEDTHING_LED)

#include "ledthing.h"
#include "leds.h"
#include "../../module/temperature.h"

extern "C" void ledthing_uart_tx(uint8_t v);
#ifdef __AVR__

#include <util/crc16.h>

asm(
"\nledthing_dly_half_bit:" // Note: this fn doesnt touch carry
	"\n\tnop" // 19.667
	"\n\tnop" //
	"\n\tldi r25, 19" // this is X above
"\n0:\tdec r25"
	"\n\tbrne 0b"
	"\n\tret"

	"\n\n\t.global ledthing_uart_tx"
"\nledthing_uart_tx:"
	"\n\tcli"
	"\n\tsbi 0x4, 5" // DDRB
	"\n\trcall ledthing_dly_half_bit" // receiver only checks for half of the stop bit, and we only send half, so
	"\n\tcbi 0x5, 5" // PORTB // start bit
	"\n\tsec"		// inject end marker bit to r16
	"\n\tror r24"		// and pull bit0 to C(arry)
"\n0:\trcall ledthing_dly_half_bit"	// dlys dont touch carry
	"\n\trcall ledthing_dly_half_bit"
	"\n\tcbi 0x5, 5" // PORTB
	"\n\tbrcc 1f"			// this works like a skip now, so cycles effectively 1
	"\n\tsbi 0x5, 5" // PORTB
"\n1:\tlsr r24"		// pull next bit to Carry and test result for 0 (if so, end marker eaten)
	"\n\tbrne 0b"
	"\n\tnop"	// brne normalization to 2 cycles :P
	// wait to send last bit
	"\n\trcall ledthing_dly_half_bit"
	"\n\trcall ledthing_dly_half_bit"
	"\n\tnop" // cbi
	"\n\tnop" // brcc
	// send the stop bit
	"\n\tsbi 0x5, 5" // PORTB
	"\n\tnop" // lsr
	"\n\tnop" // brne1
	"\n\tnop" // brne2
	"\n\trcall ledthing_dly_half_bit"
	"\n\tsei"
	"\n\tcbi 0x4, 5" // DDRB
	"\n\tret"
);

static uint8_t ledthing_read_pin(void) {
	return PINB & _BV(5);
}

#else
#error LEDTHING UART TX unimplemented on non-AVR

static uint8_t
_crc_ibutton_update(uint8_t crc, uint8_t data) {
	uint8_t i;
	crc = crc ^ data;
	for (i = 0; i < 8; i++) {
		if (crc & 0x01)
            		crc = (crc >> 1) ^ 0x8C;
                else
                	crc >>= 1;
        }
        return crc;
}
#endif

static uint8_t crc8(const uint8_t *b, uint8_t cnt) {
	uint8_t crc = 0;
	for (uint8_t i = 0; i < cnt; i++) {
		crc = _crc_ibutton_update(crc, b[i]);
	}
	return crc;
}

static uint8_t make_frame(uint8_t *frame, const uint8_t *buf, uint8_t cnt) {
	frame[0] = 0x9D;
	memcpy(frame+1,buf,cnt);
	frame[cnt+1] = crc8(buf,cnt);
	return cnt+2;
}

static uint8_t make_nop(uint8_t *frame) {
	const uint8_t nop[1] = { 0 };
	// Make a long sync-frame for the NOP to synchronize better
	for (uint8_t i=0;i<12;i++) frame[i] = 0x9D;
	return make_frame(frame+12, nop, 1) + 12;
}

static uint8_t make_set_nch(uint8_t *frame, uint8_t ch1, uint8_t nch, uint16_t *val) {
	uint8_t buf[10];
	buf[0] = 1;
	buf[1] = (ch1 << 4) | nch;
	for (uint8_t ci=0; ci < nch; ci++) {
		buf[2 + 2*ci] = val[ci] & 0xFF;
		buf[3 + 2*ci] = val[ci] >> 8;
	}
	return make_frame(frame,buf,2+(2*nch));
}

static uint8_t make_set_rgb(uint8_t *frame, uint16_t r, uint16_t g, uint16_t b) {
	uint16_t grb[3] = { g, r, b };
	return make_set_nch(frame, 0, 3, grb);
}

static uint8_t make_set_bedred(uint8_t *frame, uint16_t red) {
	uint16_t rv[1] = { red };
	return make_set_nch(frame, 3, 1, rv);
}

static uint8_t wait_pin_state(uint8_t state, uint8_t ms) {
	millis_t nw = millis();
	millis_t passed;
	do {
		if (ledthing_read_pin() == state) break;
		passed = millis() - nw;
	} while (passed < ms);
	if (ledthing_read_pin() != state) return 1;
	return 0;
}

static uint8_t do_cmd(uint8_t *frame, uint8_t len) {
	for (uint8_t n = 0; n < len; n++) ledthing_uart_tx(frame[n]);
	if (wait_pin_state(1,2)) return 1;
	if (wait_pin_state(0,5)) return 1;
	if (wait_pin_state(1,2)) return 1;
	return 0;
}


static const uint16_t ledcurve[32] PROGMEM = {
	   0,     4,     8,    12,    16,    20,    24,    32,
	  44,    64,    88,   120,   168,   232,   316,   432,
	 596,   816,  1116,  1528,  2092,  2860,  3912,  5352,
	7320, 10012, 13696, 18732, 25616, 35036, 47916, 65535
};



static uint16_t curve_lookup(uint8_t input) {
	uint8_t tgti = input>>3;
	uint16_t tgt = pgm_read_word(&(ledcurve[tgti]));
	if (tgti < 31) {
		uint16_t nxt = pgm_read_word(&(ledcurve[tgti+1]));
		uint16_t stp = nxt - tgt;
		stp = (stp * input&7) / 8;
		tgt += stp;
	}
	return tgt;
}

void ledthing_set_led_color(const LEDColor &color) {
	uint8_t frame[16];
	uint8_t fl;
	fl = make_set_rgb(frame, curve_lookup(color.r), curve_lookup(color.g), curve_lookup(color.b));
	for (uint8_t n=0; n < 3; n++) {
		if (!do_cmd(frame, fl)) break;
	}
}


void ledthing_init(void) {
	uint8_t frame[16];
	uint8_t fl;
	PORTB |= _BV(5);
	fl = make_nop(frame);
	uint8_t n;
	for (n=0; n < 3; n++) {
		if (!do_cmd(frame, fl)) break;
	}
//	if (n==3) return;
//	ledthing_set_led_color(LEDColor(0,0,0));
}

void ledthing_handle_status(void) {
#if HAS_HEATED_BED
	static uint8_t state = 0;
	static int old_red = -1;
	static millis_t next_status_led_update_ms = 0;
 	if (ELAPSED(millis(), next_status_led_update_ms)) {
		next_status_led_update_ms += 500; // Update every 0.5s
		int temp = (int) (thermalManager.degBed() + 0.5);
		uint8_t state_intensity[4] = { 0, 32, 128, 180 };
		uint8_t temp_us = (state * 10) + 50;
		uint8_t temp_ds = temp_us - 15;
		uint8_t new_state = state;
		if ((temp >= temp_us)&&(new_state < 3)) new_state++;
		else if ((temp < temp_ds)&&(new_state)) new_state--;
		state = new_state;

		const int new_red = state_intensity[state];
		if (new_red != old_red) {
			uint8_t frame[16];
			uint8_t fl = make_set_bedred(frame, curve_lookup(new_red));
			for (uint8_t n=0; n < 3; n++) {
				if (!do_cmd(frame, fl)) break;
			}
			old_red = new_red;
		}
	}
#endif
}

#endif
