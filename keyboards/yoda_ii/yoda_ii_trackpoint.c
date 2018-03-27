/* Copyright 2018 Jonathan A. Kollasch
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "ch.h"
#include "hal.h"

#include "debug.h"
#include "timer.h"
#include "host.h"
#include "report.h"
#include "wait.h"
#include "matrix.h"
#include "pointing_device.h"

#include QMK_KEYBOARD_H

#if defined(KEYBOARD_yoda_ii)
#define Y2TP_USE_I2C1
#define Y2TP_N 1
#define Y2TP_D 3
#define Y2TPS_N 1
#define Y2TPS_D 30
#elif defined(KEYBOARD_ht32_test)
#define Y2TP_USE_I2C0
#ifndef LINE_TPREQ
#define LINE_TPREQ PAL_LINE(GPIO_A, 15)
#endif
#define Y2TP_N 1
#define Y2TP_D 1
#endif

#define Y2TP_SCALE(n) (((int32_t)(n))*Y2TP_N/Y2TP_D)
#define Y2TP_SCROLL_SCALE(n) (((int32_t)(n))*Y2TPS_N/Y2TPS_D)

#define HT32_I2C0_IRQ_PRIORITY 6
#define HT32_I2C1_IRQ_PRIORITY 6


#if defined(Y2TP_USE_I2C0)
#define I2CU I2C0
#elif defined(Y2TP_USE_I2C1)
#define I2CU I2C1
#else
#error no I2C unit selected
#endif

static volatile union y2tp_report {
	int8_t   s[3];
	uint8_t  u[3];
	uint32_t w;
} y2tp_report;

static volatile struct y2tp_state {
	union y2tp_report rxb;
	uint8_t rxn;
	uint8_t txb[4];
	uint8_t txn;
} y2tp_state;

#if defined(Y2TP_USE_I2C0)
OSAL_IRQ_HANDLER(HT32_I2C0_IRQ_VECTOR) {
#elif defined(Y2TP_USE_I2C1)
OSAL_IRQ_HANDLER(HT32_I2C1_IRQ_VECTOR) {
#endif

	OSAL_IRQ_PROLOGUE();

#if defined(KEYBOARD_ht32_test)
	palClearLine(LINE_LED2);
#endif

	uint32_t sr = I2CU->SR;
	uint8_t dr;

	switch (sr & (I2C_SR_TXNRX|I2C_SR_BUSBUSY|I2C_SR_TXDE|I2C_SR_RXDNE|I2C_SR_BUSERR|I2C_SR_RXNACK|I2C_SR_ADRS|I2C_SR_STO)) {
	case I2C_SR_TXNRX|I2C_SR_BUSBUSY|I2C_SR_TXDE|I2C_SR_ADRS:
		/* FALLTHROUGH */
	case I2C_SR_TXNRX|I2C_SR_BUSBUSY|I2C_SR_TXDE:
		if (y2tp_state.txn < sizeof(y2tp_state.txb)) {
			dr = y2tp_state.txb[y2tp_state.txn];
			I2CU->DR = dr;
		}
		y2tp_state.txn++;
		break;
	case I2C_SR_BUSBUSY|I2C_SR_ADRS:
		y2tp_state.rxn = 0;
		break;
	case I2C_SR_BUSBUSY|I2C_SR_RXDNE:
	case I2C_SR_RXDNE|I2C_SR_STO:
		dr = I2CU->DR;
		if (y2tp_state.rxn < sizeof(y2tp_state.rxb))
			y2tp_state.rxb.u[y2tp_state.rxn] = dr;
		y2tp_state.rxn++;
		if (y2tp_state.rxn >= 3 && (I2C_SR_STO & sr)) {
			y2tp_report = y2tp_state.rxb;
		}
		break;
	case I2C_SR_BUSBUSY|I2C_SR_RXNACK:
	case I2C_SR_RXNACK|I2C_SR_STO:
		I2CU->SR = (I2C_SR_TOUTF|I2C_SR_BUSERR|I2C_SR_RXNACK|I2C_SR_ARBLOS) & sr;
		/* FALLTHROUGH */
	case I2C_SR_STO:
		if (y2tp_state.rxn >= 3 && (I2C_SR_STO & sr)) {
			y2tp_report = y2tp_state.rxb;
		}
		break;
	default:
#if defined(KEYBOARD_ht32_test)
		palSetLine(LINE_LED2);
#endif
#if defined(KEYBOARD_ht32_test)
		while (true)
			;
#endif
		break;
	}

	OSAL_IRQ_EPILOGUE();
}

void
pointing_device_init(void)
{
#if 0
	debug_config.enable = 1;
	debug_config.mouse = 1;
#endif

#if 0
	if (debug_mouse) {
		print("enabling i2c trackpoint interface\n");
	}
#endif

#if defined(Y2TP_USE_I2C0)
	RSTCU->APBPRSTR0 |= RSTCU_APBPRSTR0_I2C0RST;
	CKCU->APBCCR0 |= CKCU_APBCCR0_I2C0EN;
#elif defined(Y2TP_USE_I2C1)
	RSTCU->APBPRSTR0 |= RSTCU_APBPRSTR0_I2C1RST;
	CKCU->APBCCR0 |= CKCU_APBCCR0_I2C1EN;
#else
#error
#endif
#if defined(KEYBOARD_yoda_ii)
	palSetLine(LINE_TPPWR);
	palSetLineMode(LINE_TPPWR, PAL_MODE_OUTPUT_OPENDRAIN|PAL_HT32_MODE_INE);
	palSetGroupMode(GPIO_C, 0x3, 9, PAL_MODE_HT32_AF(AFIO_I2C)); /* I2C1 1653/1654 */
	palSetLine(LINE_TPREQ);
	palSetLineMode(LINE_TPREQ, PAL_MODE_OUTPUT_OPENDRAIN|PAL_HT32_MODE_PU|PAL_HT32_MODE_INE);
#elif defined(KEYBOARD_ht32_test)
	palSetGroupMode(GPIO_B, 0x3, 0, PAL_MODE_HT32_AF(AFIO_I2C)); /* I2C0 1655/1656 */
	palSetLineMode(LINE_TPREQ, PAL_MODE_OUTPUT_OPENDRAIN|PAL_HT32_MODE_PU);
#else
#error
#endif
	I2CU->CR = I2C_CR_I2CEN|I2C_CR_AA;
	I2CU->ADDR = 0x30;
	I2CU->SHPGR = (HT32_CK_AHB_FREQUENCY/100000*40/100)-6;
	I2CU->SLPGR = (HT32_CK_AHB_FREQUENCY/100000*60/100)-6;
	I2CU->IER |= I2C_IER_TXDEIE|I2C_IER_RXDNEIE|I2C_IER_BUSERRIE|I2C_IER_RXNACKIE|I2C_IER_ADRSIE|I2C_IER_STOIE;

#if defined(Y2TP_USE_I2C0)
	nvicEnableVector(I2C0_IRQn, HT32_I2C0_IRQ_PRIORITY);
#elif defined(Y2TP_USE_I2C1)
	nvicEnableVector(I2C1_IRQn, HT32_I2C1_IRQ_PRIORITY);
#endif

#if defined(KEYBOARD_ht32_test)
	palClearLine(LINE_TPREQ);
	wait_us(1000);
	palSetLine(LINE_TPREQ);
#elif defined(KEYBOARD_yoda_ii)
	palClearLine(LINE_TPPWR);
#endif
}

static void
dump_state(void)
{
	if (!debug_mouse)
		return;

	xprintf("%ud ", timer_read());
#if 0
	print("mouse_report raw: [");
	phex(mouse_report.buttons); print("|");
	print_hex8((uint8_t)mouse_report.x); print(" ");
	print_hex8((uint8_t)mouse_report.y); print("]\n");
#endif
	print("raw: [");
	print_hex8((uint8_t)y2tp_report.u[0]); print(",");
	print_hex8((uint8_t)y2tp_report.u[1]); print(",");
	print_hex8((uint8_t)y2tp_report.u[2]); print("]\n");
}

__attribute__ ((weak))
bool y2tp_is_scrolling(void)
{
	return false;
}

void
pointing_device_task(void)
{
	static const union y2tp_report zero_report = { .s = { 0, 0 } };
	static report_mouse_t mouse_report = {
		.buttons = 0,
	};
	static int16_t ax, ay;

#if defined(KEYBOARD_yoda_ii)
	uint8_t r8 = matrix_get_row(8);
	uint8_t new_buttons  =
	    ((r8 >> 5) & 1) << 0 | /* MOUSE_BTN1 */
	    ((r8 >> 7) & 1) << 1 | /* MOUSE_BTN2 */
	    ((r8 >> 6) & 1) << 2;  /* MOUSE_BTN3 */
	new_buttons |= y2tp_report.s[0] & 0x1;  /* Z tap button */
#elif defined(KEYBOARD_ht32_test)
	uint8_t new_buttons = (matrix_get_row(13) >> 0) & 0x3;
#endif

	if (y2tp_report.s[0] || y2tp_report.s[1] || y2tp_report.s[2]) {
		dump_state();

		ax += y2tp_report.s[1];
		ay -= y2tp_report.s[2];

		y2tp_report = zero_report;
	}

	if (Y2TP_SCALE(ax) ||
	    Y2TP_SCALE(ay) ||
	    Y2TP_SCROLL_SCALE(ay) ||
	    Y2TP_SCROLL_SCALE(ax) ||
	    mouse_report.buttons ^ new_buttons) {
		mouse_report.buttons = new_buttons;
		if (y2tp_is_scrolling() && (Y2TP_SCROLL_SCALE(ay) || Y2TP_SCROLL_SCALE(ax))) {
			mouse_report.x = 0;
			mouse_report.y = 0;
			mouse_report.v = -Y2TP_SCROLL_SCALE(ay);
			mouse_report.h = Y2TP_SCROLL_SCALE(ax);
			ax = ay = 0;
		} else if (!y2tp_is_scrolling() && (Y2TP_SCALE(ax) || Y2TP_SCALE(ay))) {
			mouse_report.x = Y2TP_SCALE(ax);
			mouse_report.y = Y2TP_SCALE(ay);
			mouse_report.v = 0;
			mouse_report.h = 0;
			ax = ay = 0;
		} else {
			mouse_report.x = 0;
			mouse_report.y = 0;
			mouse_report.v = 0;
			mouse_report.h = 0;
		}
		host_mouse_send(&mouse_report);
	}

	//pointing_device_send();
}

void
y2tp_request(uint8_t x)
{
	y2tp_state.txn = 0;
	y2tp_state.txb[0] = 0x7e;
	y2tp_state.txb[1] = 0x44;
	y2tp_state.txb[2] = 0x01;
	y2tp_state.txb[3] = x;

	palClearLine(LINE_TPREQ);
	wait_us(100);
	palSetLine(LINE_TPREQ);
}
