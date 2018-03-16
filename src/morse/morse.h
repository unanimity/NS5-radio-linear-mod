/*
 * morse.h
 *
 *  Created on: 2012. 12. 19.
 *      Author: OSSI
 */

#ifndef MORSE_H_
#define MORSE_H_

#include <asf.h>
#include "DAC.h"
#include "ad9954.h"

//#define MAX_DATA_SIZE 59

void morse_set_sendFlag(void);
void morse_clear_sendFlag(void);

void morse_set_WPM(uint8_t wpm);

void morse_init(uint8_t wpm);
uint8_t morse_is_ready(void);
void morse_send_bytes(uint8_t * bytes, bool async);
void morse_set_pa_level(uint16_t level);

#endif /* MORSE_H_ */
