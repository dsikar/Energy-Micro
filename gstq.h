/*
 * gstq.h
 *
 *  Created on: 27 Mar 2017
 *      Author: dsikar
 */

#ifndef GSTQ_H_
#define GSTQ_H_

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"
#include "em_rtc.h"

/************************* Local macros for pin control************************/
// Enable
#define LED_PORT 	gpioPortC
#define LED_PIN  	0
// PWM
#define PA14_PORT 	gpioPortA
#define PA14_PIN 	14

/************************* Local timing macros ************************/
// Clock frequency
#define RTC_FREQ 32768

/* Musical note frequencies for clock running at RTC_FREQ
 * To calculate, divide clock frequency by note frequency
 * multiplied by 2 e.g.
 * G ~ 392 Hz ~ 32768 / 784 ~ 42 cycles on a 32768 Hz
 * clock.
 */

// HARDCODED TO REDUCE RUNTIME ARITHMETIC
#define F4_SHARP 44 // RTC_FREQ / 370 * 2) // 44 // 370.0 Hz
#define G4 42 //RTC_FREQ / 392 * 2) // 42 // 392 Hz
#define A4 37 // RTC_FREQ / 440 * 2) // 37 // 440 Hz
#define B4 33 // RTC_FREQ / 494 * 2) // 33 // 493.9
#define C5 31 // RTC_FREQ / 523 * 2) // 31 // 523.3
#define D5 28 // RTC_FREQ / 587 * 2) // 28 // 587.3
#define E5 25 // RTC_FREQ / 659 * 2) // 25 // 659.3
#define REST 50 // Special case, no PWM, pin will be set to 0.

// Song tempo - hardcoded for now, parametrised in future
// Use judiciously to avoid floating point arithmetic and division by zero
// e.g. 60, 120, 180 should be ok for now
#define BPM 60
// Parameters to define note duration
#define SECONDS_PER_MINUTE 60
#define CROTCHET_DIV 4 // 1 Beat divided by 4
#define QUAVER_DIV 8   // 1 Beat divided by 8 and so on
#define DOTTED_QUAVER_DIV 6
#define DOTTED_CROTCHET_DIV 3
#define DOTTED_MINIM_DIV 1

/* Tempo for TIMER0 clock with 1024 prescaler
 * Assuming national anthem played at 60 BPM
 * e.g. 1 second (crochet) ~ 13672 cycles
 */

// HARDCODED TO REDUCE RUNTIME ARITHMETIC
// 1 second at 60 BPM
#define DOTTED_MINIM 32768 //  RTC_FREQ / (BPM / SECONDS_PER_MINUTE) * DOTTED_MINIM_DIV // RTC_FREQ // Duration = unit e.g. crotchet (1/60 of a minute)
#define DOTTED_CROTCHET 20508 // RTC_FREQ / (BPM / SECONDS_PER_MINUTE) * DOTTED_CROTCHET_DIV // 20508
#define CROTCHET 13672 // RTC_FREQ / (BPM / SECONDS_PER_MINUTE) * CROTCHET_DIV
#define DOTTED_QUAVER 10254
#define QUAVER 6836 // RTC_FREQ / (BPM / SECONDS_PER_MINUTE) * DOTTED_QUAVER_DIV // 6836
#define SEMIQUAVER 3418
// HARDCODED TO AVOID RUNTIME ARITHMETIC
#define SONG_LENGTH 55 // (sizeof(notes) / sizeof(note) - 1) ~ number of notes minus 1 ~ zero indexed

/*******************************************************************************
 *******************************   STRUCTS   ***********************************
 ******************************************************************************/

typedef struct {
	uint32_t pitch;
	uint32_t duration;
} note;

typedef enum { FALSE, TRUE } boolean;

/************************* GSTQ functions ************************/
note GetNextNote();
boolean IsRest(void);
void LoadNote(void);
void initRTC(void);

#endif /* GSTQ_H_ */
