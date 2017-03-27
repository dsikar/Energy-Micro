/******************************************************************************
 * @file gstq.c
 * @brief Play national anthem alarm tone
 * @author Anonymous
 * @version 1.01
 ******************************************************************************
 * @section License
 * <b>(C) Copyright 2014 Silicon Labs, http://www.silabs.com</b>
 *******************************************************************************
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 *    claim that you wrote the original software.
 * 2. Altered source versions must be plainly marked as such, and must not be
 *    misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 *
 * DISCLAIMER OF WARRANTY/LIMITATION OF REMEDIES: Silicon Labs has no
 * obligation to support this Software. Silicon Labs is providing the
 * Software "AS IS", with no express or implied warranties of any kind,
 * including, but not limited to, any implied warranties of merchantability
 * or fitness for any particular purpose or warranties against infringement
 * of any proprietary rights of a third party.
 *
 * Silicon Labs will not be liable for any consequential, incidental, or
 * special damages, or any other relief, or for any claim by any third party,
 * arising from your use of this Software.
 *
 ******************************************************************************/
#include "gstq.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"
#include "em_rtc.h"

// Initialise count and song struct
uint8_t iCountNote = 0;
note notes[SONG_LENGTH] =
{
	// BAR 1 - 19 notes and pauses
	{ .pitch = G4, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	{ .pitch = G4, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	{ .pitch = A4, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	// BAR 2
	{ .pitch = F4_SHARP, .duration = CROTCHET },
	{ .pitch = REST, .duration = QUAVER },
	{ .pitch = G4, .duration = QUAVER },
	{ .pitch = A4, .duration = CROTCHET },
	// BAR 3
	{ .pitch = B4, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	{ .pitch = B4, .duration = CROTCHET },
	{ .pitch = C5, .duration = QUAVER },
	// BAR 4
	{ .pitch = B4, .duration = DOTTED_CROTCHET },
	{ .pitch = A4, .duration = QUAVER },
	{ .pitch = G4, .duration = CROTCHET },
	// BAR 5
	{ .pitch = A4, .duration = CROTCHET },
	{ .pitch = G4, .duration = CROTCHET },
	{ .pitch = F4_SHARP, .duration = CROTCHET },

	// BAR 6 -24
	{ .pitch = G4, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	{ .pitch = G4, .duration = QUAVER },
	{ .pitch = A4, .duration = QUAVER },
	{ .pitch = B4, .duration = QUAVER },
	{ .pitch = C5, .duration = QUAVER },
	// BAR 7
	{ .pitch = D5, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	{ .pitch = D5, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	{ .pitch = D5, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	// BAR 8
	{ .pitch = D5, .duration = DOTTED_CROTCHET },
	{ .pitch = C5, .duration = QUAVER },
	{ .pitch = B4, .duration = CROTCHET },
	// BAR 9
	{ .pitch = C5, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	{ .pitch = C5, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	{ .pitch = C5, .duration = DOTTED_QUAVER },
	{ .pitch = REST, .duration = SEMIQUAVER },
	// BAR 10
	{ .pitch = C5, .duration = DOTTED_CROTCHET },
	{ .pitch = B4, .duration = QUAVER },
	{ .pitch = A4, .duration = CROTCHET },

	// BAR 11 - 13
	{ .pitch = B4, .duration = CROTCHET },
	{ .pitch = C5, .duration = QUAVER },
	{ .pitch = B4, .duration = QUAVER },
	{ .pitch = A4, .duration = QUAVER },
	{ .pitch = G4, .duration = QUAVER },
	// BAR 12
	{ .pitch = B4, .duration = DOTTED_CROTCHET },
	{ .pitch = C5, .duration = QUAVER },
	{ .pitch = D5, .duration = CROTCHET },
	// BAR 13
	{ .pitch = E5, .duration = QUAVER },
	{ .pitch = C5, .duration = QUAVER },
	{ .pitch = B4, .duration = CROTCHET },
	{ .pitch = A4, .duration = CROTCHET },
	// BAR 14
	{ .pitch = G4, .duration = DOTTED_MINIM }
};

/**************************************************************************//**
 * @brief GetNextNote
 * Get note counter position in note struct array.
 * @return next note to be played.
 *****************************************************************************/
note GetNextNote() {
	note playNote;
	playNote.pitch = notes[iCountNote].pitch;
	playNote.duration = notes[iCountNote].duration;
	iCountNote = (iCountNote < SONG_LENGTH ? iCountNote + 1 : 0);
	return playNote;
}

/**************************************************************************//**
 * @brief IsRest
 * Sounder state based on parsed pitch. REST means no sound. Anything else,
 * sound.
 * @return true if output pin should be toggled at pitch frequency.
 *****************************************************************************/
boolean IsRest(void) {
	if(notes[iCountNote].pitch == REST) {
		return TRUE;
	}
	return FALSE;
}

/**************************************************************************//**
 * @brief LoadNote
 * Set pitch timer (RTC) and duration timer (TIMER0);
 *****************************************************************************/
void LoadNote(void) {
	note playNote = GetNextNote();
	// Set new pitch
	RTC_CompareSet(0, playNote.pitch);
	// Reset counter
	RTC_CounterReset();
	// TIMER_TopSet(TIMER0, playNote.duration);
	/* FROM datasheet:
	 * 19.3.1.5 Top Value Buffer
The TIMERn_TOP register can be altered either by writing it directly or by writing to the TIMER_TOPB
(buffer) register. When writing to the buffer register the TIMERn_TOPB register will be written to
TIMERn_TOP on the next update event. Buffering ensures that the TOP value is not set below the
actual count value. The TOPBV flag in TIMERn_STATUS indicates whether the TIMERn_TOPB register
contains data that have not yet been written to the TIMERn_TOP register (see Figure 19.5 (p. 278) .
pg 278
	 */
	TIMER_TopBufSet(TIMER0, playNote.duration);
}

/**************************************************************************//**
 * @brief RTC_IRQHandler
 * Interrupt Service Routine RTC Interrupt Line
 *****************************************************************************/
void RTC_IRQHandler(void)
{
  /* Get and clear interrupt flags */
  uint32_t flags = RTC_IntGet();
  RTC_IntClear(flags);
  /* Toggle LED if overflow flag was set */
  if ( flags & RTC_IF_COMP0 ) {
    // GPIO_PinOutToggle(LED_PORT, LED_PIN);
	if(!IsRest()) {
		GPIO_PinOutToggle(PA14_PORT, PA14_PIN);
	} else {
	      /* Turn LED off */
	      GPIO_PortOutSetVal(PA14_PORT, 0x0, 1<<PA14_PIN);
	}
  }
}

/**************************************************************************//**
 * @brief TIMER0_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER0_IRQHandler(void)
{
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER0, TIMER_IF_OF);
  /* Load next note (pitch and duration) */
  LoadNote();
}

/**************************************************************************//**
 * @brief initRTC
 * Interrupt Service Routine TIMER0 Interrupt Line
 ****************************************************************************/
void initRTC(void)
{

  /* Configure to overflow every 50ms */
//  RTC_CompareSet(0, 50 * RTC_FREQ / 1000);

  /* Configure to overflow every 1/784 s
   * i.e TC_FREQ 32768 / 728 ~ 42
   * */
	RTC_CompareSet(0, REST);
	// Reset counter
	RTC_CounterReset();

  /* Configure and enable RTC. Fill in this struct */
  RTC_Init_TypeDef rtcInit;
  rtcInit.comp0Top = true;
  rtcInit.debugRun = false;
  rtcInit.enable   = true;
  RTC_Init(&rtcInit);

  /* Enable overflow interrupt */
  RTC_IntEnable(RTC_IEN_COMP0);
  NVIC_EnableIRQ(RTC_IRQn);

}

int main(void)
{
  /* Initialize chip */
  CHIP_Init();

  /* Enable clock for GPIO */
  CMU_ClockEnable(cmuClock_GPIO, true);

  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER0, true);

  /* Select TIMER0 parameters */
  TIMER_Init_TypeDef timerInit =
  {
    .enable     = true,
    .debugRun   = true,
    .prescale   = timerPrescale1024,
    .clkSel     = timerClkSelHFPerClk,
    .fallAction = timerInputActionNone,
    .riseAction = timerInputActionNone,
    .mode       = timerModeUp,
    .dmaClrAct  = false,
    .quadModeX4 = false,
    .oneShot    = false,
    .sync       = false,
  };

  /* Enable overflow interrupt */
  TIMER_IntEnable(TIMER0, TIMER_IF_OF);

  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER0_IRQn);

  /* Set TIMER Top value - start with a long rest*/
  TIMER_TopSet(TIMER0, DOTTED_MINIM);

  /* Configure TIMER */
  TIMER_Init(TIMER0, &timerInit);

  /* Configure LED pin as push/pull output */
  GPIO_PinModeSet(LED_PORT,         /* Port */
                  LED_PIN,          /* Pin */
                  gpioModePushPull, /* Mode */
                  0 );              /* Output value */

  /* Configure LED pin as push/pull output */
  GPIO_PinModeSet(PA14_PORT,         /* Port */
		  	  	  PA14_PIN,          /* Pin */
                  gpioModePushPull, /* Mode */
                  0 );              /* Output value */

  /* Start LFXO and wait until it is stable */
  CMU_OscillatorEnable(cmuOsc_LFXO, true, true);

  /* Select LFXO as the clock source for RTC clock domain */
  CMU_ClockSelectSet(cmuClock_LFA,cmuSelect_LFXO);

  /* Enable clock to RTC */
  CMU_ClockEnable(cmuClock_RTC, true);

  /* Enable clock to the interface of the low energy modules */
  CMU_ClockEnable(cmuClock_CORELE, true);

  /* Configure RTC */
  initRTC();

  /* Stay in this loop forever */
  while (1) {
    /* Enter EM2 */
	// EMU_EnterEM2(false);
	EMU_EnterEM1();
  }
}
