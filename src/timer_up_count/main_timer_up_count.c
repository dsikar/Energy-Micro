/*****************************************************************************
 * @file main_timer_up_count.c
 * @brief TIMER Up Count Demo Application
 * @author Silicon Labs
 * @version 1.09
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

#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_gpio.h"
#include "em_system.h"
#include "em_timer.h"
#include "em_chip.h"

// #define TEST_ENVIRONMENT

#if defined TEST_ENVIRONMENT
  #define LED_PORT gpioPortC
  #define LED_PIN  0
  #define REED_PORT gpioPortB
  #define REED_PIN  1
#else
  #define LED_PORT gpioPortA
  #define LED_PIN  3
  #define REED_PORT gpioPortB
  #define REED_PIN  11
#endif

#define BEACON_ENABLE_PORT gpioPortC
#define BEACON_ENABLE_PIN  9
#define SM1_ENABLE_PORT gpioPortC
#define SM1_ENABLE_PIN  12

// Alarm state struct
/* structure for alarm counter */
typedef struct
{
	bool		IsAlarmOn;
	uint8_t		AlarmOnCounter;
	uint8_t		Countdown;
} ALARM_COUNTER;


/*************************local function and structure ************************/
// static volatile TOMES_DATA 	st_ToneData;
// The actual structure definition should go in timer.h as this is timer related

static volatile ALARM_COUNTER st_AlarmCounter;

void initAlarmCounter(void)
{
	st_AlarmCounter.IsAlarmOn = false;
	st_AlarmCounter.AlarmOnCounter = 0;
	st_AlarmCounter.Countdown = 2; //
}
/* 13671 Hz -> 14Mhz (clock frequency) / 1024 (prescaler)
  Setting TOP to 27342 results in an overflow each 2 seconds
  Setting TOP to 13671 results in an overflow each 1 seconds  */
#define TOP 27342

// Comment Template - USE!
/*****************************************************************************//**
 * @brief
 *
 * @details
 *
 *
 * @param
 *
 * @return
 *
 ******************************************************************************/


/*****************************************************************************//**
 * @brief Read Reed port
 *
 * @details By Applying magnet,
 *
 *
 * @param
 *
 * @return <ture/false>
 *
 ******************************************************************************/
bool hal_readREED(void)
{
	return GPIO_PinInGet(REED_PORT, REED_PIN);
}

/*****************************************************************************//**
 * @brief			Set GPIO pin
 *
 * @details		Set GPIO as high or low borrowed from GpioPins.c for now
 *
 * @param			port number
 * @param			pin	 number
 * @param			high (true or false)
 *
 * @return
 *
 ******************************************************************************/
void SetPin(GPIO_Port_TypeDef port,uint8_t pin,bool high)
{
	if (high)
		GPIO->P[port].DOUTSET = (uint32_t)(1 << pin);
	else
		GPIO->P[port].DOUTCLR = (uint32_t)(1 << pin);
}

/**************************************************************************//**
 * @brief TIMER2_IRQHandler
 * Interrupt Service Routine TIMER0 Interrupt Line
 *****************************************************************************/
void TIMER2_IRQHandler(void)
{ 
  /* Clear flag for TIMER0 overflow interrupt */
  TIMER_IntClear(TIMER2, TIMER_IF_OF);
  
  /* Toggle LED ON/OFF */
  // GPIO_PinOutToggle(LED_PORT, LED_PIN);

  if(!(hal_readREED())) {
	  /* ENABLE PWR */
	  // GPIO->P[BEACON_ENABLE_PORT].DOUTSET = 1 << BEACON_ENABLE_PIN;
	  // GPIO->P[SM1_ENABLE_PORT].DOUTSET = 1 << SM1_ENABLE_PIN;
	  // GPIO->P[LED_PORT].DOUTSET = 1 << LED_PIN;
	  SetPin(LED_PORT, LED_PIN, 1);
  }
  else {
	  // GPIO->P[LED_PORT].DOUTCLR = 1 << LED_PIN;
	  SetPin(LED_PORT, LED_PIN, 0);
	  /* DISABLE PWR */
	  // GPIO->P[BEACON_ENABLE_PORT].DOUTCLR = 1 << BEACON_ENABLE_PIN;
	  // GPIO->P[SM1_ENABLE_PORT].DOUTCLR = 1 << SM1_ENABLE_PIN;
  }
  /*
  if(!(hal_readREED()))
	  st_AlarmCounter.AlarmOnCounter = st_AlarmCounter.Countdown;
  else
	  st_AlarmCounter.AlarmOnCounter = (st_AlarmCounter.AlarmOnCounter > 0 ? st_AlarmCounter.AlarmOnCounter-- : 0);
  */
}



// DIP SWITCH POWER DISABLE
/**************************************************************************//**
 * @brief  Main function
 * Main is called from __iar_program_start, see assembly startup file
 *****************************************************************************/
int main(void)
{  
  /* Initialize chip */
  CHIP_Init();
    
  /* Enable clock for GPIO module */
  CMU_ClockEnable(cmuClock_GPIO, true);
  
  /* Enable clock for TIMER0 module */
  CMU_ClockEnable(cmuClock_TIMER2, true);
  
  /* Configure pin as push pull output for LED/pin drive */
  GPIO_PinModeSet(LED_PORT, LED_PIN, gpioModePushPull, 0);
  /* Configure pin as push pull output for Beacon enable/pin drive */
  GPIO_PinModeSet(BEACON_ENABLE_PORT, BEACON_ENABLE_PIN, gpioModePushPull, 0);
  /* Configure pin as push pull output for SM1 enable/pin drive */
  GPIO_PinModeSet(SM1_ENABLE_PORT, SM1_ENABLE_PIN, gpioModePushPull, 0);
  /* Configure pin as input for REED/pin drive */
  GPIO_PinModeSet(REED_PORT, REED_PIN, gpioModeInput, 0);

  GPIO->P[BEACON_ENABLE_PORT].DOUTSET = 1 << BEACON_ENABLE_PIN;
  GPIO->P[SM1_ENABLE_PORT].DOUTSET = 1 << SM1_ENABLE_PIN;

  SetPin(LED_PORT, LED_PIN, 0);

  // Set Alarm state

  /* Select TIMER2 parameters */
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
  TIMER_IntEnable(TIMER2, TIMER_IF_OF);
  
  /* Enable TIMER0 interrupt vector in NVIC */
  NVIC_EnableIRQ(TIMER2_IRQn);
  
  /* Set TIMER Top value */
  TIMER_TopSet(TIMER2, TOP);
  
  /* Configure TIMER */
  TIMER_Init(TIMER2, &timerInit);
  
  while(1)
  {
    /* Go to EM1 */
     EMU_EnterEM1();
	  // bool truth = hal_readREED();
  } 
 
}

