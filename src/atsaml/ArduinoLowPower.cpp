#define ARDUINO_ARCH_SAMD

#if defined(ARDUINO_ARCH_SAMD)
#include "ArduinoLowPower.h"

#if (SAML21)
#include "sam.h"

void ArduinoLowPowerClass::idle() {
  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
}

void ArduinoLowPowerClass::idle(uint32_t millis) {
  setAlarmIn(millis);
  idle();
}

void ArduinoLowPowerClass::sleep() {
#ifndef USB_DISABLED
  if (SERIAL_PORT_USBVIRTUAL) {
    USBDevice.standby();
  } else {
    USBDevice.detach();
    restoreUSBDevice = true;
  }
#endif
  GCLK_GENCTRL_Type gclkConfig;
  gclkConfig.reg = 0;
  gclkConfig.reg = GCLK->GENCTRL[0].reg;
  gclkConfig.bit.SRC =
      GCLK_GENCTRL_SRC_OSC16M_Val; // GCLK_GENCTRL_SRC_OSCULP32K_Val
                                   // ;//GCLK_GENCTRL_SRC_OSC16M_Val
  GCLK->GENCTRL[0].reg = gclkConfig.reg;

  while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(0)) {
    /* Wait for synchronization */
  };
  OSCCTRL->OSC16MCTRL.reg |= OSCCTRL_OSC16MCTRL_ONDEMAND;

  /* Clear performance level status */
  PM->INTFLAG.reg = PM_INTFLAG_PLRDY;
  /* Switch performance level to PL0 - best power saving */
  PM->PLCFG.reg = PM_PLCFG_PLSEL_PL0_Val;
  while (!PM->INTFLAG.reg) {
    ;
  }

  OSCCTRL_DFLLCTRL_Type dfllCtrlSlp;
  dfllCtrlSlp.reg = OSCCTRL->DFLLCTRL.reg;
  dfllCtrlSlp.bit.ENABLE = 0;
  OSCCTRL->DFLLCTRL.reg = dfllCtrlSlp.reg;

  // disable DFLL GCLK
  /* Disable the peripheral channel 0 ( DFLL )*/
  GCLK->PCHCTRL[0].reg &= ~GCLK_PCHCTRL_CHEN;

  while (GCLK->PCHCTRL[0].reg & GCLK_PCHCTRL_CHEN) {
    /* Wait for clock synchronization */
  }

  // disable xosc32k clock
  OSC32KCTRL->XOSC32K.reg &= ~OSC32KCTRL_XOSC32K_ENABLE;

  // disable generator 1
  GCLK->GENCTRL[1].bit.GENEN = 0;

  // Disable systick interrupt:  See
  // https://www.avrfreaks.net/forum/samd21-samd21e16b-sporadically-locks-and-does-not-wake-standby-sleep-mode
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;
  // set core voltage regulator to "runstandby" - erratta Main Voltage Regulator
  // Reference:15264
  // SUPC->VREG.bit.RUNSTDBY = 1;
  // SUPC->VREG.bit.STDBYPL0 = 1;

  /* CPU and BUS clocks slow down - slow down busses BEFORE cpu.. */
  MCLK->BUPDIV.reg =
      MCLK_BUPDIV_BUPDIV_DIV128; /** Divide Main clock ( 4MHz OSC ) by 64,ie run
                                    at 31.768kHz */
  MCLK->LPDIV.reg =
      MCLK_BUPDIV_BUPDIV_DIV128; /** Divide low power clock ( 4MHz OSC ) by 64,
                                    ie run at 31.768kHz*/
  MCLK->CPUDIV.reg = MCLK_CPUDIV_CPUDIV_DIV64; /**(MCLK_CPUDIV) Divide by 64 ,ie
                                                  run at 62.5kHz */

  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
  // sleeping here, will wake from here ( except from OFF or Backup modes, those
  // look like POR )

  /* CPU and BUS clocks back to "regular ratio"*/
  MCLK->CPUDIV.reg =
      MCLK_CPUDIV_CPUDIV_DIV1; /**(MCLK_CPUDIV) Divide by 1 ,ie run at 4MHz..
                                  until we start the DFLL again */
  MCLK->BUPDIV.reg =
      MCLK_BUPDIV_BUPDIV_DIV1; /** Div 1, so run these at main clock rate */
  MCLK->LPDIV.reg =
      MCLK_BUPDIV_BUPDIV_DIV1; /**low power domain back to CPU clock speed */

  // enable xosc32k clock
  OSC32KCTRL->XOSC32K.reg |= OSC32KCTRL_XOSC32K_ENABLE;
  // wait for clock to become ready
  while ((OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0)
    ;

  GCLK->GENCTRL[1].bit.GENEN = 1; // re-enable generator 1 ( xosc32k )
  while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(1)) {
    /* Wait for synchronization */
  };

  /* Enable DFLL peripheral channel */
  GCLK->PCHCTRL[0].reg |= GCLK_PCHCTRL_CHEN;

  while (GCLK->PCHCTRL[0].reg & GCLK_PCHCTRL_CHEN) {
    /* Wait for clock synchronization */
  }

  // re-enable DFLL
  dfllCtrlSlp.bit.ENABLE = 1;
  OSCCTRL->DFLLCTRL.reg = dfllCtrlSlp.reg;

  /* Clear performance level status */
  PM->INTFLAG.reg = PM_INTFLAG_PLRDY;
  /* Switch performance level to PL2 - Highest performance */
  PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2_Val;
  /* Waiting performance level ready */
  while (!PM->INTFLAG.reg) {
    ;
  }

  OSCCTRL->DFLLCTRL.reg = OSCCTRL_DFLLCTRL_ENABLE;

  gclkConfig.reg = 0;
  gclkConfig.reg = GCLK->GENCTRL[0].reg;
  gclkConfig.bit.SRC = GCLK_GENCTRL_SRC_DFLL48M_Val;
  GCLK->GENCTRL[0].reg = gclkConfig.reg;
  while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(0)) {
    /* Wait for synchronization */
  };
  //	GCLK->GENCTRL[0].reg |= GCLK_GENCTRL_GENEN;
  /*  Switch to PL2 to be sure configuration of GCLK0 is safe */
  // Enable systick interrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;

#ifndef USB_DISABLED
  if (restoreUSBDevice) {
    delay(1);
    USBDevice.init();
    USBDevice.attach();
  }
#endif
}

void ArduinoLowPowerClass::sleep(uint32_t millis) {
  setAlarmIn(millis);
  sleep();
}

void ArduinoLowPowerClass::deepSleep() { sleep(); }

void ArduinoLowPowerClass::deepSleep(uint32_t millis) { sleep(millis); }

void ArduinoLowPowerClass::setAlarmIn(uint32_t millis) {
  if (!rtc.isConfigured()) {
    attachInterruptWakeup(RTC_ALARM_WAKEUP, NULL, (irq_mode)0);
  }

  uint32_t now = rtc.getEpoch();
  rtc.setAlarmEpoch(now + millis / 1000);
  rtc.enableAlarm(rtc.MATCH_YYMMDDHHMMSS);
}

void ArduinoLowPowerClass::attachInterruptWakeup(uint32_t pin,
                                                 voidFuncPtr callback,
                                                 irq_mode mode) {
  if (pin > PINS_COUNT) {
    // check for external wakeup sources
    // RTC library should call this API to enable the alarm subsystem
    switch (pin) {
    case RTC_ALARM_WAKEUP:
      rtc.begin(false);
      rtc.attachInterrupt(callback);
      /*case UART_WAKEUP:*/
    }
    return;
  }

  uint8_t in = g_APinDescription[pin].ulExtInt;
  if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI)
    return;

  // pinMode(pin, INPUT_PULLUP);
  attachInterrupt(pin, callback, mode);
#if (SAMD21)
  configGCLK6();
#endif

#if (SAMD21)
  // Enable wakeup capability on pin in case being used during sleep
  EIC->WAKEUP.reg |= (1 << in);

#elif (SAML21 || SAMR34)
  // Enable wakeup capability on pin in case being used during sleep
  EIC->CTRLA.bit.CKSEL =
      1; // use ULP32k as source ( SAML21 is different to D21 series, EIC can be
         // set to use ULP32k without GCLK )
         // Enable EIC
  EIC->CTRLA.bit.ENABLE = 1;
  while (EIC->SYNCBUSY.bit.ENABLE == 1) { /*wait for sync*/
  }

  /* Errata: Make sure that the Flash does not power all the way down
   * when in sleep mode. */
  // NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
  NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_WAKEUPINSTANT_Val;
#endif
}

ArduinoLowPowerClass LowPower;

#endif
#endif // ARDUINO_ARCH_SAMD
