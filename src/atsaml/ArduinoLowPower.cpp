#if defined(ARDUINO_ARCH_SAMD)
#include "ArduinoLowPower.h"

#if (SAML21)
#include "sam.h"

void ArduinoLowPowerClass::idle() {
  // SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  // PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_IDLE;
  // __DSB();
  // __WFI();
}

void ArduinoLowPowerClass::idle(uint32_t millis) {
  setAlarmIn(millis);
  idle();
}

void ArduinoLowPowerClass::sleep() {
  // before modifying the regulator settings, make sure the voltage is stable
  while (SUPC->INTFLAG.bit.VCORERDY != 1) {
  }
  SUPC->BOD12.bit.ENABLE = 0;
  SUPC->BOD33.bit.ENABLE = 0;

  // Set standby config
  PM->STDBYCFG.reg = PM_STDBYCFG_PDCFG(PM_STDBYCFG_PDCFG_DEFAULT_Val) |
                     (false << PM_STDBYCFG_DPGPD0_Pos) |
                     (false << PM_STDBYCFG_DPGPD1_Pos) |
                     PM_STDBYCFG_VREGSMOD(0) |
                     PM_STDBYCFG_LINKPD(PM_STDBYCFG_LINKPD_DEFAULT_Val) |
                     PM_STDBYCFG_BBIASHS(false) | PM_STDBYCFG_BBIASLP(false);

  // Prepare OSC16M to 4MHz mode, use for sleep
  OSCCTRL->OSC16MCTRL.bit.ENABLE = 1;
  OSCCTRL->OSC16MCTRL.bit.FSEL = OSCCTRL_OSC16MCTRL_FSEL_4_Val;
  OSCCTRL->OSC16MCTRL.bit.RUNSTDBY = 1;
  OSCCTRL->OSC16MCTRL.bit.ONDEMAND = 1;

  GCLK_GENCTRL_Type oldGCLKGenCtrl;
  oldGCLKGenCtrl.reg = 0;
  oldGCLKGenCtrl.reg = GCLK->GENCTRL[0].reg;

  // Switch to OSC16M
  GCLK_GENCTRL_Type gclkConfig;
  gclkConfig.reg = 0;
  gclkConfig.reg = GCLK->GENCTRL[0].reg;
  gclkConfig.bit.SRC = GCLK_GENCTRL_SRC_OSC16M_Val;

  GCLK->GENCTRL[0].reg = gclkConfig.reg;

  // Wait for the clock to switch
  while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_MASK)
    ;

  PM->INTFLAG.reg = PM_INTFLAG_PLRDY;
  PM->PLCFG.reg = PM_PLCFG_PLSEL_PL0_Val;
  while (!PM->INTFLAG.reg) {
    ;
  }
  // disable xosc32k clock
  OSC32KCTRL->XOSC32K.bit.ONDEMAND = 0;
  OSC32KCTRL->XOSC32K.reg &= ~OSC32KCTRL_XOSC32K_ENABLE;
  while ((OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0)
    ;

  // Disable systick interrupt:  See
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;

  SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
  PM->SLEEPCFG.reg = PM_SLEEPCFG_SLEEPMODE_STANDBY;
  __DSB();
  __WFI();

  // enable xosc32k clock
  OSC32KCTRL->XOSC32K.reg |= OSC32KCTRL_XOSC32K_ENABLE;

  // wait for clock to become ready
  while ((OSC32KCTRL->STATUS.reg & OSC32KCTRL_STATUS_XOSC32KRDY) == 0)
    ;

  // Restore GCLK0 to previous state
  GCLK->GENCTRL[0].reg = oldGCLKGenCtrl.reg;
  while (GCLK->SYNCBUSY.reg & GCLK_SYNCBUSY_GENCTRL(0)) {
  };

  /* Clear performance level status */
  PM->INTFLAG.reg = PM_INTFLAG_PLRDY;
  /* Switch performance level to PL2 - Highest performance */
  PM->PLCFG.reg = PM_PLCFG_PLSEL_PL2_Val;
  /* Waiting performance level ready */
  while (!PM->INTFLAG.reg) {
    ;
  }

  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
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

  // EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
  // if (in == NOT_AN_INTERRUPT || in == EXTERNAL_INT_NMI)
  // 		return;

  // //pinMode(pin, INPUT_PULLUP);
  // attachInterrupt(pin, callback, mode);

  // configGCLK6();

  // // Enable wakeup capability on pin in case being used during sleep
  // EIC->WAKEUP.reg |= (1 << in);
}

void ArduinoLowPowerClass::attachAdcInterrupt(uint32_t pin,
                                              voidFuncPtr callback,
                                              adc_interrupt mode, uint16_t lo,
                                              uint16_t hi) {
  // uint8_t winmode = 0;

  // switch (mode) {
  // 	case ADC_INT_BETWEEN:   winmode = ADC_WINCTRL_WINMODE_MODE3; break;
  // 	case ADC_INT_OUTSIDE:   winmode = ADC_WINCTRL_WINMODE_MODE4; break;
  // 	case ADC_INT_ABOVE_MIN: winmode = ADC_WINCTRL_WINMODE_MODE1; break;
  // 	case ADC_INT_BELOW_MAX: winmode = ADC_WINCTRL_WINMODE_MODE2; break;
  // 	default: return;
  // }

  // adc_cb = callback;

  // configGCLK6();

  // // Configure ADC to use GCLK6 (OSCULP32K)
  // while (GCLK->STATUS.bit.SYNCBUSY) {}
  // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC
  // 					| GCLK_CLKCTRL_GEN_GCLK6
  // 					| GCLK_CLKCTRL_CLKEN;
  // while (GCLK->STATUS.bit.SYNCBUSY) {}

  // // Set ADC prescaler as low as possible
  // ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV4;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Configure window mode
  // ADC->WINLT.reg = lo;
  // ADC->WINUT.reg = hi;
  // ADC->WINCTRL.reg = winmode;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Enable window interrupt
  // ADC->INTENSET.bit.WINMON = 1;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Enable ADC in standby mode
  // ADC->CTRLA.bit.RUNSTDBY = 1;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Enable continuous conversions
  // ADC->CTRLB.bit.FREERUN = 1;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Configure input mux
  // ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[pin].ulADCChannelNumber;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Enable the ADC
  // ADC->CTRLA.bit.ENABLE = 1;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Start continuous conversions
  // ADC->SWTRIG.bit.START = 1;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Enable the ADC interrupt
  // NVIC_EnableIRQ(ADC_IRQn);
}

void ArduinoLowPowerClass::detachAdcInterrupt() {
  // // Disable the ADC interrupt
  // NVIC_DisableIRQ(ADC_IRQn);

  // // Disable the ADC
  // ADC->CTRLA.bit.ENABLE = 0;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Disable continuous conversions
  // ADC->CTRLB.bit.FREERUN = 0;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Disable ADC in standby mode
  // ADC->CTRLA.bit.RUNSTDBY = 1;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Disable window interrupt
  // ADC->INTENCLR.bit.WINMON = 1;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Disable window mode
  // ADC->WINCTRL.reg = ADC_WINCTRL_WINMODE_DISABLE;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Restore ADC prescaler
  // ADC->CTRLB.bit.PRESCALER = ADC_CTRLB_PRESCALER_DIV512_Val;
  // while (ADC->STATUS.bit.SYNCBUSY) {}

  // // Restore ADC clock
  // while (GCLK->STATUS.bit.SYNCBUSY) {}
  // GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_ADC
  // 					| GCLK_CLKCTRL_GEN_GCLK0
  // 					| GCLK_CLKCTRL_CLKEN;
  // while (GCLK->STATUS.bit.SYNCBUSY) {}

  // adc_cb = nullptr;
}

void ADC_Handler() {
  // // Clear the interrupt flag
  // ADC->INTFLAG.bit.WINMON = 1;
  // LowPower.adc_cb();
}

ArduinoLowPowerClass LowPower;

#endif
#endif // ARDUINO_ARCH_SAMD
