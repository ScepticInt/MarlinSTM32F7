#ifdef ARDUINO_ARCH_SAMD
#include "ZeroPi.h"
#include "WVariant.h"

const GPIO_t extio[NUM_EXTIO] = {
  {PORTA, 2},
  {PORTB, 8},
  {PORTB, 9},
  {PORTA, 4},
  {PORTB, 10},
  {PORTA, 12},
  {PORTB, 11},
  {PORTA, 22},
  {PORTA, 23},
  {PORTA, 11},
  {PORTA, 10},
  {PORTA, 31},
  {PORTA, 30}
};

const SLOT_t slot[NUM_SLOTS] = {
  // slot1
  { SLOT_NULL,
    {{PORTB, 4}, {PORTB, 5}, {PORTB, 6}, {PORTB, 7}, {PORTA, 8}, {PORTA, 9}}
  },
  // slot2
  { SLOT_NULL,
    {{PORTB, 17}, {PORTB, 16}, {PORTB, 1}, {PORTB, 0}, {PORTA, 20}, {PORTA, 15}}/*{PORTA, 21}} old*/
  },
  // slot3
  { SLOT_NULL,
    {{PORTB, 12}, {PORTB, 13}, {PORTB, 14}, {PORTB, 15}, {PORTA, 19}, {PORTA, 18}}
  },
  // slot4
  { SLOT_NULL,
    {{PORTA, 16}, {PORTA, 21}/*{PORTA, 15}} old*/, {PORTA, 14}, {PORTA, 13}, {PORTA, 6}, {PORTA, 7}}
  }
};

void GPIO_PinMode(EPortType PORTx, uint32_t Pin, uint32_t ulMode) {
  switch (ulMode) {
    case INPUT:
      // Set pin to input mode
      PORT->Group[PORTx].PINCFG[Pin].reg = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_PULLEN);
      PORT->Group[PORTx].DIRCLR.reg = (uint32_t)(1 << Pin);
      break;

    case INPUT_PULLUP:
      // Set pin to input mode with pull-up resistor enabled
      PORT->Group[PORTx].PINCFG[Pin].reg = (uint8_t)(PORT_PINCFG_INEN | PORT_PINCFG_PULLEN);
      PORT->Group[PORTx].DIRCLR.reg = (uint32_t)(1 << Pin);
      // Enable pull level (cf '22.6.3.2 Input Configuration' and '22.8.7 Data Output Value Set')
      PORT->Group[PORTx].OUTSET.reg = (uint32_t)(1<<Pin);
      break;

    case OUTPUT:
      PORT->Group[PORTx].PINCFG[Pin].reg |= PORT_PINCFG_DRVSTR;//Output Driver Strength Selection
      PORT->Group[PORTx].DIRSET.reg = (uint32_t)(1 << Pin);
      break;

    default:
      // do nothing
      break;
  }
}

void GPIO_WriteBit(EPortType PORTx, uint16_t Pin, uint32_t BitVal) {
  switch (BitVal) {
    case LOW:
      PORT->Group[PORTx].OUTCLR.reg = (1ul << Pin);
      break;
    case HIGH:
      PORT->Group[PORTx].OUTSET.reg = (1ul << Pin);
      break;
    default:
      break;
  }
}

uint8_t GPIO_ReadInputDataBit(EPortType PORTx, uint16_t Pin) {
  return ((PORT->Group[PORTx].IN.reg & (1ul << Pin))) ? HIGH : LOW;
}

void stepperEnable(int s, int en) {
  GPIO_WriteBit(slot[s].gpio[STEP_EN].port, slot[s].gpio[STEP_EN].pin, en ? Bit_RESET : Bit_SET);
}

// stepper related functions
void stepperInit(int s) {
  for (int n = 0; n < SLOT_NUM_PINS; n++)
    GPIO_PinMode(slot[s].gpio[n].port, slot[s].gpio[n].pin, OUTPUT);
  stepperEnable(s,0);
}

void slotSetup(int slot, int fun) {
  if (fun == SLOT_STEPPER) stepperInit(slot);
}

void stepperSetResolution(int s, int res, DRIVER_t type) {
  switch (type) {
    case A4982: // only MS1 MS2
      GPIO_PinMode(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, OUTPUT);
      GPIO_PinMode(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, OUTPUT);
      // GPIO_PinMode(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, OUTPUT);
      switch (res) {
        case 1:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          // GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 2:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          // GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 4:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          // GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        // case 8:
        //   GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
        //   GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
        //   GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
        //   break;
        case 16:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          // GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_SET);
          break;
        default:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          // GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
      }
      break;
    case A4988:
      GPIO_PinMode(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, OUTPUT);
      GPIO_PinMode(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, OUTPUT);
      GPIO_PinMode(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, OUTPUT);
      switch (res) {
        case 1:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 2:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 4:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 8:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 16:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_SET);
          break;
        default:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
      }
      break;
    case DRV8825:
      GPIO_PinMode(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, OUTPUT);
      GPIO_PinMode(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, OUTPUT);
      GPIO_PinMode(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, OUTPUT);
      switch (res) {
        case 1:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 2:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 4:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 8:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 16:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_SET);
          break;
        case 32:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_SET);
          break;
        default:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
      }
      break;
    case TB67S269:
    case TB67S109:
      GPIO_PinMode(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, OUTPUT);
      GPIO_PinMode(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, OUTPUT);
      GPIO_PinMode(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, OUTPUT);
      switch (res) {
        case 1:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_SET);
          break;
        case 2://Half step resolution(Type (A))
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 4:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_SET);
          break;
        case 8:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_RESET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_SET);
          break;
        case 16:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_RESET);
          break;
        case 32:
          GPIO_WriteBit(slot[s].gpio[STEP_MS1].port, slot[s].gpio[STEP_MS1].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS2].port, slot[s].gpio[STEP_MS2].pin, Bit_SET);
          GPIO_WriteBit(slot[s].gpio[STEP_MS3].port, slot[s].gpio[STEP_MS3].pin, Bit_SET);
          break;
        default:

          break;
      }
      break;

    default: break;
  }
}

//T0  PA05
//T1  PB02
// temperature sensor
void tempInit(void) {
  //  int portT0 = PORTA;
  //  int pinT0 = 5;
  //  int portT1 = PORTB;
  //  int pinT1 = 2;

  uint32_t temp;

  // Get whole current setup for both odd and even pins and remove odd one
  temp = (PORT->Group[PORTA].PMUX[5 >> 1].reg) & PORT_PMUX_PMUXE(0xF);
  // Set new muxing
  PORT->Group[PORTA].PMUX[5 >> 1].reg = temp|PORT_PMUX_PMUXO(PORT_PMUX_PMUXO_B_Val);
  // Enable port mux
  PORT->Group[PORTA].PINCFG[5].reg |= PORT_PINCFG_PMUXEN;

  temp = (PORT->Group[PORTB].PMUX[2 >> 1].reg) & PORT_PMUX_PMUXO(0xF);
  PORT->Group[PORTB].PMUX[2 >> 1].reg = temp|PORT_PMUX_PMUXE(PORT_PMUX_PMUXE_B_Val);
  PORT->Group[PORTB].PINCFG[2].reg |= PORT_PINCFG_PMUXEN; // Enable port mux

  ADC->CTRLA.bit.ENABLE = 0x01;             // Enable ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1) { /* nada */ }
  ADC->CTRLB.bit.RESSEL = ADC_CTRLB_RESSEL_10BIT_Val;//10bit ADC
  while (ADC->STATUS.bit.SYNCBUSY == 1) { /* nada */ }
  ADC->CTRLA.bit.ENABLE = 0x01;
  while (ADC->STATUS.bit.SYNCBUSY == 1) { /* nada */ }

}

uint32_t analogReadChannel(int channel) {
  uint32_t valueRead = 0;

  ADC->INPUTCTRL.bit.MUXPOS = channel;
  while (ADC->STATUS.bit.SYNCBUSY == 1) { /* nada */ }
  // Start conversion
  ADC->SWTRIG.bit.START = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1) { /* nada */ }
  // Clear the Data Ready flag
  ADC->INTFLAG.bit.RESRDY = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1) { /* nada */ }
  ADC->SWTRIG.bit.START = 1;
  while (ADC->STATUS.bit.SYNCBUSY == 1) { /* nada */ }
  // Store the value
  while (ADC->INTFLAG.bit.RESRDY == 0);   // Waiting for conversion to complete
  valueRead = ADC->RESULT.reg;
}


//void initTemperatureTimer(void) {
//  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK3 | GCLK_CLKCTRL_ID(GCM_TC4_TC5));
//
//  TEMPERATURE_TIMER->COUNT16.CTRLA.reg &= ~(TC_CTRLA_ENABLE);       //disable TC module
//  TEMPERATURE_TIMER->COUNT16.CTRLA.reg |=TC_CTRLA_MODE_COUNT16;
//  TEMPERATURE_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
//  TEMPERATURE_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV16;//500kHz
//  TEMPERATURE_TIMER->COUNT16.CC[0].reg = 500;           //1000Hz
//  TEMPERATURE_TIMER->COUNT16.INTENSET.reg = TC_INTFLAG_MC0;
//  TEMPERATURE_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
//  TEMPERATURE_TIMER->COUNT16.INTFLAG.reg = 0xFF;
//  NVIC_EnableIRQ(TEMPERATURE_TIMER_IRQ);
//}


// mosfet control
//void mosfetInit(void) {
//  uint32_t temp;
//  GPIO_PinMode(PORTB, 30, OUTPUT);
//  GPIO_PinMode(PORTB, 31, OUTPUT);

//
////PB31
//    // Get whole current setup for both odd and even pins and remove odd one
//    temp = (PORT->Group[PORTB].PMUX[31 >> 1].reg) & PORT_PMUX_PMUXE(0xF);
//    // Set new muxing
//    PORT->Group[PORTB].PMUX[31 >> 1].reg = temp | PORT_PMUX_PMUXO(PORT_PMUX_PMUXO_E_Val);
//    // Enable port mux
//    PORT->Group[PORTB].PINCFG[31].reg |= PORT_PINCFG_PMUXEN;

////PB30
//    temp = (PORT->Group[PORTB].PMUX[30 >> 1].reg) & PORT_PMUX_PMUXO(0xF);
//    PORT->Group[PORTB].PMUX[30 >> 1].reg = temp | PORT_PMUX_PMUXE(PORT_PMUX_PMUXE_E_Val);
//    PORT->Group[PORTB].PINCFG[30].reg |= PORT_PINCFG_PMUXEN; // Enable port mux
//
//
//  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC0_TCC1));
//  TCC0->CTRLA.reg &= ~(TCC_CTRLA_ENABLE);       //disable TCC module
//  TCC0->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV256;  //setting prescaler to divide by 256
//  TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;      //
//  TCC0->CC[0].reg = 0;                //
//  TCC0->CC[1].reg = 0;                //
//  TCC0->PER.reg = 254;
//  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE;       //ENABLE
//}

//void mosfetSet(int index, int value) {
//  TCC0->CC[index].reg = value;
//}

/*
 * Pins descriptions
 */
const PinDescription zeroPiPinDescription[] = {
  // 0..13 - Digital pins
  // ----------------------
  // 0/1 - SERCOM/UART (Serial1)
  { PORTA, 11, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // RX: SERCOM0/PAD[3]
  { PORTA, 10, PIO_SERCOM, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // TX: SERCOM0/PAD[2]

  // 2..12
  // Digital Low
  { PORTA, 14, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 },
  { PORTA,  9, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_9 }, // TCC0/WO[1]
  { PORTA,  8, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_NMI },  // TCC0/WO[0]
  { PORTA, 15, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH1, TC3_CH1, EXTERNAL_INT_15 }, // TC3/WO[1]
  { PORTA, 20, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH6, TCC0_CH6, EXTERNAL_INT_4 }, // TCC0/WO[6]
  { PORTA, 21, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 },

  // Digital High
  { PORTA,  6, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH0, TCC1_CH0, EXTERNAL_INT_4 }, // TCC1/WO[0]
  { PORTA,  7, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM1_CH1, TCC1_CH1, EXTERNAL_INT_5 }, // TCC1/WO[1]
  { PORTA, 18, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM3_CH0, TC3_CH0, EXTERNAL_INT_2 }, // TC3/WO[0]
  { PORTA, 16, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM2_CH0, TCC2_CH0, EXTERNAL_INT_0 }, // TCC2/WO[0]
  { PORTA, 19, PIO_TIMER_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH3, TCC0_CH3, EXTERNAL_INT_3 }, // TCC0/WO[3]

  // 13 (LED)
  { PORTA, 17, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM2_CH1, NOT_ON_TIMER, EXTERNAL_INT_1 }, // TCC2/WO[1]

  // 14..19 - Analog pins
  // --------------------
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[0]
  { PORTB,  8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel2, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8 }, // ADC/AIN[2]
  { PORTB,  9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel3, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9 }, // ADC/AIN[3]
  { PORTA,  4, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, // ADC/AIN[4]
  { PORTA,  5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, // ADC/AIN[5]
  { PORTB,  2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // ADC/AIN[10]

  // 20..21 I2C pins (SDA/SCL and also EDBG:SDA/SCL)
  // ----------------------
  { PORTA, 22, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, // SCL: SERCOM3/PAD[1]

  // 22..24 - SPI pins (ICSP:MISO,SCK,MOSI)
  // ----------------------
  { PORTA, 12, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, // MISO: SERCOM4/PAD[0]
  { PORTB, 10, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10 }, // MOSI: SERCOM4/PAD[2]
  { PORTB, 11, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11 }, // SCK: SERCOM4/PAD[3]

  // 25..26 - RX/TX LEDS (PB03/PA27)
  // --------------------
  { PORTB,  3, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only
  { PORTA, 27, PIO_OUTPUT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // used as output only

  // 27..29 - USB
  // --------------------
  { PORTA, 28, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB Host enable
  { PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DM
  { PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // USB/DP

  // 30..41 - EDBG
  // ----------------------
  // 30/31 - EDBG/UART
  { PORTB, 22, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // TX: SERCOM5/PAD[2]
  { PORTB, 23, PIO_SERCOM_ALT, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // RX: SERCOM5/PAD[3]

  // 32/33 I2C (SDA/SCL and also EDBG:SDA/SCL)
  { PORTA, 22, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SDA: SERCOM3/PAD[0]
  { PORTA, 23, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCL: SERCOM3/PAD[1]

  // 34..37 - EDBG/SPI
  { PORTA, 19, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MISO: SERCOM1/PAD[3]
  { PORTA, 16, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // MOSI: SERCOM1/PAD[0]
  { PORTA, 18, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SS: SERCOM1/PAD[2]
  { PORTA, 17, PIO_SERCOM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SCK: SERCOM1/PAD[1]

  // 38..41 - EDBG/Digital
  { PORTA, 13, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH5, NOT_ON_TIMER, EXTERNAL_INT_13 }, // EIC/EXTINT[13] *TCC2/WO[1] TCC0/WO[7]
  { PORTA, 21, PIO_PWM_ALT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM0_CH7, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 7
  { PORTA,  6, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH0, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 8
  { PORTA,  7, PIO_PWM, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), No_ADC_Channel, PWM1_CH1, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // Pin 9

  // 42 (AREF)
  { PORTA,  3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // DAC/VREFP

  // ----------------------
  // 43 - Alternate use of A0 (DAC output)
  { PORTA,  2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2 }, // DAC/VOUT

  //44..47
  { PORTB,  4, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4 }, //
  { PORTB,  5, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5 }, //
  { PORTB,  6, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6 }, //
  { PORTB,  7, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7 }, //
   //48..51
  { PORTB, 17, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, //
  { PORTB, 16, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, //
  { PORTB,  1, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_0 }, //
  { PORTB,  0, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_1 }, //
  //52..55
  { PORTB, 12, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_12 }, //
  { PORTB, 13, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_13 }, //
  { PORTB, 14, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_14 }, //
  { PORTB, 15, PIO_TIMER, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_15 }, //
  //56..57
  { PORTB, 30, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH0, TCC0_CH0, EXTERNAL_INT_14 }, // TCC0/WO[0]
  { PORTB, 31, PIO_TIMER, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM|PIN_ATTR_TIMER), No_ADC_Channel, PWM0_CH1, TCC0_CH1, EXTERNAL_INT_15 }, // TCC0/WO[1]
  //58..59
  { PORTA, 30, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SWCLK
  { PORTA, 31, PIO_DIGITAL, (PIN_ATTR_DIGITAL), No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NONE }, // SWDIO
};

#endif // ARDUINO_ARCH_SAMD
