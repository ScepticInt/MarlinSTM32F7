#ifndef ZEROPI_H
#define ZEROPI_H

#include "arduino.h"

#define NUM_SLOTS 4
#define SLOT_NUM_PINS 6
#define NUM_EXTIO 13

#define SLOT_NULL 0x0
#define SLOT_STEPPER 0x1
#define SLOT_MOTOR 0x2

// stepper function define
#define STEP_EN 0x0
#define STEP_MS1 0x1
#define STEP_MS2 0x2
#define STEP_MS3 0x3
#define STEP_STP 0x4
#define STEP_DIR 0x5

#define  Bit_RESET 	0
#define  Bit_SET 	1

/// Read a pin
#define READ(IO) GPIO_ReadInputDataBit(zeroPiPinDescription[IO].ulPort, zeroPiPinDescription[IO].ulPin)
#define WRITE(IO, v)  GPIO_WriteBit(zeroPiPinDescription[IO].ulPort, zeroPiPinDescription[IO].ulPin, v) 

/// toggle a pin
#define TOGGLE(IO)  //do {DIO ##  IO ## _RPORT = MASK(DIO ## IO ## _PIN); } while (0)

/// set pin as input
#define	SET_INPUT(IO) GPIO_PinMode(zeroPiPinDescription[IO].ulPort, zeroPiPinDescription[IO].ulPin, INPUT );
/// set pin as output
#define	SET_OUTPUT(IO) GPIO_PinMode(zeroPiPinDescription[IO].ulPort, zeroPiPinDescription[IO].ulPin, OUTPUT );

/// check if pin is an input
#define	GET_INPUT(IO)  //((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) == 0)
/// check if pin is an output
#define	GET_OUTPUT(IO)  //((DIO ## IO ## _DDR & MASK(DIO ## IO ## _PIN)) != 0)

/// check if pin is an timer
#define	GET_TIMER(IO)  //((DIO ## IO ## _PWM)
#define SET_INPUT_PULLUP(IO)  GPIO_PinMode(zeroPiPinDescription[IO].ulPort, zeroPiPinDescription[IO].ulPin, INPUT_PULLUP );

#define strncpy_P(dest, src, num) strncpy((dest), (src), (num))

//#define TEMPERATURE_TIMER	TC5
//#define TEMPERATURE_TIMER_IRQ	TC5_IRQn
//#define TEMPERATURE_TIMER_ISR()	void TC5_Handler( void )

typedef enum _DRIVER {
	A4982		  = 0,
	A4988		  = 1,
	DRV8825		= 2,
	TB67S269	= 3,
	TB67S109	= 4
} DRIVER_t;

typedef struct _extIO {
  EPortType port;
  uint16_t pin;
} GPIO_t;

typedef struct _slot {
  int function;
  GPIO_t gpio[SLOT_NUM_PINS];
} SLOT_t;

//GPIO_t extio[];
extern const GPIO_t extio[];
extern const SLOT_t slot[];
extern const PinDescription zeroPiPinDescription[];
void GPIO_PinMode( EPortType PORTx, uint32_t Pin, uint32_t ulMode);
void GPIO_WriteBit(EPortType PORTx, uint16_t Pin, uint32_t BitVal);
uint8_t GPIO_ReadInputDataBit(EPortType PORTx, uint16_t Pin);

uint32_t analogReadChannel(int channel);
void slotSetup(int slot, int fun);
void stepperSetResolution(int s, int res, DRIVER_t type);
void mosfetInit(void);
void mosfetSet(int index, int value);
void tempInit(void);
void initTemperatureTimer(void);

#endif // ZEROPI_H
