/*
  stepper.h - stepper motor driver: executes motion plans of planner.c using the stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef stepper_h
#define stepper_h

#include "planner.h"
#include "stepper_indirection.h"

#ifdef ARDUINO_ARCH_SAMD

  #define STEPPER_TIMER                       TC5
  #define STEPPER_TIMER_ID                    GCM_TC4_TC5
  #define STEPPER_TIMER_IRQ                   TC5_IRQn
  #define ENABLE_STEPPER_DRIVER_INTERRUPT()   NVIC_EnableIRQ(TC5_IRQn)
  #define DISABLE_STEPPER_DRIVER_INTERRUPT()  NVIC_DisableIRQ(TC5_IRQn)
  #define STEPPER_TIMER_ISR()                 void TC5_Handler()
  #define STEPPER_TIMER_OCR                   TC5->COUNT16.CC[0].reg
  #define STEPPER_TIMER_CNT                   TC5->COUNT16.COUNT.reg
  #define	STEPPER_TIMER_CLK                   2000000UL

  #define ADVANCE_STEPPER_TIMER                       TC4
  #define ADVANCE_STEPPER_TIMER_ID                    GCM_TC4_TC5
  #define ADVANCE_STEPPER_TIMER_IRQ                   TC4_IRQn
  #define ENABLE_ADVANCE_STEPPER_DRIVER_INTERRUPT()   NVIC_EnableIRQ(TC4_IRQn)
  #define DISABLE_ADVANCE_STEPPER_DRIVER_INTERRUPT()  NVIC_DisableIRQ(TC4_IRQn)
  #define ADVANCE_STEPPER_TIMER_ISR()                 void TC4_Handler()
  #define ADVANCE_STEPPER_TIMER_OCR                   TC4->COUNT16.CC[0].reg
  #define	ADVANCE_STEPPER_TIMER_CLK                   10000UL //10kHz

#else // AVR

  #define ENABLE_STEPPER_DRIVER_INTERRUPT()   TIMSK1 |= (1<<OCIE1A)
  #define DISABLE_STEPPER_DRIVER_INTERRUPT()  TIMSK1 &= ~(1<<OCIE1A)
  #define STEPPER_TIMER_ISR()                 ISR(TIMER1_COMPA_vect)
  #define STEPPER_TIMER_OCR                   OCR1A //Output Compare Register
  #define STEPPER_TIMER_CNT                   TCNT1

#endif // SAMD or AVR

#if EXTRUDERS > 3
  #define E_STEP_WRITE(v) { if(current_block->active_extruder == 3) { E3_STEP_WRITE(v); } else { if(current_block->active_extruder == 2) { E2_STEP_WRITE(v); } else { if(current_block->active_extruder == 1) { E1_STEP_WRITE(v); } else { E0_STEP_WRITE(v); }}}}
  #define NORM_E_DIR() { if(current_block->active_extruder == 3) { E3_DIR_WRITE( !INVERT_E3_DIR); } else { if(current_block->active_extruder == 2) { E2_DIR_WRITE(!INVERT_E2_DIR); } else { if(current_block->active_extruder == 1) { E1_DIR_WRITE(!INVERT_E1_DIR); } else { E0_DIR_WRITE(!INVERT_E0_DIR); }}}}
  #define REV_E_DIR() { if(current_block->active_extruder == 3) { E3_DIR_WRITE(INVERT_E3_DIR); } else { if(current_block->active_extruder == 2) { E2_DIR_WRITE(INVERT_E2_DIR); } else { if(current_block->active_extruder == 1) { E1_DIR_WRITE(INVERT_E1_DIR); } else { E0_DIR_WRITE(INVERT_E0_DIR); }}}}
#elif EXTRUDERS > 2
  #define E_STEP_WRITE(v) { if(current_block->active_extruder == 2) { E2_STEP_WRITE(v); } else { if(current_block->active_extruder == 1) { E1_STEP_WRITE(v); } else { E0_STEP_WRITE(v); }}}
  #define NORM_E_DIR() { if(current_block->active_extruder == 2) { E2_DIR_WRITE(!INVERT_E2_DIR); } else { if(current_block->active_extruder == 1) { E1_DIR_WRITE(!INVERT_E1_DIR); } else { E0_DIR_WRITE(!INVERT_E0_DIR); }}}
  #define REV_E_DIR() { if(current_block->active_extruder == 2) { E2_DIR_WRITE(INVERT_E2_DIR); } else { if(current_block->active_extruder == 1) { E1_DIR_WRITE(INVERT_E1_DIR); } else { E0_DIR_WRITE(INVERT_E0_DIR); }}}
#elif EXTRUDERS > 1
  #if DISABLED(DUAL_X_CARRIAGE)
    #define E_STEP_WRITE(v) { if(current_block->active_extruder == 1) { E1_STEP_WRITE(v); } else { E0_STEP_WRITE(v); }}
    #define NORM_E_DIR() { if(current_block->active_extruder == 1) { E1_DIR_WRITE(!INVERT_E1_DIR); } else { E0_DIR_WRITE(!INVERT_E0_DIR); }}
    #define REV_E_DIR() { if(current_block->active_extruder == 1) { E1_DIR_WRITE(INVERT_E1_DIR); } else { E0_DIR_WRITE(INVERT_E0_DIR); }}
  #else
    extern bool extruder_duplication_enabled;
    #define E_STEP_WRITE(v) { if(extruder_duplication_enabled) { E0_STEP_WRITE(v); E1_STEP_WRITE(v); } else if(current_block->active_extruder == 1) { E1_STEP_WRITE(v); } else { E0_STEP_WRITE(v); }}
    #define NORM_E_DIR() { if(extruder_duplication_enabled) { E0_DIR_WRITE(!INVERT_E0_DIR); E1_DIR_WRITE(!INVERT_E1_DIR); } else if(current_block->active_extruder == 1) { E1_DIR_WRITE(!INVERT_E1_DIR); } else { E0_DIR_WRITE(!INVERT_E0_DIR); }}
    #define REV_E_DIR() { if(extruder_duplication_enabled) { E0_DIR_WRITE(INVERT_E0_DIR); E1_DIR_WRITE(INVERT_E1_DIR); } else if(current_block->active_extruder == 1) { E1_DIR_WRITE(INVERT_E1_DIR); } else { E0_DIR_WRITE(INVERT_E0_DIR); }}
  #endif
#else
  #define E_STEP_WRITE(v) E0_STEP_WRITE(v)
  #define NORM_E_DIR() E0_DIR_WRITE(!INVERT_E0_DIR)
  #define REV_E_DIR() E0_DIR_WRITE(INVERT_E0_DIR)
#endif

#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  extern bool abort_on_endstop_hit;
#endif

// Initialize and start the stepper motor subsystem
void st_init();

// Block until all buffered steps are executed
void st_synchronize();

// Set current position in steps
void st_set_position(const long& x, const long& y, const long& z, const long& e);
void st_set_e_position(const long& e);

// Get current position in steps
long st_get_position(uint8_t axis);

// Get current position in mm
float st_get_position_mm(AxisEnum axis);

// The stepper subsystem goes to sleep when it runs out of things to execute. Call this
// to notify the subsystem that it is time to go to work.
void st_wake_up();


void checkHitEndstops(); //call from somewhere to create an serial error message with the locations the endstops where hit, in case they were triggered
void endstops_hit_on_purpose(); //avoid creation of the message, i.e. after homing and before a routine call of checkHitEndstops();

void enable_endstops(bool check); // Enable/disable endstop checking

void checkStepperErrors(); //Print errors detected by the stepper

void finishAndDisableSteppers();

extern block_t* current_block;  // A pointer to the block currently being traced

void quickStop();

void digitalPotWrite(int address, int value);
void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2);
void microstep_mode(uint8_t driver, uint8_t stepping);
void digipot_init();
void digipot_current(uint8_t driver, int current);
void microstep_init();
void microstep_readings();

#if ENABLED(Z_DUAL_ENDSTOPS)
  void In_Homing_Process(bool state);
  void Lock_z_motor(bool state);
  void Lock_z2_motor(bool state);
#endif

#if ENABLED(BABYSTEPPING)
  void babystep(const uint8_t axis, const bool direction); // perform a short step with a single stepper motor, outside of any convention
#endif

#endif
