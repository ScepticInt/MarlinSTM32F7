/**
 * Marlin 3D Printer Firmware
 * Copyright (C) 2016 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * M600: Pause for filament change
 *
 *  E[distance] - Retract the filament this far (negative value)
 *  Z[distance] - Move the Z axis by this distance
 *  X[position] - Move to this X position, with Y
 *  Y[position] - Move to this Y position, with X
 *  L[distance] - Retract distance for removal (manual reload)
 *
 *  Default values are used for omitted arguments.
 *
 */
inline void gcode_M600() {

  if (thermalManager.tooColdToExtrude(active_extruder)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_TOO_COLD_FOR_M600);
    return;
  }

  // Show initial message and wait for synchronize steppers
  lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_INIT);
  stepper.synchronize();

  float lastpos[NUM_AXIS];

  // Save current position of all axes
  LOOP_XYZE(i)
    lastpos[i] = destination[i] = current_position[i];

  // Define runplan for move axes
  #if ENABLED(DELTA)
    #define RUNPLAN(RATE_MM_S) inverse_kinematics(destination); \
                               planner.buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], RATE_MM_S, active_extruder);
  #else
    #define RUNPLAN(RATE_MM_S) line_to_destination(MMS_TO_MMM(RATE_MM_S));
  #endif

  KEEPALIVE_STATE(IN_HANDLER);

  // Initial retract before move to filament change position
  if (code_seen('E')) destination[E_AXIS] += code_value_axis_units(E_AXIS);
  #if defined(FILAMENT_CHANGE_RETRACT_LENGTH) && FILAMENT_CHANGE_RETRACT_LENGTH > 0
    else destination[E_AXIS] -= FILAMENT_CHANGE_RETRACT_LENGTH;
  #endif

  RUNPLAN(FILAMENT_CHANGE_RETRACT_FEEDRATE);

  // Lift Z axis
  float z_lift = code_seen('Z') ? code_value_axis_units(Z_AXIS) :
    #if defined(FILAMENT_CHANGE_Z_ADD) && FILAMENT_CHANGE_Z_ADD > 0
      FILAMENT_CHANGE_Z_ADD
    #else
      0
    #endif
  ;

  if (z_lift > 0) {
    destination[Z_AXIS] += z_lift;
    NOMORE(destination[Z_AXIS], Z_MAX_POS);
    RUNPLAN(FILAMENT_CHANGE_Z_FEEDRATE);
  }

  // Move XY axes to filament exchange position
  if (code_seen('X')) destination[X_AXIS] = code_value_axis_units(X_AXIS);
  #ifdef FILAMENT_CHANGE_X_POS
    else destination[X_AXIS] = FILAMENT_CHANGE_X_POS;
  #endif

  if (code_seen('Y')) destination[Y_AXIS] = code_value_axis_units(Y_AXIS);
  #ifdef FILAMENT_CHANGE_Y_POS
    else destination[Y_AXIS] = FILAMENT_CHANGE_Y_POS;
  #endif

  RUNPLAN(FILAMENT_CHANGE_XY_FEEDRATE);

  stepper.synchronize();
  lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_UNLOAD);

  // Unload filament
  if (code_seen('L')) destination[E_AXIS] += code_value_axis_units(E_AXIS);
  #if defined(FILAMENT_CHANGE_UNLOAD_LENGTH) && FILAMENT_CHANGE_UNLOAD_LENGTH > 0
    else destination[E_AXIS] -= FILAMENT_CHANGE_UNLOAD_LENGTH;
  #endif

  RUNPLAN(FILAMENT_CHANGE_UNLOAD_FEEDRATE);

  // Synchronize steppers and then disable extruders steppers for manual filament changing
  stepper.synchronize();
  disable_e0();
  disable_e1();
  disable_e2();
  disable_e3();
  delay(100);

  #if HAS_BUZZER
    millis_t next_tick = 0;
  #endif

  // Wait for filament insert by user and press button
  lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_INSERT);

  while (!lcd_clicked()) {
    #if HAS_BUZZER
      millis_t ms = millis();
      if (ms >= next_tick) {
        buzzer.tone(300, 2000);
        next_tick = ms + 2500; // Beep every 2.5s while waiting
      }
    #endif
    idle(true);
  }
  delay(100);
  while (lcd_clicked()) idle(true);
  delay(100);

  // Show load message
  lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_LOAD);

  // Load filament
  if (code_seen('L')) destination[E_AXIS] -= code_value_axis_units(E_AXIS);
  #if defined(FILAMENT_CHANGE_LOAD_LENGTH) && FILAMENT_CHANGE_LOAD_LENGTH > 0
    else destination[E_AXIS] += FILAMENT_CHANGE_LOAD_LENGTH;
  #endif

  RUNPLAN(FILAMENT_CHANGE_LOAD_FEEDRATE);
  stepper.synchronize();

  #if defined(FILAMENT_CHANGE_EXTRUDE_LENGTH) && FILAMENT_CHANGE_EXTRUDE_LENGTH > 0
    do {
      // Extrude filament to get into hotend
      lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_EXTRUDE);
      destination[E_AXIS] += FILAMENT_CHANGE_EXTRUDE_LENGTH;
      RUNPLAN(FILAMENT_CHANGE_EXTRUDE_FEEDRATE);
      stepper.synchronize();
      // Ask user if more filament should be extruded
      KEEPALIVE_STATE(PAUSED_FOR_USER);
      lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_OPTION);
      while (filament_change_menu_response == FILAMENT_CHANGE_RESPONSE_WAIT_FOR) idle(true);
      KEEPALIVE_STATE(IN_HANDLER);
    } while (filament_change_menu_response != FILAMENT_CHANGE_RESPONSE_RESUME_PRINT);
  #endif

  lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_RESUME);

  KEEPALIVE_STATE(IN_HANDLER);

  // Set extruder to saved position
  current_position[E_AXIS] = lastpos[E_AXIS];
  destination[E_AXIS] = lastpos[E_AXIS];
  planner.set_e_position_mm(current_position[E_AXIS]);

  #if ENABLED(DELTA)
    // Move XYZ to starting position, then E
    inverse_kinematics(lastpos);
    planner.buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], destination[E_AXIS], FILAMENT_CHANGE_XY_FEEDRATE, active_extruder);
    planner.buffer_line(delta[X_AXIS], delta[Y_AXIS], delta[Z_AXIS], lastpos[E_AXIS], FILAMENT_CHANGE_XY_FEEDRATE, active_extruder);
  #else
    // Move XY to starting position, then Z, then E
    destination[X_AXIS] = lastpos[X_AXIS];
    destination[Y_AXIS] = lastpos[Y_AXIS];
    RUNPLAN(FILAMENT_CHANGE_XY_FEEDRATE);
    destination[Z_AXIS] = lastpos[Z_AXIS];
    RUNPLAN(FILAMENT_CHANGE_Z_FEEDRATE);
  #endif
  stepper.synchronize();

  #if ENABLED(FILAMENT_RUNOUT_SENSOR)
    filament_ran_out = false;
  #endif

  // Show status screen
  lcd_filament_change_show_message(FILAMENT_CHANGE_MESSAGE_STATUS);
}
