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
 * M301: Set PID (CL) for a hotend
 *
 *   P[float] Kp term
 *   I[float] Ki term (unscaled)
 *   D[float] Kd term (unscaled)
 *
 * With PID_ADD_EXTRUSION_RATE:
 *
 *   C[float] Kc term
 *   L[float] LPQ length
 */
inline void gcode_M301() {

  // multi-extruder PID patch: M301 updates or prints a single extruder's PID values
  // default behaviour (omitting E parameter) is to update for extruder 0 only
  int e = code_seen('E') ? code_value_int() : 0; // extruder being updated

  if (e < HOTENDS) { // catch bad input value
    if (code_seen('P')) PID_PARAM(Kp, e) = code_value_float();
    if (code_seen('I')) PID_PARAM(Ki, e) = scalePID_i(code_value_float());
    if (code_seen('D')) PID_PARAM(Kd, e) = scalePID_d(code_value_float());
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      if (code_seen('C')) PID_PARAM(Kc, e) = code_value_float();
      if (code_seen('L')) lpq_len = code_value_float();
      NOMORE(lpq_len, LPQ_MAX_LEN);
    #endif

    thermalManager.updatePID();
    SERIAL_ECHO_START;
    #if ENABLED(PID_PARAMS_PER_HOTEND)
      SERIAL_ECHOPGM(" e:"); // specify extruder in serial output
      SERIAL_ECHO(e);
    #endif // PID_PARAMS_PER_HOTEND
    SERIAL_ECHOPGM(" p:");
    SERIAL_ECHO(PID_PARAM(Kp, e));
    SERIAL_ECHOPGM(" i:");
    SERIAL_ECHO(unscalePID_i(PID_PARAM(Ki, e)));
    SERIAL_ECHOPGM(" d:");
    SERIAL_ECHO(unscalePID_d(PID_PARAM(Kd, e)));
    #if ENABLED(PID_ADD_EXTRUSION_RATE)
      SERIAL_ECHOPGM(" c:");
      //Kc does not have scaling applied above, or in resetting defaults
      SERIAL_ECHO(PID_PARAM(Kc, e));
    #endif
    SERIAL_EOL;
  }
  else {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN(MSG_INVALID_EXTRUDER);
  }
}
