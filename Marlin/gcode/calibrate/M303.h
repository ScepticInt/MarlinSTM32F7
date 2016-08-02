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
 * M303: PID relay autotune
 *
 *       S<temperature> sets the target temperature. (default 150C)
 *       E<extruder> (-1 for the bed) (default 0)
 *       C<cycles>
 *       U<bool> with a non-zero value will apply the result to current settings
 */
inline void gcode_M303() {
  #if HAS_PID_HEATING
    int e = code_seen('E') ? code_value_int() : 0;
    int c = code_seen('C') ? code_value_int() : 5;
    bool u = code_seen('U') && code_value_bool();

    float temp = code_seen('S') ? code_value_temp_abs() : (e < 0 ? 70.0 : 150.0);

    if (e >= 0 && e < HOTENDS)
      target_extruder = e;

    KEEPALIVE_STATE(NOT_BUSY); // don't send "busy: processing" messages during autotune output

    thermalManager.PID_autotune(temp, e, c, u);

    KEEPALIVE_STATE(IN_HANDLER);
  #else
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_M303_DISABLED);
  #endif
}
