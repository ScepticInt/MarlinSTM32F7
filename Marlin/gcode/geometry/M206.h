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
 * M206: Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
 */
inline void gcode_M206() {
  LOOP_XYZ(i)
    if (code_seen(axis_codes[i]))
      set_home_offset((AxisEnum)i, code_value_axis_units(i));

  #if ENABLED(SCARA)
    if (code_seen('T')) set_home_offset(X_AXIS, code_value_axis_units(X_AXIS)); // Theta
    if (code_seen('P')) set_home_offset(Y_AXIS, code_value_axis_units(Y_AXIS)); // Psi
  #endif

  SYNC_PLAN_POSITION_KINEMATIC();
  report_current_position();
}
