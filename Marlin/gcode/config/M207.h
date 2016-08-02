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
 * M207: Set firmware retraction values
 *
 *   S[+units]    retract_length
 *   W[+units]    retract_length_swap (multi-extruder)
 *   F[units/min] retract_feedrate_mm_s
 *   Z[units]     retract_zlift
 */
inline void gcode_M207() {
  if (code_seen('S')) retract_length = code_value_axis_units(E_AXIS);
  if (code_seen('F')) retract_feedrate_mm_s = MMM_TO_MMS(code_value_axis_units(E_AXIS));
  if (code_seen('Z')) retract_zlift = code_value_axis_units(Z_AXIS);
  #if EXTRUDERS > 1
    if (code_seen('W')) retract_length_swap = code_value_axis_units(E_AXIS);
  #endif
}
