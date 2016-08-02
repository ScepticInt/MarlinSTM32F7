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
 * M208: Set firmware un-retraction values
 *
 *   S[+units]    retract_recover_length (in addition to M207 S*)
 *   W[+units]    retract_recover_length_swap (multi-extruder)
 *   F[units/min] retract_recover_feedrate_mm_s
 */
inline void gcode_M208() {
  if (code_seen('S')) retract_recover_length = code_value_axis_units(E_AXIS);
  if (code_seen('F')) retract_recover_feedrate_mm_s = MMM_TO_MMS(code_value_axis_units(E_AXIS));
  #if EXTRUDERS > 1
    if (code_seen('W')) retract_recover_length_swap = code_value_axis_units(E_AXIS);
  #endif
}
