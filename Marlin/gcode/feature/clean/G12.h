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
 * G12: Clean the nozzle
 */
inline void gcode_G12() {
  // Don't allow nozzle cleaning without homing first
  if (axis_unhomed_error(true, true, true)) { return; }

  uint8_t const pattern = code_seen('P') ? code_value_ushort() : 0;
  uint8_t const strokes = code_seen('S') ? code_value_ushort() : NOZZLE_CLEAN_STROKES;
  uint8_t const objects = code_seen('T') ? code_value_ushort() : 3;

  Nozzle::clean(pattern, strokes, objects);
}
