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
 * M665: Set delta configurations
 *
 *    L = diagonal rod
 *    R = delta radius
 *    S = segments per second
 *    A = Alpha (Tower 1) diagonal rod trim
 *    B = Beta (Tower 2) diagonal rod trim
 *    C = Gamma (Tower 3) diagonal rod trim
 */
inline void gcode_M665() {
  if (code_seen('L')) delta_diagonal_rod = code_value_linear_units();
  if (code_seen('R')) delta_radius = code_value_linear_units();
  if (code_seen('S')) delta_segments_per_second = code_value_float();
  if (code_seen('A')) delta_diagonal_rod_trim_tower_1 = code_value_linear_units();
  if (code_seen('B')) delta_diagonal_rod_trim_tower_2 = code_value_linear_units();
  if (code_seen('C')) delta_diagonal_rod_trim_tower_3 = code_value_linear_units();
  recalc_delta_settings(delta_radius, delta_diagonal_rod);
}
