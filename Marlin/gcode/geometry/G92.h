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
 * G92: Set current position to given X Y Z E
 */
inline void gcode_G92() {
  bool didE = code_seen('E');

  if (!didE) stepper.synchronize();

  bool didXYZ = false;
  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i])) {
      float p = current_position[i],
            v = code_value_axis_units(i);

      current_position[i] = v;

      if (i != E_AXIS) {
        position_shift[i] += v - p; // Offset the coordinate space
        update_software_endstops((AxisEnum)i);
        didXYZ = true;
      }
    }
  }
  if (didXYZ)
    SYNC_PLAN_POSITION_KINEMATIC();
  else if (didE)
    sync_plan_position_e();
}
