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
 * M421: Set a single Mesh Bed Leveling Z coordinate
 * Use either 'M421 X<linear> Y<linear> Z<linear>' or 'M421 I<xindex> J<yindex> Z<linear>'
 */
inline void gcode_M421() {
  int8_t px = 0, py = 0;
  float z = 0;
  bool hasX, hasY, hasZ, hasI, hasJ;
  if ((hasX = code_seen('X'))) px = mbl.probe_index_x(code_value_axis_units(X_AXIS));
  if ((hasY = code_seen('Y'))) py = mbl.probe_index_y(code_value_axis_units(Y_AXIS));
  if ((hasI = code_seen('I'))) px = code_value_axis_units(X_AXIS);
  if ((hasJ = code_seen('J'))) py = code_value_axis_units(Y_AXIS);
  if ((hasZ = code_seen('Z'))) z = code_value_axis_units(Z_AXIS);

  if (hasX && hasY && hasZ) {

    if (px >= 0 && py >= 0)
      mbl.set_z(px, py, z);
    else {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_ERR_MESH_XY);
    }
  }
  else if (hasI && hasJ && hasZ) {
    if (px >= 0 && px < MESH_NUM_X_POINTS && py >= 0 && py < MESH_NUM_Y_POINTS)
      mbl.set_z(px, py, z);
    else {
      SERIAL_ERROR_START;
      SERIAL_ERRORLNPGM(MSG_ERR_MESH_XY);
    }
  }
  else {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(MSG_ERR_M421_PARAMETERS);
  }
}
