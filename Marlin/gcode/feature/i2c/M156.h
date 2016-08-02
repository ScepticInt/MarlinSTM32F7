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
 * M156: Request X bytes from I2C slave device
 *
 * Usage: M156 A<slave device address base 10> B<number of bytes>
 */
inline void gcode_M156() {
  uint8_t addr = code_seen('A') ? code_value_byte() : 0;
  int bytes    = code_seen('B') ? code_value_int() : 1;

  if (addr && bytes > 0 && bytes <= 32) {
    i2c.address(addr);
    i2c.reqbytes(bytes);
  }
  else {
    SERIAL_ERROR_START;
    SERIAL_ERRORLN("Bad i2c request");
  }
}
