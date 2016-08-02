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
 * M155: Send data to a I2C slave device
 *
 * This is a PoC, the formating and arguments for the GCODE will
 * change to be more compatible, the current proposal is:
 *
 *  M155 A<slave device address base 10> ; Sets the I2C slave address the data will be sent to
 *
 *  M155 B<byte-1 value in base 10>
 *  M155 B<byte-2 value in base 10>
 *  M155 B<byte-3 value in base 10>
 *
 *  M155 S1 ; Send the buffered data and reset the buffer
 *  M155 R1 ; Reset the buffer without sending data
 *
 */
inline void gcode_M155() {
  // Set the target address
  if (code_seen('A'))
    i2c.address(code_value_byte());

  // Add a new byte to the buffer
  else if (code_seen('B'))
    i2c.addbyte(code_value_int());

  // Flush the buffer to the bus
  else if (code_seen('S')) i2c.send();

  // Reset and rewind the buffer
  else if (code_seen('R')) i2c.reset();
}
