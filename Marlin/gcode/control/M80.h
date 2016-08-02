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
 * M80: Turn on Power Supply
 */
inline void gcode_M80() {
  OUT_WRITE(PS_ON_PIN, PS_ON_AWAKE); //GND

  /**
   * If you have a switch on suicide pin, this is useful
   * if you want to start another print with suicide feature after
   * a print without suicide...
   */
  #if HAS_SUICIDE
    OUT_WRITE(SUICIDE_PIN, HIGH);
  #endif

  #if ENABLED(ULTIPANEL)
    powersupply = true;
    LCD_MESSAGEPGM(WELCOME_MSG);
    lcd_update();
  #endif
}

