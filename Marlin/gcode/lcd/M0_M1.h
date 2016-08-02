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
 * M0: Unconditional stop - Wait for user button press on LCD
 * M1: Conditional stop   - Wait for user button press on LCD
 */
inline void gcode_M0_M1() {
  char* args = current_command_args;

  millis_t codenum = 0;
  bool hasP = false, hasS = false;
  if (code_seen('P')) {
    codenum = code_value_millis(); // milliseconds to wait
    hasP = codenum > 0;
  }
  if (code_seen('S')) {
    codenum = code_value_millis_from_seconds(); // seconds to wait
    hasS = codenum > 0;
  }

  if (!hasP && !hasS && *args != '\0')
    lcd_setstatus(args, true);
  else {
    LCD_MESSAGEPGM(MSG_USERWAIT);
    #if ENABLED(LCD_PROGRESS_BAR) && PROGRESS_MSG_EXPIRE > 0
      dontExpireStatus();
    #endif
  }

  lcd_ignore_click();
  stepper.synchronize();
  refresh_cmd_timeout();
  if (codenum > 0) {
    codenum += previous_cmd_ms;  // wait until this time for a click
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    while (PENDING(millis(), codenum) && !lcd_clicked()) idle();
    KEEPALIVE_STATE(IN_HANDLER);
    lcd_ignore_click(false);
  }
  else {
    if (!lcd_detected()) return;
    KEEPALIVE_STATE(PAUSED_FOR_USER);
    while (!lcd_clicked()) idle();
    KEEPALIVE_STATE(IN_HANDLER);
  }
  if (IS_SD_PRINTING)
    LCD_MESSAGEPGM(MSG_RESUMING);
  else
    LCD_MESSAGEPGM(WELCOME_MSG);
}
