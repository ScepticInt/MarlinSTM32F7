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
 * M428: Set home_offset based on the distance between the
 *       current_position and the nearest "reference point."
 *       If an axis is past center its endstop position
 *       is the reference-point. Otherwise it uses 0. This allows
 *       the Z offset to be set near the bed when using a max endstop.
 *
 *       M428 can't be used more than 2cm away from 0 or an endstop.
 *
 *       Use M206 to set these values directly.
 */
inline void gcode_M428() {
  bool err = false;
  LOOP_XYZ(i) {
    if (axis_homed[i]) {
      float base = (current_position[i] > (sw_endstop_min[i] + sw_endstop_max[i]) * 0.5) ? base_home_pos(i) : 0,
            diff = current_position[i] - LOGICAL_POSITION(base, i);
      if (diff > -20 && diff < 20) {
        set_home_offset((AxisEnum)i, home_offset[i] - diff);
      }
      else {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM(MSG_ERR_M428_TOO_FAR);
        LCD_ALERTMESSAGEPGM("Err: Too far!");
        #if HAS_BUZZER
          buzzer.tone(200, 40);
        #endif
        err = true;
        break;
      }
    }
  }

  if (!err) {
    SYNC_PLAN_POSITION_KINEMATIC();
    report_current_position();
    LCD_MESSAGEPGM(MSG_HOME_OFFSETS_APPLIED);
    #if HAS_BUZZER
      buzzer.tone(200, 659);
      buzzer.tone(200, 698);
    #endif
  }
}

