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

bool SCARA_move_to_cal(uint8_t delta_x, uint8_t delta_y) {
  //SoftEndsEnabled = false;              // Ignore soft endstops during calibration
  //SERIAL_ECHOLNPGM(" Soft endstops disabled");
  if (IsRunning()) {
    //gcode_get_destination(); // For X Y Z E F
    delta[X_AXIS] = delta_x;
    delta[Y_AXIS] = delta_y;
    forward_kinematics_SCARA(delta);
    destination[X_AXIS] = delta[X_AXIS] / axis_scaling[X_AXIS];
    destination[Y_AXIS] = delta[Y_AXIS] / axis_scaling[Y_AXIS];
    prepare_move_to_destination();
    //ok_to_send();
    return true;
  }
  return false;
}

/**
 * M360: SCARA calibration: Move to cal-position ThetaA (0 deg calibration)
 */
inline bool gcode_M360() {
  SERIAL_ECHOLNPGM(" Cal: Theta 0");
  return SCARA_move_to_cal(0, 120);
}

/**
 * M361: SCARA calibration: Move to cal-position ThetaB (90 deg calibration - steps per degree)
 */
inline bool gcode_M361() {
  SERIAL_ECHOLNPGM(" Cal: Theta 90");
  return SCARA_move_to_cal(90, 130);
}

/**
 * M362: SCARA calibration: Move to cal-position PsiA (0 deg calibration)
 */
inline bool gcode_M362() {
  SERIAL_ECHOLNPGM(" Cal: Psi 0");
  return SCARA_move_to_cal(60, 180);
}

/**
 * M363: SCARA calibration: Move to cal-position PsiB (90 deg calibration - steps per degree)
 */
inline bool gcode_M363() {
  SERIAL_ECHOLNPGM(" Cal: Psi 90");
  return SCARA_move_to_cal(50, 90);
}

/**
 * M364: SCARA calibration: Move to cal-position PSIC (90 deg to Theta calibration position)
 */
inline bool gcode_M364() {
  SERIAL_ECHOLNPGM(" Cal: Theta-Psi 90");
  return SCARA_move_to_cal(45, 135);
}

/**
 * M365: SCARA calibration: Scaling factor, X, Y, Z axis
 */
inline void gcode_M365() {
  LOOP_XYZ(i)
    if (code_seen(axis_codes[i]))
      axis_scaling[i] = code_value_float();
}
