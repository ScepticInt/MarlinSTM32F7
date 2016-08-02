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
 * G30: Do a single Z probe at the current XY
 */
inline void gcode_G30() {

  set_destination_to_current();

  if (code_seen('X'))
    destination[X_AXIS] = constrain(code_value_float() + X_PROBE_OFFSET_FROM_EXTRUDER, sw_endstop_min[X_AXIS], sw_endstop_max[X_AXIS]);

  if (code_seen('Y'))
    destination[Y_AXIS] = constrain(code_value_float() + Y_PROBE_OFFSET_FROM_EXTRUDER, sw_endstop_min[Y_AXIS], sw_endstop_max[Y_AXIS]);

  do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);

  setup_for_endstop_or_probe_move();

  // TODO: clear the leveling matrix or the planner will be set incorrectly
  float measured_z = probe_pt(current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER,
                              current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER,
                              true, 1);

  SERIAL_PROTOCOLPGM("Bed X: ");
  SERIAL_PROTOCOL(current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER + 0.0001);
  SERIAL_PROTOCOLPGM(" Y: ");
  SERIAL_PROTOCOL(current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER + 0.0001);
  SERIAL_PROTOCOLPGM(" Z: ");
  SERIAL_PROTOCOL(measured_z + 0.0001);
  SERIAL_EOL;

  clean_up_after_endstop_or_probe_move();

  report_current_position();
}
