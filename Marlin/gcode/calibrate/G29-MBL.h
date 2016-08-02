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

inline void _mbl_goto_xy(float x, float y) {
  float old_feedrate_mm_m = feedrate_mm_m;
  feedrate_mm_m = homing_feedrate_mm_m[X_AXIS];

  current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
    #if Z_RAISE_BETWEEN_PROBINGS > MIN_Z_HEIGHT_FOR_HOMING
      + Z_RAISE_BETWEEN_PROBINGS
    #elif MIN_Z_HEIGHT_FOR_HOMING > 0
      + MIN_Z_HEIGHT_FOR_HOMING
    #endif
  ;
  line_to_current_position();

  current_position[X_AXIS] = LOGICAL_X_POSITION(x);
  current_position[Y_AXIS] = LOGICAL_Y_POSITION(y);
  line_to_current_position();

  #if Z_RAISE_BETWEEN_PROBINGS > 0 || MIN_Z_HEIGHT_FOR_HOMING > 0
    current_position[Z_AXIS] = LOGICAL_Z_POSITION(MESH_HOME_SEARCH_Z);
    line_to_current_position();
  #endif

  feedrate_mm_m = old_feedrate_mm_m;
  stepper.synchronize();
}

/**
 * G29: Mesh-based Z probe, probes a grid and produces a
 *      mesh to compensate for variable bed height
 *
 * Parameters With MESH_BED_LEVELING:
 *
 *  S0              Produce a mesh report
 *  S1              Start probing mesh points
 *  S2              Probe the next mesh point
 *  S3 Xn Yn Zn.nn  Manually modify a single point
 *  S4 Zn.nn        Set z offset. Positive away from bed, negative closer to bed.
 *  S5              Reset and disable mesh
 *
 * The S0 report the points as below
 *
 *  +----> X-axis  1-n
 *  |
 *  |
 *  v Y-axis  1-n
 *
 */
inline void gcode_G29() {

  static int probe_point = -1;
  MeshLevelingState state = code_seen('S') ? (MeshLevelingState)code_value_byte() : MeshReport;
  if (state < 0 || state > 5) {
    SERIAL_PROTOCOLLNPGM("S out of range (0-5).");
    return;
  }

  int8_t px, py;

  switch (state) {
    case MeshReport:
      if (mbl.has_mesh()) {
        SERIAL_PROTOCOLPAIR("State: ", mbl.active() ? "On" : "Off");
        SERIAL_PROTOCOLPAIR("\nNum X,Y: ", MESH_NUM_X_POINTS);
        SERIAL_PROTOCOLCHAR(','); SERIAL_PROTOCOL(MESH_NUM_Y_POINTS);
        SERIAL_PROTOCOLPAIR("\nZ search height: ", MESH_HOME_SEARCH_Z);
        SERIAL_PROTOCOLPGM("\nZ offset: "); SERIAL_PROTOCOL_F(mbl.z_offset, 5);
        SERIAL_PROTOCOLLNPGM("\nMeasured points:");
        for (py = 0; py < MESH_NUM_Y_POINTS; py++) {
          for (px = 0; px < MESH_NUM_X_POINTS; px++) {
            SERIAL_PROTOCOLPGM("  ");
            SERIAL_PROTOCOL_F(mbl.z_values[py][px], 5);
          }
          SERIAL_EOL;
        }
      }
      else
        SERIAL_PROTOCOLLNPGM("Mesh bed leveling not active.");
      break;

    case MeshStart:
      mbl.reset();
      probe_point = 0;
      enqueue_and_echo_commands_P(PSTR("G28\nG29 S2"));
      break;

    case MeshNext:
      if (probe_point < 0) {
        SERIAL_PROTOCOLLNPGM("Start mesh probing with \"G29 S1\" first.");
        return;
      }
      // For each G29 S2...
      if (probe_point == 0) {
        // For the initial G29 S2 make Z a positive value (e.g., 4.0)
        current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
          #if Z_HOME_DIR > 0
            + Z_MAX_POS
          #endif
        ;
        SYNC_PLAN_POSITION_KINEMATIC();
      }
      else {
        // For G29 S2 after adjusting Z.
        mbl.set_zigzag_z(probe_point - 1, current_position[Z_AXIS]);
      }
      // If there's another point to sample, move there with optional lift.
      if (probe_point < (MESH_NUM_X_POINTS) * (MESH_NUM_Y_POINTS)) {
        mbl.zigzag(probe_point, px, py);
        _mbl_goto_xy(mbl.get_probe_x(px), mbl.get_probe_y(py));
        probe_point++;
      }
      else {
        // One last "return to the bed" (as originally coded) at completion
        current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
          #if Z_RAISE_BETWEEN_PROBINGS > MIN_Z_HEIGHT_FOR_HOMING
            + Z_RAISE_BETWEEN_PROBINGS
          #elif MIN_Z_HEIGHT_FOR_HOMING > 0
            + MIN_Z_HEIGHT_FOR_HOMING
          #endif
        ;
        line_to_current_position();
        stepper.synchronize();

        // After recording the last point, activate the mbl and home
        SERIAL_PROTOCOLLNPGM("Mesh probing done.");
        probe_point = -1;
        mbl.set_has_mesh(true);
        enqueue_and_echo_commands_P(PSTR("G28"));
      }
      break;

    case MeshSet:
      if (code_seen('X')) {
        px = code_value_int() - 1;
        if (px < 0 || px >= MESH_NUM_X_POINTS) {
          SERIAL_PROTOCOLLNPGM("X out of range (1-" STRINGIFY(MESH_NUM_X_POINTS) ").");
          return;
        }
      }
      else {
        SERIAL_PROTOCOLLNPGM("X not entered.");
        return;
      }
      if (code_seen('Y')) {
        py = code_value_int() - 1;
        if (py < 0 || py >= MESH_NUM_Y_POINTS) {
          SERIAL_PROTOCOLLNPGM("Y out of range (1-" STRINGIFY(MESH_NUM_Y_POINTS) ").");
          return;
        }
      }
      else {
        SERIAL_PROTOCOLLNPGM("Y not entered.");
        return;
      }
      if (code_seen('Z')) {
        mbl.z_values[py][px] = code_value_axis_units(Z_AXIS);
      }
      else {
        SERIAL_PROTOCOLLNPGM("Z not entered.");
        return;
      }
      break;

    case MeshSetZOffset:
      if (code_seen('Z')) {
        mbl.z_offset = code_value_axis_units(Z_AXIS);
      }
      else {
        SERIAL_PROTOCOLLNPGM("Z not entered.");
        return;
      }
      break;

    case MeshReset:
      if (mbl.active()) {
        current_position[Z_AXIS] +=
          mbl.get_z(RAW_CURRENT_POSITION(X_AXIS), RAW_CURRENT_POSITION(Y_AXIS)) - MESH_HOME_SEARCH_Z;
        mbl.reset();
        SYNC_PLAN_POSITION_KINEMATIC();
      }
      else
        mbl.reset();

  } // switch(state)

  report_current_position();
}
