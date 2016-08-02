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
 * G29: Detailed Z probe, probes the bed at 3 or more points.
 *      Will fail if the printer has not been homed with G28.
 *
 * Enhanced G29 Auto Bed Leveling Probe Routine
 *
 * Parameters With AUTO_BED_LEVELING_GRID:
 *
 *  P  Set the size of the grid that will be probed (P x P points).
 *     Not supported by non-linear delta printer bed leveling.
 *     Example: "G29 P4"
 *
 *  S  Set the XY travel speed between probe points (in units/min)
 *
 *  D  Dry-Run mode. Just evaluate the bed Topology - Don't apply
 *     or clean the rotation Matrix. Useful to check the topology
 *     after a first run of G29.
 *
 *  V  Set the verbose level (0-4). Example: "G29 V3"
 *
 *  T  Generate a Bed Topology Report. Example: "G29 P5 T" for a detailed report.
 *     This is useful for manual bed leveling and finding flaws in the bed (to
 *     assist with part placement).
 *     Not supported by non-linear delta printer bed leveling.
 *
 *  F  Set the Front limit of the probing grid
 *  B  Set the Back limit of the probing grid
 *  L  Set the Left limit of the probing grid
 *  R  Set the Right limit of the probing grid
 *
 * Global Parameters:
 *
 * E/e By default G29 will engage the Z probe, test the bed, then disengage.
 *     Include "E" to engage/disengage the Z probe for each sample.
 *     There's no extra effect if you have a fixed Z probe.
 *     Usage: "G29 E" or "G29 e"
 *
 */
inline void gcode_G29() {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) {
      SERIAL_ECHOLNPGM(">>> gcode_G29");
      DEBUG_POS("", current_position);
    }
  #endif

  // Don't allow auto-leveling without homing first
  if (axis_unhomed_error(true, true, true)) return;

  int verbose_level = code_seen('V') ? code_value_int() : 1;
  if (verbose_level < 0 || verbose_level > 4) {
    SERIAL_ECHOLNPGM("?(V)erbose Level is implausible (0-4).");
    return;
  }

  bool dryrun = code_seen('D');
  bool stow_probe_after_each = code_seen('E');

  #if ENABLED(AUTO_BED_LEVELING_GRID)

    #if DISABLED(DELTA)
      bool do_topography_map = verbose_level > 2 || code_seen('T');
    #endif

    if (verbose_level > 0) {
      SERIAL_PROTOCOLLNPGM("G29 Auto Bed Leveling");
      if (dryrun) SERIAL_PROTOCOLLNPGM("Running in DRY-RUN mode");
    }

    int auto_bed_leveling_grid_points = AUTO_BED_LEVELING_GRID_POINTS;

    #if DISABLED(DELTA)
      if (code_seen('P')) auto_bed_leveling_grid_points = code_value_int();
      if (auto_bed_leveling_grid_points < 2) {
        SERIAL_PROTOCOLLNPGM("?Number of probed (P)oints is implausible (2 minimum).");
        return;
      }
    #endif

    xy_probe_feedrate_mm_m = code_seen('S') ? (int)code_value_linear_units() : XY_PROBE_SPEED;

    int left_probe_bed_position = code_seen('L') ? (int)code_value_axis_units(X_AXIS) : LOGICAL_X_POSITION(LEFT_PROBE_BED_POSITION),
        right_probe_bed_position = code_seen('R') ? (int)code_value_axis_units(X_AXIS) : LOGICAL_X_POSITION(RIGHT_PROBE_BED_POSITION),
        front_probe_bed_position = code_seen('F') ? (int)code_value_axis_units(Y_AXIS) : LOGICAL_Y_POSITION(FRONT_PROBE_BED_POSITION),
        back_probe_bed_position = code_seen('B') ? (int)code_value_axis_units(Y_AXIS) : LOGICAL_Y_POSITION(BACK_PROBE_BED_POSITION);

    bool left_out_l = left_probe_bed_position < LOGICAL_X_POSITION(MIN_PROBE_X),
         left_out = left_out_l || left_probe_bed_position > right_probe_bed_position - (MIN_PROBE_EDGE),
         right_out_r = right_probe_bed_position > LOGICAL_X_POSITION(MAX_PROBE_X),
         right_out = right_out_r || right_probe_bed_position < left_probe_bed_position + MIN_PROBE_EDGE,
         front_out_f = front_probe_bed_position < LOGICAL_Y_POSITION(MIN_PROBE_Y),
         front_out = front_out_f || front_probe_bed_position > back_probe_bed_position - (MIN_PROBE_EDGE),
         back_out_b = back_probe_bed_position > LOGICAL_Y_POSITION(MAX_PROBE_Y),
         back_out = back_out_b || back_probe_bed_position < front_probe_bed_position + MIN_PROBE_EDGE;

    if (left_out || right_out || front_out || back_out) {
      if (left_out) {
        out_of_range_error(PSTR("(L)eft"));
        left_probe_bed_position = left_out_l ? LOGICAL_X_POSITION(MIN_PROBE_X) : right_probe_bed_position - (MIN_PROBE_EDGE);
      }
      if (right_out) {
        out_of_range_error(PSTR("(R)ight"));
        right_probe_bed_position = right_out_r ? LOGICAL_Y_POSITION(MAX_PROBE_X) : left_probe_bed_position + MIN_PROBE_EDGE;
      }
      if (front_out) {
        out_of_range_error(PSTR("(F)ront"));
        front_probe_bed_position = front_out_f ? LOGICAL_Y_POSITION(MIN_PROBE_Y) : back_probe_bed_position - (MIN_PROBE_EDGE);
      }
      if (back_out) {
        out_of_range_error(PSTR("(B)ack"));
        back_probe_bed_position = back_out_b ? LOGICAL_Y_POSITION(MAX_PROBE_Y) : front_probe_bed_position + MIN_PROBE_EDGE;
      }
      return;
    }

  #endif // AUTO_BED_LEVELING_GRID

  if (!dryrun) {

    #if ENABLED(DEBUG_LEVELING_FEATURE) && DISABLED(DELTA)
      if (DEBUGGING(LEVELING)) {
        vector_3 corrected_position = planner.adjusted_position();
        DEBUG_POS("BEFORE matrix.set_to_identity", corrected_position);
        DEBUG_POS("BEFORE matrix.set_to_identity", current_position);
      }
    #endif

    // make sure the bed_level_rotation_matrix is identity or the planner will get it wrong
    planner.bed_level_matrix.set_to_identity();

    #if ENABLED(DELTA)
      reset_bed_level();
    #else //!DELTA

      //vector_3 corrected_position = planner.adjusted_position();
      //corrected_position.debug("position before G29");
      vector_3 uncorrected_position = planner.adjusted_position();
      //uncorrected_position.debug("position during G29");
      current_position[X_AXIS] = uncorrected_position.x;
      current_position[Y_AXIS] = uncorrected_position.y;
      current_position[Z_AXIS] = uncorrected_position.z;

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("AFTER matrix.set_to_identity", uncorrected_position);
      #endif

      SYNC_PLAN_POSITION_KINEMATIC();

    #endif // !DELTA
  }

  stepper.synchronize();

  setup_for_endstop_or_probe_move();

  // Deploy the probe. Probe will raise if needed.
  if (DEPLOY_PROBE()) return;

  bed_leveling_in_progress = true;

  #if ENABLED(AUTO_BED_LEVELING_GRID)

    // probe at the points of a lattice grid
    const int xGridSpacing = (right_probe_bed_position - left_probe_bed_position) / (auto_bed_leveling_grid_points - 1),
              yGridSpacing = (back_probe_bed_position - front_probe_bed_position) / (auto_bed_leveling_grid_points - 1);

    #if ENABLED(DELTA)
      delta_grid_spacing[0] = xGridSpacing;
      delta_grid_spacing[1] = yGridSpacing;
      float zoffset = zprobe_zoffset;
      if (code_seen('Z')) zoffset += code_value_axis_units(Z_AXIS);
    #else // !DELTA
      /**
       * solve the plane equation ax + by + d = z
       * A is the matrix with rows [x y 1] for all the probed points
       * B is the vector of the Z positions
       * the normal vector to the plane is formed by the coefficients of the
       * plane equation in the standard form, which is Vx*x+Vy*y+Vz*z+d = 0
       * so Vx = -a Vy = -b Vz = 1 (we want the vector facing towards positive Z
       */

      int abl2 = sq(auto_bed_leveling_grid_points);

      double eqnAMatrix[abl2 * 3], // "A" matrix of the linear system of equations
             eqnBVector[abl2],     // "B" vector of Z points
             mean = 0.0;
      int8_t indexIntoAB[auto_bed_leveling_grid_points][auto_bed_leveling_grid_points];
    #endif // !DELTA

    int probePointCounter = 0;
    bool zig = (auto_bed_leveling_grid_points & 1) ? true : false; //always end at [RIGHT_PROBE_BED_POSITION, BACK_PROBE_BED_POSITION]

    for (int yCount = 0; yCount < auto_bed_leveling_grid_points; yCount++) {
      double yProbe = front_probe_bed_position + yGridSpacing * yCount;
      int xStart, xStop, xInc;

      if (zig) {
        xStart = 0;
        xStop = auto_bed_leveling_grid_points;
        xInc = 1;
      }
      else {
        xStart = auto_bed_leveling_grid_points - 1;
        xStop = -1;
        xInc = -1;
      }

      zig = !zig;

      for (int xCount = xStart; xCount != xStop; xCount += xInc) {
        double xProbe = left_probe_bed_position + xGridSpacing * xCount;

        #if ENABLED(DELTA)
          // Avoid probing the corners (outside the round or hexagon print surface) on a delta printer.
          float distance_from_center = HYPOT(xProbe, yProbe);
          if (distance_from_center > DELTA_PROBEABLE_RADIUS) continue;
        #endif //DELTA

        float measured_z = probe_pt(xProbe, yProbe, stow_probe_after_each, verbose_level);

        #if DISABLED(DELTA)
          mean += measured_z;

          eqnBVector[probePointCounter] = measured_z;
          eqnAMatrix[probePointCounter + 0 * abl2] = xProbe;
          eqnAMatrix[probePointCounter + 1 * abl2] = yProbe;
          eqnAMatrix[probePointCounter + 2 * abl2] = 1;
          indexIntoAB[xCount][yCount] = probePointCounter;
        #else
          bed_level[xCount][yCount] = measured_z + zoffset;
        #endif

        probePointCounter++;

        idle();

      } //xProbe
    } //yProbe

  #else // !AUTO_BED_LEVELING_GRID

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> 3-point Leveling");
    #endif

    // Probe at 3 arbitrary points
    float z_at_pt_1 = probe_pt( LOGICAL_X_POSITION(ABL_PROBE_PT_1_X),
                                LOGICAL_Y_POSITION(ABL_PROBE_PT_1_Y),
                                stow_probe_after_each, verbose_level),
          z_at_pt_2 = probe_pt( LOGICAL_X_POSITION(ABL_PROBE_PT_2_X),
                                LOGICAL_Y_POSITION(ABL_PROBE_PT_2_Y),
                                stow_probe_after_each, verbose_level),
          z_at_pt_3 = probe_pt( LOGICAL_X_POSITION(ABL_PROBE_PT_3_X),
                                LOGICAL_Y_POSITION(ABL_PROBE_PT_3_Y),
                                stow_probe_after_each, verbose_level);

    if (!dryrun) set_bed_level_equation_3pts(z_at_pt_1, z_at_pt_2, z_at_pt_3);

  #endif // !AUTO_BED_LEVELING_GRID

  // Raise to _Z_RAISE_PROBE_DEPLOY_STOW. Stow the probe.
  if (STOW_PROBE()) return;

  // Restore state after probing
  clean_up_after_endstop_or_probe_move();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) DEBUG_POS("> probing complete", current_position);
  #endif

  // Calculate leveling, print reports, correct the position
  #if ENABLED(AUTO_BED_LEVELING_GRID)
    #if ENABLED(DELTA)

      if (!dryrun) extrapolate_unprobed_bed_level();
      print_bed_level();

    #else // !DELTA

      // solve lsq problem
      double plane_equation_coefficients[3];
      qr_solve(plane_equation_coefficients, abl2, 3, eqnAMatrix, eqnBVector);

      mean /= abl2;

      if (verbose_level) {
        SERIAL_PROTOCOLPGM("Eqn coefficients: a: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[0], 8);
        SERIAL_PROTOCOLPGM(" b: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[1], 8);
        SERIAL_PROTOCOLPGM(" d: ");
        SERIAL_PROTOCOL_F(plane_equation_coefficients[2], 8);
        SERIAL_EOL;
        if (verbose_level > 2) {
          SERIAL_PROTOCOLPGM("Mean of sampled points: ");
          SERIAL_PROTOCOL_F(mean, 8);
          SERIAL_EOL;
        }
      }

      if (!dryrun) set_bed_level_equation_lsq(plane_equation_coefficients);

      // Show the Topography map if enabled
      if (do_topography_map) {

        SERIAL_PROTOCOLLNPGM("\nBed Height Topography:\n"
                               "   +--- BACK --+\n"
                               "   |           |\n"
                               " L |    (+)    | R\n"
                               " E |           | I\n"
                               " F | (-) N (+) | G\n"
                               " T |           | H\n"
                               "   |    (-)    | T\n"
                               "   |           |\n"
                               "   O-- FRONT --+\n"
                               " (0,0)");

        float min_diff = 999;

        for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
          for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
            int ind = indexIntoAB[xx][yy];
            float diff = eqnBVector[ind] - mean;

            float x_tmp = eqnAMatrix[ind + 0 * abl2],
                  y_tmp = eqnAMatrix[ind + 1 * abl2],
                  z_tmp = 0;

            apply_rotation_xyz(planner.bed_level_matrix, x_tmp, y_tmp, z_tmp);

            NOMORE(min_diff, eqnBVector[ind] - z_tmp);

            if (diff >= 0.0)
              SERIAL_PROTOCOLPGM(" +");   // Include + for column alignment
            else
              SERIAL_PROTOCOLCHAR(' ');
            SERIAL_PROTOCOL_F(diff, 5);
          } // xx
          SERIAL_EOL;
        } // yy
        SERIAL_EOL;
        if (verbose_level > 3) {
          SERIAL_PROTOCOLLNPGM("\nCorrected Bed Height vs. Bed Topology:");

          for (int yy = auto_bed_leveling_grid_points - 1; yy >= 0; yy--) {
            for (int xx = 0; xx < auto_bed_leveling_grid_points; xx++) {
              int ind = indexIntoAB[xx][yy];
              float x_tmp = eqnAMatrix[ind + 0 * abl2],
                    y_tmp = eqnAMatrix[ind + 1 * abl2],
                    z_tmp = 0;

              apply_rotation_xyz(planner.bed_level_matrix, x_tmp, y_tmp, z_tmp);

              float diff = eqnBVector[ind] - z_tmp - min_diff;
              if (diff >= 0.0)
                SERIAL_PROTOCOLPGM(" +");
              // Include + for column alignment
              else
                SERIAL_PROTOCOLCHAR(' ');
              SERIAL_PROTOCOL_F(diff, 5);
            } // xx
            SERIAL_EOL;
          } // yy
          SERIAL_EOL;
        }
      } //do_topography_map
    #endif //!DELTA
  #endif // AUTO_BED_LEVELING_GRID

  #if DISABLED(DELTA)
    if (verbose_level > 0)
      planner.bed_level_matrix.debug("\n\nBed Level Correction Matrix:");

    if (!dryrun) {
      /**
       * Correct the Z height difference from Z probe position and nozzle tip position.
       * The Z height on homing is measured by Z probe, but the Z probe is quite far
       * from the nozzle. When the bed is uneven, this height must be corrected.
       */
      float x_tmp = current_position[X_AXIS] + X_PROBE_OFFSET_FROM_EXTRUDER,
            y_tmp = current_position[Y_AXIS] + Y_PROBE_OFFSET_FROM_EXTRUDER,
            z_tmp = current_position[Z_AXIS],
            stepper_z = stepper.get_axis_position_mm(Z_AXIS);  //get the real Z (since planner.adjusted_position is now correcting the plane)

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_ECHOPAIR("> BEFORE apply_rotation_xyz > stepper_z = ", stepper_z);
          SERIAL_ECHOPAIR(" ... z_tmp  = ", z_tmp);
          SERIAL_EOL;
        }
      #endif

      // Apply the correction sending the Z probe offset
      apply_rotation_xyz(planner.bed_level_matrix, x_tmp, y_tmp, z_tmp);

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) {
          SERIAL_ECHOPAIR("> AFTER apply_rotation_xyz > z_tmp  = ", z_tmp);
          SERIAL_EOL;
        }
      #endif

      // Adjust the current Z and send it to the planner.
      current_position[Z_AXIS] += z_tmp - stepper_z;
      SYNC_PLAN_POSITION_KINEMATIC();

      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> corrected Z in G29", current_position);
      #endif
    }
  #endif // !DELTA

  #ifdef Z_PROBE_END_SCRIPT
    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) {
        SERIAL_ECHOPGM("Z Probe End Script: ");
        SERIAL_ECHOLNPGM(Z_PROBE_END_SCRIPT);
      }
    #endif
    enqueue_and_echo_commands_P(PSTR(Z_PROBE_END_SCRIPT));
    stepper.synchronize();
  #endif

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< gcode_G29");
  #endif

  bed_leveling_in_progress = false;

  report_current_position();

  KEEPALIVE_STATE(IN_HANDLER);
}
