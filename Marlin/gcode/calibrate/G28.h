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

#if ENABLED(QUICK_HOME)

  static void quick_home_xy() {

    // Pretend the current position is 0,0
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;
    sync_plan_position();

    int x_axis_home_dir =
      #if ENABLED(DUAL_X_CARRIAGE)
        x_home_dir(active_extruder)
      #else
        home_dir(X_AXIS)
      #endif
    ;

    float mlx = max_length(X_AXIS),
          mly = max_length(Y_AXIS),
          mlratio = mlx > mly ? mly / mlx : mlx / mly,
          fr_mm_m = min(homing_feedrate_mm_m[X_AXIS], homing_feedrate_mm_m[Y_AXIS]) * sqrt(sq(mlratio) + 1.0);

    do_blocking_move_to_xy(1.5 * mlx * x_axis_home_dir, 1.5 * mly * home_dir(Y_AXIS), fr_mm_m);
    endstops.hit_on_purpose(); // clear endstop hit flags
    current_position[X_AXIS] = current_position[Y_AXIS] = 0.0;

  }

#endif // QUICK_HOME

/**
 * G28: Home all axes according to settings
 *
 * Parameters
 *
 *  None  Home to all axes with no parameters.
 *        With QUICK_HOME enabled XY will home together, then Z.
 *
 * Cartesian parameters
 *
 *  X   Home to the X endstop
 *  Y   Home to the Y endstop
 *  Z   Home to the Z endstop
 *
 */
inline void gcode_G28() {

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM(">>> gcode_G28");
  #endif

  // Wait for planner moves to finish!
  stepper.synchronize();

  // For auto bed leveling, clear the level matrix
  #if ENABLED(AUTO_BED_LEVELING_FEATURE)
    planner.bed_level_matrix.set_to_identity();
    #if ENABLED(DELTA)
      reset_bed_level();
    #endif
  #endif

  // Always home with tool 0 active
  #if HOTENDS > 1
    uint8_t old_tool_index = active_extruder;
    tool_change(0, 0, true);
  #endif

  #if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
    extruder_duplication_enabled = false;
  #endif

  /**
   * For mesh bed leveling deactivate the mesh calculations, will be turned
   * on again when homing all axis
   */
  #if ENABLED(MESH_BED_LEVELING)
    float pre_home_z = MESH_HOME_SEARCH_Z;
    if (mbl.active()) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("MBL was active");
      #endif
      // Save known Z position if already homed
      if (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) {
        pre_home_z = current_position[Z_AXIS];
        pre_home_z += mbl.get_z(RAW_CURRENT_POSITION(X_AXIS), RAW_CURRENT_POSITION(Y_AXIS));
      }
      mbl.set_active(false);
      current_position[Z_AXIS] = pre_home_z;
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("Set Z to pre_home_z", current_position);
      #endif
    }
  #endif

  setup_for_endstop_or_probe_move();
  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> endstops.enable(true)");
  #endif
  endstops.enable(true); // Enable endstops for next homing move


  #if ENABLED(DELTA)
    /**
     * A delta can only safely home all axes at the same time
     */

    // Pretend the current position is 0,0,0
    // This is like quick_home_xy() but for 3 towers.
    current_position[X_AXIS] = current_position[Y_AXIS] = current_position[Z_AXIS] = 0.0;
    sync_plan_position();

    // Move all carriages up together until the first endstop is hit.
    current_position[X_AXIS] = current_position[Y_AXIS] = current_position[Z_AXIS] = 3.0 * (Z_MAX_LENGTH);
    feedrate_mm_m = 1.732 * homing_feedrate_mm_m[X_AXIS];
    line_to_current_position();
    stepper.synchronize();
    endstops.hit_on_purpose(); // clear endstop hit flags
    current_position[X_AXIS] = current_position[Y_AXIS] = current_position[Z_AXIS] = 0.0;

    // take care of back off and rehome. Now one carriage is at the top.
    HOMEAXIS(X);
    HOMEAXIS(Y);
    HOMEAXIS(Z);

    SYNC_PLAN_POSITION_KINEMATIC();

    #if ENABLED(DEBUG_LEVELING_FEATURE)
      if (DEBUGGING(LEVELING)) DEBUG_POS("(DELTA)", current_position);
    #endif

  #else // NOT DELTA

    bool homeX = code_seen('X'), homeY = code_seen('Y'), homeZ = code_seen('Z');

    home_all_axis = (!homeX && !homeY && !homeZ) || (homeX && homeY && homeZ);

    set_destination_to_current();

    #if Z_HOME_DIR > 0  // If homing away from BED do Z first

      if (home_all_axis || homeZ) {
        HOMEAXIS(Z);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> HOMEAXIS(Z)", current_position);
        #endif
      }

    #else

      if (home_all_axis || homeX || homeY) {
        // Raise Z before homing any other axes and z is not already high enough (never lower z)
        destination[Z_AXIS] = LOGICAL_Z_POSITION(MIN_Z_HEIGHT_FOR_HOMING);
        if (destination[Z_AXIS] > current_position[Z_AXIS]) {

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOPAIR("Raise Z (before homing) to ", destination[Z_AXIS]);
              SERIAL_EOL;
            }
          #endif

          do_blocking_move_to_z(destination[Z_AXIS]);
        }
      }

    #endif

    #if ENABLED(QUICK_HOME)

      if (home_all_axis || (homeX && homeY)) quick_home_xy();

    #endif

    #if ENABLED(HOME_Y_BEFORE_X)

      // Home Y
      if (home_all_axis || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }

    #endif

    // Home X
    if (home_all_axis || homeX) {
      #if ENABLED(DUAL_X_CARRIAGE)
        int tmp_extruder = active_extruder;
        active_extruder = !active_extruder;
        HOMEAXIS(X);
        inactive_extruder_x_pos = RAW_X_POSITION(current_position[X_AXIS]);
        active_extruder = tmp_extruder;
        HOMEAXIS(X);
        // reset state used by the different modes
        memcpy(raised_parked_position, current_position, sizeof(raised_parked_position));
        delayed_move_time = 0;
        active_extruder_parked = true;
      #else
        HOMEAXIS(X);
      #endif
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) DEBUG_POS("> homeX", current_position);
      #endif
    }

    #if DISABLED(HOME_Y_BEFORE_X)
      // Home Y
      if (home_all_axis || homeY) {
        HOMEAXIS(Y);
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> homeY", current_position);
        #endif
      }
    #endif

    // Home Z last if homing towards the bed
    #if Z_HOME_DIR < 0

      if (home_all_axis || homeZ) {

        #if ENABLED(Z_SAFE_HOMING)

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOLNPGM("> Z_SAFE_HOMING >>>");
            }
          #endif

          if (home_all_axis) {

            /**
             * At this point we already have Z at MIN_Z_HEIGHT_FOR_HOMING height
             * No need to move Z any more as this height should already be safe
             * enough to reach Z_SAFE_HOMING XY positions.
             * Just make sure the planner is in sync.
             */
            SYNC_PLAN_POSITION_KINEMATIC();

            /**
             * Move the Z probe (or just the nozzle) to the safe homing point
             */
            destination[X_AXIS] = round(Z_SAFE_HOMING_X_POINT - (X_PROBE_OFFSET_FROM_EXTRUDER));
            destination[Y_AXIS] = round(Z_SAFE_HOMING_Y_POINT - (Y_PROBE_OFFSET_FROM_EXTRUDER));
            destination[Z_AXIS] = current_position[Z_AXIS]; // Z is already at the right height

            #if ENABLED(DEBUG_LEVELING_FEATURE)
              if (DEBUGGING(LEVELING)) {
                DEBUG_POS("> Z_SAFE_HOMING > home_all_axis", current_position);
                DEBUG_POS("> Z_SAFE_HOMING > home_all_axis", destination);
              }
            #endif

            // Move in the XY plane
            do_blocking_move_to_xy(destination[X_AXIS], destination[Y_AXIS]);
          }

          // Let's see if X and Y are homed
          if (axis_unhomed_error(true, true, false)) return;

          /**
           * Make sure the Z probe is within the physical limits
           * NOTE: This doesn't necessarily ensure the Z probe is also
           * within the bed!
           */
          float cpx = RAW_CURRENT_POSITION(X_AXIS), cpy = RAW_CURRENT_POSITION(Y_AXIS);
          if (   cpx >= X_MIN_POS - (X_PROBE_OFFSET_FROM_EXTRUDER)
              && cpx <= X_MAX_POS - (X_PROBE_OFFSET_FROM_EXTRUDER)
              && cpy >= Y_MIN_POS - (Y_PROBE_OFFSET_FROM_EXTRUDER)
              && cpy <= Y_MAX_POS - (Y_PROBE_OFFSET_FROM_EXTRUDER)) {

            // Home the Z axis
            HOMEAXIS(Z);
          }
          else {
            LCD_MESSAGEPGM(MSG_ZPROBE_OUT);
            SERIAL_ECHO_START;
            SERIAL_ECHOLNPGM(MSG_ZPROBE_OUT);
          }

          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) {
              SERIAL_ECHOLNPGM("<<< Z_SAFE_HOMING");
            }
          #endif

        #else // !Z_SAFE_HOMING

          HOMEAXIS(Z);

        #endif // !Z_SAFE_HOMING

        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("> (home_all_axis || homeZ) > final", current_position);
        #endif

      } // home_all_axis || homeZ

    #endif // Z_HOME_DIR < 0

    SYNC_PLAN_POSITION_KINEMATIC();

  #endif // !DELTA (gcode_G28)

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("> endstops.not_homing()");
  #endif
  endstops.not_homing();
  endstops.hit_on_purpose(); // clear endstop hit flags

  // Enable mesh leveling again
  #if ENABLED(MESH_BED_LEVELING)
    if (mbl.has_mesh()) {
      #if ENABLED(DEBUG_LEVELING_FEATURE)
        if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("MBL has mesh");
      #endif
      if (home_all_axis || (axis_homed[X_AXIS] && axis_homed[Y_AXIS] && homeZ)) {
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("MBL Z homing");
        #endif
        current_position[Z_AXIS] = MESH_HOME_SEARCH_Z
          #if Z_HOME_DIR > 0
            + Z_MAX_POS
          #endif
        ;
        SYNC_PLAN_POSITION_KINEMATIC();
        mbl.set_active(true);
        #if ENABLED(MESH_G28_REST_ORIGIN)
          current_position[Z_AXIS] = 0.0;
          set_destination_to_current();
          feedrate_mm_m = homing_feedrate_mm_m[Z_AXIS];
          line_to_destination();
          stepper.synchronize();
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("MBL Rest Origin", current_position);
          #endif
        #else
          current_position[Z_AXIS] = MESH_HOME_SEARCH_Z -
            mbl.get_z(RAW_CURRENT_POSITION(X_AXIS), RAW_CURRENT_POSITION(Y_AXIS))
            #if Z_HOME_DIR > 0
              + Z_MAX_POS
            #endif
          ;
          #if ENABLED(DEBUG_LEVELING_FEATURE)
            if (DEBUGGING(LEVELING)) DEBUG_POS("MBL adjusted MESH_HOME_SEARCH_Z", current_position);
          #endif
        #endif
      }
      else if ((axis_homed[X_AXIS] && axis_homed[Y_AXIS] && axis_homed[Z_AXIS]) && (homeX || homeY)) {
        current_position[Z_AXIS] = pre_home_z;
        SYNC_PLAN_POSITION_KINEMATIC();
        mbl.set_active(true);
        current_position[Z_AXIS] = pre_home_z -
          mbl.get_z(RAW_CURRENT_POSITION(X_AXIS), RAW_CURRENT_POSITION(Y_AXIS));
        #if ENABLED(DEBUG_LEVELING_FEATURE)
          if (DEBUGGING(LEVELING)) DEBUG_POS("MBL Home X or Y", current_position);
        #endif
      }
    }
  #endif

  #if ENABLED(DELTA)
    // move to a height where we can use the full xy-area
    do_blocking_move_to_z(delta_clip_start_height);
  #endif

  clean_up_after_endstop_or_probe_move();

  #if ENABLED(DEBUG_LEVELING_FEATURE)
    if (DEBUGGING(LEVELING)) SERIAL_ECHOLNPGM("<<< gcode_G28");
  #endif

  // Restore the active tool after homing
  #if HOTENDS > 1
    tool_change(old_tool_index, 0, true);
  #endif

  report_current_position();
}
