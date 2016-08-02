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

#ifndef MIN_COOLING_SLOPE_DEG
  #define MIN_COOLING_SLOPE_DEG 1.50
#endif
#ifndef MIN_COOLING_SLOPE_TIME
  #define MIN_COOLING_SLOPE_TIME 60
#endif

/**
 * M109: Sxxx Wait for extruder(s) to reach temperature. Waits only when heating.
 *       Rxxx Wait for extruder(s) to reach temperature. Waits when heating and cooling.
 */
inline void gcode_M109() {

  if (get_target_extruder_from_command(109)) return;
  if (DEBUGGING(DRYRUN)) return;

  #if ENABLED(SINGLENOZZLE)
    if (target_extruder != active_extruder) return;
  #endif

  bool no_wait_for_cooling = code_seen('S');
  if (no_wait_for_cooling || code_seen('R')) {
    thermalManager.setTargetHotend(code_value_temp_abs(), target_extruder);
    #if ENABLED(DUAL_X_CARRIAGE)
      if (dual_x_carriage_mode == DXC_DUPLICATION_MODE && target_extruder == 0)
        thermalManager.setTargetHotend(code_value_temp_abs() == 0.0 ? 0.0 : code_value_temp_abs() + duplicate_extruder_temp_offset, 1);
    #endif

    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      /**
       * We use half EXTRUDE_MINTEMP here to allow nozzles to be put into hot
       * stand by mode, for instance in a dual extruder setup, without affecting
       * the running print timer.
       */
      if (code_value_temp_abs() <= (EXTRUDE_MINTEMP)/2) {
        print_job_timer.stop();
        LCD_MESSAGEPGM(WELCOME_MSG);
      }
      /**
       * We do not check if the timer is already running because this check will
       * be done for us inside the Stopwatch::start() method thus a running timer
       * will not restart.
       */
      else print_job_timer.start();
    #endif

    if (thermalManager.isHeatingHotend(target_extruder)) LCD_MESSAGEPGM(MSG_HEATING);
  }

  #if ENABLED(AUTOTEMP)
    planner.autotemp_M109();
  #endif

  #if TEMP_RESIDENCY_TIME > 0
    millis_t residency_start_ms = 0;
    // Loop until the temperature has stabilized
    #define TEMP_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_RESIDENCY_TIME) * 1000UL))
  #else
    // Loop until the temperature is very close target
    #define TEMP_CONDITIONS (wants_to_cool ? thermalManager.isCoolingHotend(target_extruder) : thermalManager.isHeatingHotend(target_extruder))
  #endif //TEMP_RESIDENCY_TIME > 0

  float theTarget = -1.0, old_temp = 9999.0;
  bool wants_to_cool = false;
  wait_for_heatup = true;
  millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

  KEEPALIVE_STATE(NOT_BUSY);

  do {
    // Target temperature might be changed during the loop
    if (theTarget != thermalManager.degTargetHotend(target_extruder)) {
      wants_to_cool = thermalManager.isCoolingHotend(target_extruder);
      theTarget = thermalManager.degTargetHotend(target_extruder);

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;
    }

    now = millis();
    if (ELAPSED(now, next_temp_ms)) { //Print temp & remaining time every 1s while waiting
      next_temp_ms = now + 1000UL;
      print_heaterstates();
      #if TEMP_RESIDENCY_TIME > 0
        SERIAL_PROTOCOLPGM(" W:");
        if (residency_start_ms) {
          long rem = (((TEMP_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
          SERIAL_PROTOCOLLN(rem);
        }
        else {
          SERIAL_PROTOCOLLNPGM("?");
        }
      #else
        SERIAL_EOL;
      #endif
    }

    idle();
    refresh_cmd_timeout(); // to prevent stepper_inactive_time from running out

    float temp = thermalManager.degHotend(target_extruder);

    #if TEMP_RESIDENCY_TIME > 0

      float temp_diff = fabs(theTarget - temp);

      if (!residency_start_ms) {
        // Start the TEMP_RESIDENCY_TIME timer when we reach target temp for the first time.
        if (temp_diff < TEMP_WINDOW) residency_start_ms = now;
      }
      else if (temp_diff > TEMP_HYSTERESIS) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = now;
      }

    #endif //TEMP_RESIDENCY_TIME > 0

    // Prevent a wait-forever situation if R is misused i.e. M109 R0
    if (wants_to_cool) {
      // break after MIN_COOLING_SLOPE_TIME seconds
      // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG
      if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
        if (old_temp - temp < MIN_COOLING_SLOPE_DEG) break;
        next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME;
        old_temp = temp;
      }
    }

  } while (wait_for_heatup && TEMP_CONDITIONS);

  LCD_MESSAGEPGM(MSG_HEATING_COMPLETE);
  KEEPALIVE_STATE(IN_HANDLER);
}
