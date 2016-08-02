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

#ifndef MIN_COOLING_SLOPE_DEG_BED
  #define MIN_COOLING_SLOPE_DEG_BED 1.50
#endif
#ifndef MIN_COOLING_SLOPE_TIME_BED
  #define MIN_COOLING_SLOPE_TIME_BED 60
#endif

/**
 * M190: Sxxx Wait for bed current temp to reach target temp. Waits only when heating
 *       Rxxx Wait for bed current temp to reach target temp. Waits when heating and cooling
 */
inline void gcode_M190() {
  if (DEBUGGING(DRYRUN)) return;

  LCD_MESSAGEPGM(MSG_BED_HEATING);
  bool no_wait_for_cooling = code_seen('S');
  if (no_wait_for_cooling || code_seen('R')) {
    thermalManager.setTargetBed(code_value_temp_abs());
    #if ENABLED(PRINTJOB_TIMER_AUTOSTART)
      if (code_value_temp_abs() > BED_MINTEMP) {
        /**
        * We start the timer when 'heating and waiting' command arrives, LCD
        * functions never wait. Cooling down managed by extruders.
        *
        * We do not check if the timer is already running because this check will
        * be done for us inside the Stopwatch::start() method thus a running timer
        * will not restart.
        */
        print_job_timer.start();
      }
    #endif
  }

  #if TEMP_BED_RESIDENCY_TIME > 0
    millis_t residency_start_ms = 0;
    // Loop until the temperature has stabilized
    #define TEMP_BED_CONDITIONS (!residency_start_ms || PENDING(now, residency_start_ms + (TEMP_BED_RESIDENCY_TIME) * 1000UL))
  #else
    // Loop until the temperature is very close target
    #define TEMP_BED_CONDITIONS (wants_to_cool ? thermalManager.isCoolingBed() : thermalManager.isHeatingBed())
  #endif //TEMP_BED_RESIDENCY_TIME > 0

  float theTarget = -1.0, old_temp = 9999.0;
  bool wants_to_cool = false;
  wait_for_heatup = true;
  millis_t now, next_temp_ms = 0, next_cool_check_ms = 0;

  KEEPALIVE_STATE(NOT_BUSY);

  target_extruder = active_extruder; // for print_heaterstates

  do {
    // Target temperature might be changed during the loop
    if (theTarget != thermalManager.degTargetBed()) {
      wants_to_cool = thermalManager.isCoolingBed();
      theTarget = thermalManager.degTargetBed();

      // Exit if S<lower>, continue if S<higher>, R<lower>, or R<higher>
      if (no_wait_for_cooling && wants_to_cool) break;
    }

    now = millis();
    if (ELAPSED(now, next_temp_ms)) { //Print Temp Reading every 1 second while heating up.
      next_temp_ms = now + 1000UL;
      print_heaterstates();
      #if TEMP_BED_RESIDENCY_TIME > 0
        SERIAL_PROTOCOLPGM(" W:");
        if (residency_start_ms) {
          long rem = (((TEMP_BED_RESIDENCY_TIME) * 1000UL) - (now - residency_start_ms)) / 1000UL;
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

    float temp = thermalManager.degBed();

    #if TEMP_BED_RESIDENCY_TIME > 0

      float temp_diff = fabs(theTarget - temp);

      if (!residency_start_ms) {
        // Start the TEMP_BED_RESIDENCY_TIME timer when we reach target temp for the first time.
        if (temp_diff < TEMP_BED_WINDOW) residency_start_ms = now;
      }
      else if (temp_diff > TEMP_BED_HYSTERESIS) {
        // Restart the timer whenever the temperature falls outside the hysteresis.
        residency_start_ms = now;
      }

    #endif //TEMP_BED_RESIDENCY_TIME > 0

    // Prevent a wait-forever situation if R is misused i.e. M190 R0
    if (wants_to_cool) {
      // break after MIN_COOLING_SLOPE_TIME_BED seconds
      // if the temperature did not drop at least MIN_COOLING_SLOPE_DEG_BED
      if (!next_cool_check_ms || ELAPSED(now, next_cool_check_ms)) {
        if (old_temp - temp < MIN_COOLING_SLOPE_DEG_BED) break;
        next_cool_check_ms = now + 1000UL * MIN_COOLING_SLOPE_TIME_BED;
        old_temp = temp;
      }
    }

  } while (wait_for_heatup && TEMP_BED_CONDITIONS);

  LCD_MESSAGEPGM(MSG_BED_DONE);
  KEEPALIVE_STATE(IN_HANDLER);
}
