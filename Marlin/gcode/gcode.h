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

bool code_seen(char code) {
  seen_pointer = strchr(current_command_args, code);
  return (seen_pointer != NULL); // Return TRUE if the code-letter was found
}

inline float code_value_float() {
  float ret;
  char* e = strchr(seen_pointer, 'E');
  if (e) {
    *e = 0;
    ret = strtod(seen_pointer + 1, NULL);
    *e = 'E';
  }
  else
    ret = strtod(seen_pointer + 1, NULL);
  return ret;
}

inline unsigned long code_value_ulong() { return strtoul(seen_pointer + 1, NULL, 10); }

inline long code_value_long() { return strtol(seen_pointer + 1, NULL, 10); }

inline int code_value_int() { return (int)strtol(seen_pointer + 1, NULL, 10); }

inline uint16_t code_value_ushort() { return (uint16_t)strtoul(seen_pointer + 1, NULL, 10); }

inline uint8_t code_value_byte() { return (uint8_t)(constrain(strtol(seen_pointer + 1, NULL, 10), 0, 255)); }

inline bool code_value_bool() { return code_value_byte() > 0; }

#if ENABLED(INCH_MODE_SUPPORT)

  inline float axis_unit_factor(int axis) {
    return (axis == E_AXIS && volumetric_enabled ? volumetric_unit_factor : linear_unit_factor);
  }

  inline float code_value_linear_units() { return code_value_float() * linear_unit_factor; }
  inline float code_value_axis_units(int axis) { return code_value_float() * axis_unit_factor(axis); }
  inline float code_value_per_axis_unit(int axis) { return code_value_float() / axis_unit_factor(axis); }

#else

  inline float code_value_linear_units() { return code_value_float(); }
  inline float code_value_axis_units(int axis) { UNUSED(axis); return code_value_float(); }
  inline float code_value_per_axis_unit(int axis) { UNUSED(axis); return code_value_float(); }

#endif

#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  inline void set_input_temp_units(TempUnit units) { input_temp_units = units; }

  float code_value_temp_abs() {
    switch (input_temp_units) {
      case TEMPUNIT_C:
        return code_value_float();
      case TEMPUNIT_F:
        return (code_value_float() - 32) * 0.5555555556;
      case TEMPUNIT_K:
        return code_value_float() - 272.15;
      default:
        return code_value_float();
    }
  }

  float code_value_temp_diff() {
    switch (input_temp_units) {
      case TEMPUNIT_C:
      case TEMPUNIT_K:
        return code_value_float();
      case TEMPUNIT_F:
        return code_value_float() * 0.5555555556;
      default:
        return code_value_float();
    }
  }
#else
  float code_value_temp_abs() { return code_value_float(); }
  float code_value_temp_diff() { return code_value_float(); }
#endif

FORCE_INLINE millis_t code_value_millis() { return code_value_ulong(); }
inline millis_t code_value_millis_from_seconds() { return code_value_float() * 1000; }


void gcode_line_error(const char* err, bool doFlush = true) {
  SERIAL_ERROR_START;
  serialprintPGM(err);
  SERIAL_ERRORLN(gcode_LastN);
  //Serial.println(gcode_N);
  if (doFlush) FlushSerialRequestResend();
  serial_count = 0;
}

inline void get_serial_commands() {
  static char serial_line_buffer[MAX_CMD_SIZE];
  static boolean serial_comment_mode = false;

  // If the command buffer is empty for too long,
  // send "wait" to indicate Marlin is still waiting.
  #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
    static millis_t last_command_time = 0;
    millis_t ms = millis();
    if (commands_in_queue == 0 && !MYSERIAL.available() && ELAPSED(ms, last_command_time + NO_TIMEOUTS)) {
      SERIAL_ECHOLNPGM(MSG_WAIT);
      last_command_time = ms;
    }
  #endif

  /**
   * Loop while serial characters are incoming and the queue is not full
   */
  while (commands_in_queue < BUFSIZE && MYSERIAL.available() > 0) {

    char serial_char = MYSERIAL.read();

    /**
     * If the character ends the line
     */
    if (serial_char == '\n' || serial_char == '\r') {

      serial_comment_mode = false; // end of line == end of comment

      if (!serial_count) continue; // skip empty lines

      serial_line_buffer[serial_count] = 0; // terminate string
      serial_count = 0; //reset buffer

      char* command = serial_line_buffer;

      while (*command == ' ') command++; // skip any leading spaces
      char* npos = (*command == 'N') ? command : NULL; // Require the N parameter to start the line
      char* apos = strchr(command, '*');

      if (npos) {

        boolean M110 = strstr_P(command, PSTR("M110")) != NULL;

        if (M110) {
          char* n2pos = strchr(command + 4, 'N');
          if (n2pos) npos = n2pos;
        }

        gcode_N = strtol(npos + 1, NULL, 10);

        if (gcode_N != gcode_LastN + 1 && !M110) {
          gcode_line_error(PSTR(MSG_ERR_LINE_NO));
          return;
        }

        if (apos) {
          byte checksum = 0, count = 0;
          while (command[count] != '*') checksum ^= command[count++];

          if (strtol(apos + 1, NULL, 10) != checksum) {
            gcode_line_error(PSTR(MSG_ERR_CHECKSUM_MISMATCH));
            return;
          }
          // if no errors, continue parsing
        }
        else {
          gcode_line_error(PSTR(MSG_ERR_NO_CHECKSUM));
          return;
        }

        gcode_LastN = gcode_N;
        // if no errors, continue parsing
      }
      else if (apos) { // No '*' without 'N'
        gcode_line_error(PSTR(MSG_ERR_NO_LINENUMBER_WITH_CHECKSUM), false);
        return;
      }

      // Movement commands alert when stopped
      if (IsStopped()) {
        char* gpos = strchr(command, 'G');
        if (gpos) {
          int codenum = strtol(gpos + 1, NULL, 10);
          switch (codenum) {
            case 0:
            case 1:
            case 2:
            case 3:
              SERIAL_ERRORLNPGM(MSG_ERR_STOPPED);
              LCD_MESSAGEPGM(MSG_STOPPED);
              break;
          }
        }
      }

      #if DISABLED(EMERGENCY_PARSER)
        // If command was e-stop process now
        if (strcmp(command, "M108") == 0) wait_for_heatup = false;
        if (strcmp(command, "M112") == 0) kill(PSTR(MSG_KILLED));
        if (strcmp(command, "M410") == 0) { quickstop_stepper(); }
      #endif

      #if defined(NO_TIMEOUTS) && NO_TIMEOUTS > 0
        last_command_time = ms;
      #endif

      // Add the command to the queue
      _enqueuecommand(serial_line_buffer, true);
    }
    else if (serial_count >= MAX_CMD_SIZE - 1) {
      // Keep fetching, but ignore normal characters beyond the max length
      // The command will be injected when EOL is reached
    }
    else if (serial_char == '\\') {  // Handle escapes
      if (MYSERIAL.available() > 0) {
        // if we have one more character, copy it over
        serial_char = MYSERIAL.read();
        if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
      }
      // otherwise do nothing
    }
    else { // it's not a newline, carriage return or escape char
      if (serial_char == ';') serial_comment_mode = true;
      if (!serial_comment_mode) serial_line_buffer[serial_count++] = serial_char;
    }

  } // queue has space, serial has data
}

#if ENABLED(SDSUPPORT)

  inline void get_sdcard_commands() {
    static bool stop_buffering = false,
                sd_comment_mode = false;

    if (!card.sdprinting) return;

    /**
     * '#' stops reading from SD to the buffer prematurely, so procedural
     * macro calls are possible. If it occurs, stop_buffering is triggered
     * and the buffer is run dry; this character _can_ occur in serial com
     * due to checksums, however, no checksums are used in SD printing.
     */

    if (commands_in_queue == 0) stop_buffering = false;

    uint16_t sd_count = 0;
    bool card_eof = card.eof();
    while (commands_in_queue < BUFSIZE && !card_eof && !stop_buffering) {
      int16_t n = card.get();
      char sd_char = (char)n;
      card_eof = card.eof();
      if (card_eof || n == -1
          || sd_char == '\n' || sd_char == '\r'
          || ((sd_char == '#' || sd_char == ':') && !sd_comment_mode)
      ) {
        if (card_eof) {
          SERIAL_PROTOCOLLNPGM(MSG_FILE_PRINTED);
          card.printingHasFinished();
          card.checkautostart(true);
        }
        else if (n == -1) {
          SERIAL_ERROR_START;
          SERIAL_ECHOLNPGM(MSG_SD_ERR_READ);
        }
        if (sd_char == '#') stop_buffering = true;

        sd_comment_mode = false; //for new command

        if (!sd_count) continue; //skip empty lines

        command_queue[cmd_queue_index_w][sd_count] = '\0'; //terminate string
        sd_count = 0; //clear buffer

        _commit_command(false);
      }
      else if (sd_count >= MAX_CMD_SIZE - 1) {
        /**
         * Keep fetching, but ignore normal characters beyond the max length
         * The command will be injected when EOL is reached
         */
      }
      else {
        if (sd_char == ';') sd_comment_mode = true;
        if (!sd_comment_mode) command_queue[cmd_queue_index_w][sd_count++] = sd_char;
      }
    }
  }

#endif // SDSUPPORT

/**
 * Add to the circular command queue the next command from:
 *  - The command-injection queue (queued_commands_P)
 *  - The active serial input (usually USB)
 *  - The SD card file being actively printed
 */
void get_available_commands() {

  // if any immediate commands remain, don't get other commands yet
  if (drain_queued_commands_P()) return;

  get_serial_commands();

  #if ENABLED(SDSUPPORT)
    get_sdcard_commands();
  #endif
}


/**
 * Set target_extruder from the T parameter or the active_extruder
 *
 * Returns TRUE if the target is invalid
 */
bool get_target_extruder_from_command(int code) {
  if (code_seen('T')) {
    if (code_value_byte() >= EXTRUDERS) {
      SERIAL_ECHO_START;
      SERIAL_CHAR('M');
      SERIAL_ECHO(code);
      SERIAL_ECHOPAIR(" " MSG_INVALID_EXTRUDER " ", code_value_byte());
      SERIAL_EOL;
      return true;
    }
    target_extruder = code_value_byte();
  }
  else
    target_extruder = active_extruder;

  return false;
}

/**
 * Set XYZE destination and feedrate from the current GCode command
 *
 *  - Set destination from included axis codes
 *  - Set to current for missing axis codes
 *  - Set the feedrate, if included
 */
void gcode_get_destination() {
  LOOP_XYZE(i) {
    if (code_seen(axis_codes[i]))
      destination[i] = code_value_axis_units(i) + (axis_relative_modes[i] || relative_mode ? current_position[i] : 0);
    else
      destination[i] = current_position[i];
  }

  if (code_seen('F') && code_value_linear_units() > 0.0)
    feedrate_mm_m = code_value_linear_units();

  #if ENABLED(PRINTCOUNTER)
    if (!DEBUGGING(DRYRUN))
      print_job_timer.incFilamentUsed(destination[E_AXIS] - current_position[E_AXIS]);
  #endif

  // Get ABCDHI mixing factors
  #if ENABLED(MIXING_EXTRUDER) && ENABLED(DIRECT_MIXING_IN_G1)
    gcode_get_mix();
  #endif
}

/**
 * Command Handlers by Category
 */

                                  //
                                  // MOTION
                                  //

#include "motion/G0_G1.h"         // Linear Move
#include "motion/G2_G3.h"         // Arc Move
#include "motion/G4.h"            // Dwell
#include "motion/G5.h"            // Cubic B-spline
#include "motion/G10_G11.h"       // Retract / Recover

                                  //
                                  // UNITS
                                  //

#include "units/G20-G21.h"        // Set linear units
#include "units/M82.h"            // Set E codes absolute (default)
#include "units/M83.h"            // Set E codes relative while in Absolute Coordinates (G90) mode
#include "units/M92.h"            // Set axis steps-per-unit for one or more axes, X, Y, Z, and E.
#if ENABLED(TEMPERATURE_UNITS_SUPPORT)
  #include "units/M149.h"         // Set temperature units
#endif

                                  //
                                  // FEATURE
                                  //

#include "feature/park/G27.h"     // Park Nozzle
#if ENABLED(NOZZLE_CLEAN_FEATURE)
  #include "feature/clean/G12.h"  // Clean Nozzle
#endif
#if ENABLED(FILAMENT_CHANGE_FEATURE)
  #include "feature/change/M600.h" // Change Filament
#endif

                                  //
                                  // CALIBRATE
                                  //

#include "calibrate/G28.h"        // Home Axes
#if ENABLED(MESH_BED_LEVELING)
  #include "calibrate/G29-MBL.h"  // Manual Bed Probe (Mesh)
#elif ENABLED(AUTO_BED_LEVELING_FEATURE)
  #include "calibrate/G29.h"      // Automatic Bed Probe (Various)
#endif
#if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
  #include "calibrate/M48.h"      // Z Probe Repeatability Test
#endif
#if ENABLED(M100_FREE_MEMORY_WATCHER)
  #include "calibrate/M100.h"     // Free Memory Watcher
#endif
#include "calibrate/M303.h"       // PID relay autotune
#if ENABLED(MESH_BED_LEVELING)
  #include "calibrate/M420.h"     // Enable/Disable Mesh Bed Leveling
  #include "calibrate/M421.h"     // Set a single Mesh Bed Leveling Z coordinate
#endif

                                  //
                                  // PROBE
                                  //
#if HAS_BED_PROBE
  #include "probe/G30.h"          // Single Probe at XY
  #include "probe/G31-G32.h"      // Deploy / Stow the Z probe
  #include "probe/M401-M402.h"    // Deploy / Stop servo probe
  #include "probe/M851.h"         // Set Probe Z Offset
#endif

                                  //
                                  // GEOMETRY
                                  //

#include "geometry/G92.h"         // Set current position
#include "geometry/M206.h"        // Set Additional Homing Offset (X Y Z). SCARA aliases T=X, P=Y
#include "geometry/M428.h"        // Set home_offset relative to current position

                                  //
                                  // LCD
                                  //
#if ENABLED(ULTIPANEL)
  #include "lcd/M0_M1.h"          // Wait for User (with optional message)
  #include "lcd/M145.h"           // Set the heatup state for a material in the LCD menu
#endif
#include "lcd/M117.h"             // Set LCD Status Message
#if HAS_BUZZER
  #include "lcd/M300.h"           // Play a tone
#endif
#if HAS_LCD_CONTRAST
  #include "lcd/M250.h"           // Get/Set the LCD contrast
#endif

                                  //
                                  // CONTROL
                                  //

#include "control/M17.h"          // Enable power on all stepper motors
#include "control/M42.h"          // Change pin status via GCode
#if DISABLED(EMERGENCY_PARSER)
  #include "control/M112.h"       // Emergency Stop
  #include "control/M410.h"       // Quickstop - Abort all planned moves
#endif
#include "control/M111.h"         // Set the debug level
#if HAS_POWER_SWITCH
  #include "control/M80.h"        // Turn on Power Supply
#endif
#include "control/M81.h"          // Turn off power, heaters, etc.
#include "control/M18_M84.h"      // Disable all stepper motors
#include "control/M85.h"          // Set inactivity shutdown timer
#include "control/M120-M121.h"    // Enable / Disable endstops globally
#include "control/M226.h"         // Wait for pin to reach a state
#if HAS_SERVOS
  #include "control/M280.h"       // Get / Set servo position
#endif
#include "control/M400.h"         // Finish all moves
#if ENABLED(DUAL_X_CARRIAGE) || ENABLED(DUAL_NOZZLE_DUPLICATION_MODE)
  #include "control/M605.h"       // Set mode for dual nozzle setups
#endif
#if HAS_MICROSTEPS
  #include "control/M350.h"       // Set microstepping mode.
  #include "control/M351.h"       // Toggle microstepping pins directly
#endif
#include "control/M999.h"         // Restart after being stopped
#include "control/T.h"            // Tool Change

                                  //
                                  // SDCARD
                                  //
#if ENABLED(SDSUPPORT)
  #include "sdcard/M20.h"         // List SD card to serial output
  #include "sdcard/M21.h"         // Init SD Card
  #include "sdcard/M22.h"         // Release SD Card
  #include "sdcard/M23.h"         // Open a file
  #include "sdcard/M24.h"         // Start SD Print
  #include "sdcard/M25.h"         // Pause SD Print
  #include "sdcard/M26.h"         // Set SD Card file index
  #include "sdcard/M27.h"         // Get SD Card status
  #include "sdcard/M28.h"         // Start SD Write
  #include "sdcard/M29.h"         // Stop SD Write
  #include "sdcard/M30.h"         // Delete SD Card file
  #include "sdcard/M32.h"         // Select file and start SD Print
  #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
    #include "sdcard/M33.h"       // Get the long full path of a file or folder
  #endif
  #include "sdcard/M928.h"        // Start SD Write
#endif

                                  //
                                  // TIMER
                                  //

#include "stats/M31.h"            // Get the time since the start of SD Print (or last M109)
#include "stats/M75.h"            // Start print timer
#include "stats/M76.h"            // Pause print timer
#include "stats/M77.h"            // Stop print timer
#if ENABLED(PRINTCOUNTER)
  #include "stats/M78.h"          // Show print statistics
#endif

                                  //
                                  // TEMPERATURE
                                  //

#include "temperature/M104.h"     // Set hot end temperature
#include "temperature/M105.h"     // Read hot end and bed temperature
#if FAN_COUNT
  #include "temperature/M106.h"   // Set Fan Speed
  #include "temperature/M107.h"   // Fan Off
#endif
#if DISABLED(EMERGENCY_PARSER)
  #include "temperature/M108.h"   // Stop waiting for heaters
#endif
#include "temperature/M109.h"     // Wait for extruder to reach target temperature
#include "temperature/M140.h"     // Set bed temperature
#if HAS_TEMP_BED
  #include "temperature/M190.h"   // Wait for bed to reach target temperature
#endif

                                  //
                                  // HOST
                                  //

#include "host/M110.h"            // Set Current Line Number
#if ENABLED(HOST_KEEPALIVE_FEATURE)
  #include "host/M113.h"          // Get/Set Host Keepalive interval (0 to disable)
#endif
#include "host/M114.h"            // Report current position
#include "host/M115.h"            // Capabilities string
#include "host/M119.h"            // Output endstop states to serial output

                                  //
                                  // BARICUDA
                                  //
#if ENABLED(BARICUDA)
  #if HAS_HEATER_1
    #include "feature/baricuda/M126.h" // Heater 1 valve open
    #include "feature/baricuda/M127.h" // Heater 1 valve close
  #endif
  #if HAS_HEATER_2
    #include "feature/baricuda/M128.h" // Heater 2 valve open
    #include "feature/baricuda/M129.h" // Heater 2 valve close
  #endif
#endif
                                  //
                                  // BLINKM
                                  //
#if ENABLED(BLINKM)
  #include "feature/blinkm/M150.h" // Set Status LED color
#endif
                                  //
                                  // I2C
                                  //
#if ENABLED(EXPERIMENTAL_I2CBUS)
  #include "feature/i2c/M155.h"   // Send data to a I2C slave device
  #include "feature/i2c/M156.h"   // Request X bytes from I2C slave device
#endif

                                  //
                                  // CONFIG
                                  //

#include "config/M200.h"          // Set filament diameter and set E axis units to cubic units
#include "config/M201.h"          // Set max acceleration in units/s^2 for print moves (M201 X1000 Y1000)
#include "config/M203.h"          // Set Maximum Feedrate
#include "config/M204.h"          // Set Accelerations in units/sec^2
#include "config/M205.h"          // Set Advanced Settings
#if ENABLED(Z_DUAL_ENDSTOPS)
  #include "config/M666.h"        // For Z Dual Endstop setup, set z axis offset to the z2 axis.
#endif
#if ENABLED(FWRETRACT)
  #include "config/M207.h"        // Set firmware retraction values
  #include "config/M208.h"        // Set firmware un-retraction values
  #include "config/M209.h"        // Enable/Disable Auto-Retract
#endif
#if HOTENDS > 1
  #include "config/M218.h"        // Set hotend offset (in linear units)
#endif
#include "config/M220.h"          // Set speed percentage factor, aka "Feed Rate"
#include "config/M221.h"          // Set extrusion percentage
#if ENABLED(PIDTEMP)
  #include "config/M301.h"        // Set PID (CL) for hotend
#endif
#if ENABLED(PIDTEMPBED)
  #include "config/M304.h"        // Set PID for bed
#endif
#if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
  #include "config/M302.h"        // Get/Set/Allow cold extrude
#endif
#if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
  #include "config/M540.h"        // Enable/Disable SD Abort on Endstop
#endif
#if ENABLED(LIN_ADVANCE)
  #include "config/M905.h"        // Set advance factor
#endif

                                  //
                                  // DELTA
                                  //
#if ENABLED(DELTA)
  #include "delta/M665.h"         // Set delta configurations
  #include "delta/M666.h"         // Set delta endstop adjustment
#endif

                                  //
                                  // PHOTO
                                  //

#if defined(CHDK) || HAS_PHOTOGRAPH
  #include "feature/photo/M240.h" // Trigger a camera
#endif

#if ENABLED(SCARA)
  #include "scara/M360-M365.h"    // SCARA calibration
#endif

#if ENABLED(FILAMENT_WIDTH_SENSOR)
  #include "sensor/M404.h"        // Display or set (in current units) the nominal filament width (3mm, 1.75mm ) W<3.0>
  #include "sensor/M405.h"        // Enable Filament Sensor
  #include "sensor/M406.h"        // Disable Filament Sensor
  #include "sensor/M407.h"        // Get measured filament diameter on serial output
#endif

#include "eeprom/M500.h"          // Save settings to EEPROM
#include "eeprom/M501.h"          // Load settings from EEPROM
#include "eeprom/M502.h"          // Reset settings to default (in RAM)
#include "eeprom/M503.h"          // Print current settings (in RAM)

#include "trimpot/M907.h"         // Set digital trimpot motor current
#if HAS_DIGIPOTSS || ENABLED(DAC_STEPPER_CURRENT)
  #include "trimpot/M908.h"       // Control digital trimpot directly
  #if ENABLED(DAC_STEPPER_CURRENT)
    #include "trimpot/M909.h"     // Print digipot/DAC current value
    #include "trimpot/M910.h"     // Commit digipot/DAC value to external EEPROM via I2C
  #endif
#endif

#if ENABLED(MIXING_EXTRUDER)
  #include "mixing/M163.h"        // Set a single mix factor for a mixing extruder
  #if MIXING_VIRTUAL_TOOLS > 1
    #include "mixing/M164.h"      // Store the current mix factors as a virtual tool.
  #endif
  #if ENABLED(DIRECT_MIXING_IN_G1)
    #include "mixing/M165.h"      // Set multiple mix factors for a mixing extruder.
  #endif
#endif

/**
 * Command Processor
 */
#include "process_next_command.h"
