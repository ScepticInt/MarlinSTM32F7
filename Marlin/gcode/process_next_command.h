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
 * Process a single command and dispatch it to its handler
 * This is called from the main loop()
 */
void process_next_command() {
  current_command = command_queue[cmd_queue_index_r];

  if (DEBUGGING(ECHO)) {
    SERIAL_ECHO_START;
    SERIAL_ECHOLN(current_command);
  }

  // Sanitize the current command:
  //  - Skip leading spaces
  //  - Bypass N[-0-9][0-9]*[ ]*
  //  - Overwrite * with nul to mark the end
  while (*current_command == ' ') ++current_command;
  if (*current_command == 'N' && NUMERIC_SIGNED(current_command[1])) {
    current_command += 2; // skip N[-0-9]
    while (NUMERIC(*current_command)) ++current_command; // skip [0-9]*
    while (*current_command == ' ') ++current_command; // skip [ ]*
  }
  char* starpos = strchr(current_command, '*');  // * should always be the last parameter
  if (starpos) while (*starpos == ' ' || *starpos == '*') *starpos-- = '\0'; // nullify '*' and ' '

  char *cmd_ptr = current_command;

  // Get the command code, which must be G, M, or T
  char command_code = *cmd_ptr++;

  // Skip spaces to get the numeric part
  while (*cmd_ptr == ' ') cmd_ptr++;

  uint16_t codenum = 0; // define ahead of goto

  // Bail early if there's no code
  bool code_is_good = NUMERIC(*cmd_ptr);
  if (!code_is_good) goto ExitUnknownCommand;

  // Get and skip the code number
  do {
    codenum = (codenum * 10) + (*cmd_ptr - '0');
    cmd_ptr++;
  } while (NUMERIC(*cmd_ptr));

  // Skip all spaces to get to the first argument, or nul
  while (*cmd_ptr == ' ') cmd_ptr++;

  // The command's arguments (if any) start here, for sure!
  current_command_args = cmd_ptr;

  KEEPALIVE_STATE(IN_HANDLER);

  // Handle a known G, M, or T
  switch (command_code) {
    case 'G': switch (codenum) {

      // G0, G1
      case 0:
      case 1:
        gcode_G0_G1();
        break;

      // G2, G3
      #if ENABLED(ARC_SUPPORT) && DISABLED(SCARA)
        case 2: // G2  - CW ARC
        case 3: // G3  - CCW ARC
          gcode_G2_G3(codenum == 2);
          break;
      #endif

      // G4 Dwell
      case 4:
        gcode_G4();
        break;

      #if ENABLED(BEZIER_CURVE_SUPPORT)
        // G5
        case 5: // G5  - Cubic B_spline
          gcode_G5();
          break;
      #endif // BEZIER_CURVE_SUPPORT

      #if ENABLED(FWRETRACT)
        case 10: // G10: retract
        case 11: // G11: retract_recover
          gcode_G10_G11(codenum == 10);
          break;
      #endif // FWRETRACT

      #if ENABLED(NOZZLE_CLEAN_FEATURE)
        case 12:
          gcode_G12(); // G12: Nozzle Clean
          break;
      #endif // NOZZLE_CLEAN_FEATURE

      #if ENABLED(INCH_MODE_SUPPORT)
        case 20: //G20: Inch Mode
          gcode_G20();
          break;

        case 21: //G21: MM Mode
          gcode_G21();
          break;
      #endif // INCH_MODE_SUPPORT

      #if ENABLED(NOZZLE_PARK_FEATURE)
        case 27: // G27: Nozzle Park
          gcode_G27();
          break;
      #endif // NOZZLE_PARK_FEATURE

      case 28: // G28: Home all axes, one at a time
        gcode_G28();
        break;

      #if ENABLED(AUTO_BED_LEVELING_FEATURE) || ENABLED(MESH_BED_LEVELING)
        case 29: // G29 Detailed Z probe, probes the bed at 3 or more points.
          gcode_G29();
          break;
      #endif // AUTO_BED_LEVELING_FEATURE

      #if HAS_BED_PROBE

        case 30: // G30 Single Z probe
          gcode_G30();
          break;

        #if ENABLED(Z_PROBE_SLED)

            case 31: // G31: dock the sled
              gcode_G31();
              break;

            case 32: // G32: undock the sled
              gcode_G32();
              break;

        #endif // Z_PROBE_SLED
      #endif // HAS_BED_PROBE

      case 90: // G90
        relative_mode = false;
        break;
      case 91: // G91
        relative_mode = true;
        break;

      case 92: // G92
        gcode_G92();
        break;
    }
    break;

    case 'M': switch (codenum) {
      #if ENABLED(ULTIPANEL)
        case 0: // M0 - Unconditional stop - Wait for user button press on LCD
        case 1: // M1 - Conditional stop - Wait for user button press on LCD
          gcode_M0_M1();
          break;
      #endif // ULTIPANEL

      case 17:
        gcode_M17();
        break;

      #if ENABLED(SDSUPPORT)
        case 20: // M20 - list SD card
          gcode_M20(); break;
        case 21: // M21 - init SD card
          gcode_M21(); break;
        case 22: //M22 - release SD card
          gcode_M22(); break;
        case 23: //M23 - Select file
          gcode_M23(); break;
        case 24: //M24 - Start SD print
          gcode_M24(); break;
        case 25: //M25 - Pause SD print
          gcode_M25(); break;
        case 26: //M26 - Set SD index
          gcode_M26(); break;
        case 27: //M27 - Get SD status
          gcode_M27(); break;
        case 28: //M28 - Start SD write
          gcode_M28(); break;
        case 29: //M29 - Stop SD write
          gcode_M29(); break;
        case 30: //M30 <filename> Delete File
          gcode_M30(); break;
        case 32: //M32 - Select file and start SD print
          gcode_M32(); break;

        #if ENABLED(LONG_FILENAME_HOST_SUPPORT)
          case 33: //M33 - Get the long full path to a file or folder
            gcode_M33(); break;
        #endif // LONG_FILENAME_HOST_SUPPORT

        case 928: //M928 - Start SD write
          gcode_M928(); break;
      #endif //SDSUPPORT

      case 31: //M31 take time since the start of the SD print or an M109 command
        gcode_M31();
        break;

      case 42: //M42 -Change pin status via gcode
        gcode_M42();
        break;

      #if ENABLED(Z_MIN_PROBE_REPEATABILITY_TEST)
        case 48: // M48 Z probe repeatability
          gcode_M48();
          break;
      #endif // Z_MIN_PROBE_REPEATABILITY_TEST

      case 75: // Start print timer
        gcode_M75();
        break;

      case 76: // Pause print timer
        gcode_M76();
        break;

      case 77: // Stop print timer
        gcode_M77();
        break;

      #if ENABLED(PRINTCOUNTER)
        case 78: // Show print statistics
          gcode_M78();
          break;
      #endif

      #if ENABLED(M100_FREE_MEMORY_WATCHER)
        case 100:
          gcode_M100();
          break;
      #endif

      case 104: // M104
        gcode_M104();
        break;

      case 110: // M110: Set Current Line Number
        gcode_M110();
        break;

      case 111: // M111: Set debug level
        gcode_M111();
        break;

      #if DISABLED(EMERGENCY_PARSER)

        case 108: // M108: Cancel Waiting
          gcode_M108();
          break;

        case 112: // M112: Emergency Stop
          gcode_M112();
          break;

        case 410: // M410 quickstop - Abort all the planned moves.
          gcode_M410();
          break;

      #endif

      #if ENABLED(HOST_KEEPALIVE_FEATURE)
        case 113: // M113: Set Host Keepalive interval
          gcode_M113();
          break;
      #endif

      case 140: // M140: Set bed temp
        gcode_M140();
        break;

      case 105: // M105: Read current temperature
        gcode_M105();
        KEEPALIVE_STATE(NOT_BUSY);
        return; // "ok" already printed

      case 109: // M109: Wait for temperature
        gcode_M109();
        break;

      #if HAS_TEMP_BED
        case 190: // M190: Wait for bed heater to reach target
          gcode_M190();
          break;
      #endif // HAS_TEMP_BED

      #if FAN_COUNT > 0
        case 106: // M106: Fan On
          gcode_M106();
          break;
        case 107: // M107: Fan Off
          gcode_M107();
          break;
      #endif // FAN_COUNT > 0

      #if ENABLED(BARICUDA)
        // PWM for HEATER_1_PIN
        #if HAS_HEATER_1
          case 126: // M126: valve open
            gcode_M126();
            break;
          case 127: // M127: valve closed
            gcode_M127();
            break;
        #endif // HAS_HEATER_1

        // PWM for HEATER_2_PIN
        #if HAS_HEATER_2
          case 128: // M128: valve open
            gcode_M128();
            break;
          case 129: // M129: valve closed
            gcode_M129();
            break;
        #endif // HAS_HEATER_2
      #endif // BARICUDA

      #if HAS_POWER_SWITCH

        case 80: // M80: Turn on Power Supply
          gcode_M80();
          break;

      #endif // HAS_POWER_SWITCH

      case 81: // M81: Turn off Power, including Power Supply, if possible
        gcode_M81();
        break;

      case 82:
        gcode_M82();
        break;
      case 83:
        gcode_M83();
        break;
      case 18: // (for compatibility)
      case 84: // M84
        gcode_M18_M84();
        break;
      case 85: // M85
        gcode_M85();
        break;
      case 92: // M92: Set the steps-per-unit for one or more axes
        gcode_M92();
        break;
      case 115: // M115: Report capabilities
        gcode_M115();
        break;
      case 117: // M117: Set LCD message text, if possible
        gcode_M117();
        break;
      case 114: // M114: Report current position
        gcode_M114();
        break;
      case 120: // M120: Enable endstops
        gcode_M120();
        break;
      case 121: // M121: Disable endstops
        gcode_M121();
        break;
      case 119: // M119: Report endstop states
        gcode_M119();
        break;

      #if ENABLED(ULTIPANEL)

        case 145: // M145: Set material heatup parameters
          gcode_M145();
          break;

      #endif

      #if ENABLED(TEMPERATURE_UNITS_SUPPORT)
        case 149:
          gcode_M149();
          break;
      #endif

      #if ENABLED(BLINKM)

        case 150: // M150
          gcode_M150();
          break;

      #endif //BLINKM

      #if ENABLED(EXPERIMENTAL_I2CBUS)

        case 155:
          gcode_M155();
          break;

        case 156:
          gcode_M156();
          break;

      #endif //EXPERIMENTAL_I2CBUS

      #if ENABLED(MIXING_EXTRUDER)
        case 163: // M163 S<int> P<float> set weight for a mixing extruder
          gcode_M163();
          break;
        #if MIXING_VIRTUAL_TOOLS > 1
          case 164: // M164 S<int> save current mix as a virtual extruder
            gcode_M164();
            break;
        #endif
        #if ENABLED(DIRECT_MIXING_IN_G1)
          case 165: // M165 [ABCDHI]<float> set multiple mix weights
            gcode_M165();
            break;
        #endif
      #endif

      case 200: // M200 D<diameter> Set filament diameter and set E axis units to cubic. (Use S0 to revert to linear units.)
        gcode_M200();
        break;
      case 201: // M201
        gcode_M201();
        break;
      #if 0 // Not used for Sprinter/grbl gen6
        case 202: // M202
          gcode_M202();
          break;
      #endif
      case 203: // M203 max feedrate units/sec
        gcode_M203();
        break;
      case 204: // M204 acclereration S normal moves T filmanent only moves
        gcode_M204();
        break;
      case 205: //M205 advanced settings:  minimum travel speed S=while printing T=travel only,  B=minimum segment time X= maximum xy jerk, Z=maximum Z jerk
        gcode_M205();
        break;
      case 206: // M206 additional homing offset
        gcode_M206();
        break;

      #if ENABLED(DELTA)
        case 665: // M665 set delta configurations L<diagonal_rod> R<delta_radius> S<segments_per_sec>
          gcode_M665();
          break;
      #endif

      #if ENABLED(DELTA) || ENABLED(Z_DUAL_ENDSTOPS)
        case 666: // M666 set delta / dual endstop adjustment
          gcode_M666();
          break;
      #endif

      #if ENABLED(FWRETRACT)
        case 207: // M207 - Set Retract Length: S<length>, Feedrate: F<units/min>, and Z lift: Z<distance>
          gcode_M207();
          break;
        case 208: // M208 - Set Recover (unretract) Additional (!) Length: S<length> and Feedrate: F<units/min>
          gcode_M208();
          break;
        case 209: // M209 - Turn Automatic Retract Detection on/off: S<bool> (For slicers that don't support G10/11). Every normal extrude-only move will be classified as retract depending on the direction.
          gcode_M209();
          break;
      #endif // FWRETRACT

      #if HOTENDS > 1
        case 218: // M218 - Set a tool offset: T<index> X<offset> Y<offset>
          gcode_M218();
          break;
      #endif

      case 220: // M220 - Set Feedrate Percentage: S<percent> ("FR" on your LCD)
        gcode_M220();
        break;

      case 221: // M221 - Set Flow Percentage: S<percent>
        gcode_M221();
        break;

      case 226: // M226 P<pin number> S<pin state>- Wait until the specified pin reaches the state required
        gcode_M226();
        break;

      #if HAS_SERVOS
        case 280: // M280 - set servo position absolute. P: servo index, S: angle or microseconds
          gcode_M280();
          break;
      #endif // HAS_SERVOS

      #if HAS_BUZZER
        case 300: // M300 - Play beep tone
          gcode_M300();
          break;
      #endif // HAS_BUZZER

      #if ENABLED(PIDTEMP)
        case 301: // M301
          gcode_M301();
          break;
      #endif // PIDTEMP

      #if ENABLED(PIDTEMPBED)
        case 304: // M304
          gcode_M304();
          break;
      #endif // PIDTEMPBED

      #if defined(CHDK) || HAS_PHOTOGRAPH
        case 240: // M240  Triggers a camera by emulating a Canon RC-1 : http://www.doc-diy.net/photo/rc-1_hacked/
          gcode_M240();
          break;
      #endif // CHDK || PHOTOGRAPH_PIN

      #if HAS_LCD_CONTRAST
        case 250: // M250  Set LCD contrast value: C<value> (value 0..63)
          gcode_M250();
          break;
      #endif // HAS_LCD_CONTRAST

      #if ENABLED(PREVENT_DANGEROUS_EXTRUDE)
        case 302: // allow cold extrudes, or set the minimum extrude temperature
          gcode_M302();
          break;
      #endif // PREVENT_DANGEROUS_EXTRUDE

      case 303: // M303 PID autotune
        gcode_M303();
        break;

      #if ENABLED(SCARA)
        case 360:  // M360 SCARA Theta pos1
          if (gcode_M360()) return;
          break;
        case 361:  // M361 SCARA Theta pos2
          if (gcode_M361()) return;
          break;
        case 362:  // M362 SCARA Psi pos1
          if (gcode_M362()) return;
          break;
        case 363:  // M363 SCARA Psi pos2
          if (gcode_M363()) return;
          break;
        case 364:  // M364 SCARA Psi pos3 (90 deg to Theta)
          if (gcode_M364()) return;
          break;
        case 365: // M365 Set SCARA scaling for X Y Z
          gcode_M365();
          break;
      #endif // SCARA

      case 400: // M400 finish all moves
        gcode_M400();
        break;

      #if HAS_BED_PROBE
        case 401:
          gcode_M401();
          break;
        case 402:
          gcode_M402();
          break;
      #endif // HAS_BED_PROBE

      #if ENABLED(FILAMENT_WIDTH_SENSOR)
        case 404:  //M404 Enter the nominal filament width (3mm, 1.75mm ) N<3.0> or display nominal filament width
          gcode_M404();
          break;
        case 405:  //M405 Turn on filament sensor for control
          gcode_M405();
          break;
        case 406:  //M406 Turn off filament sensor for control
          gcode_M406();
          break;
        case 407:   //M407 Display measured filament diameter
          gcode_M407();
          break;
      #endif // ENABLED(FILAMENT_WIDTH_SENSOR)

      #if ENABLED(MESH_BED_LEVELING)
        case 420: // M420 Enable/Disable Mesh Bed Leveling
          gcode_M420();
          break;
        case 421: // M421 Set a Mesh Bed Leveling Z coordinate
          gcode_M421();
          break;
      #endif

      case 428: // M428 Apply current_position to home_offset
        gcode_M428();
        break;

      case 500: // M500 Store settings in EEPROM
        gcode_M500();
        break;
      case 501: // M501 Read settings from EEPROM
        gcode_M501();
        break;
      case 502: // M502 Revert to default settings
        gcode_M502();
        break;
      case 503: // M503 print settings currently in memory
        gcode_M503();
        break;

      #if ENABLED(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED)
        case 540:
          gcode_M540();
          break;
      #endif

      #if HAS_BED_PROBE
        case 851:
          gcode_M851();
          break;
      #endif // HAS_BED_PROBE

      #if ENABLED(FILAMENT_CHANGE_FEATURE)
        case 600: //Pause for filament change X[pos] Y[pos] Z[relative lift] E[initial retract] L[later retract distance for removal]
          gcode_M600();
          break;
      #endif // FILAMENT_CHANGE_FEATURE

      #if ENABLED(DUAL_X_CARRIAGE)
        case 605:
          gcode_M605();
          break;
      #endif // DUAL_X_CARRIAGE

      #if ENABLED(LIN_ADVANCE)
        case 905: // M905 Set advance factor.
          gcode_M905();
          break;
      #endif

      case 907: // M907 Set digital trimpot motor current using axis codes.
        gcode_M907();
        break;

      #if HAS_DIGIPOTSS || ENABLED(DAC_STEPPER_CURRENT)

        case 908: // M908 Control digital trimpot directly.
          gcode_M908();
          break;

        #if ENABLED(DAC_STEPPER_CURRENT) // As with Printrbot RevF

          case 909: // M909 Print digipot/DAC current value
            gcode_M909();
            break;

          case 910: // M910 Commit digipot/DAC value to external EEPROM
            gcode_M910();
            break;

        #endif

      #endif // HAS_DIGIPOTSS || DAC_STEPPER_CURRENT

      #if HAS_MICROSTEPS

        case 350: // M350 Set microstepping mode. Warning: Steps per unit remains unchanged. S code sets stepping mode for all drivers.
          gcode_M350();
          break;

        case 351: // M351 Toggle MS1 MS2 pins directly, S# determines MS1 or MS2, X# sets the pin high/low.
          gcode_M351();
          break;

      #endif // HAS_MICROSTEPS

      case 999: // M999: Restart after being Stopped
        gcode_M999();
        break;
    }
    break;

    case 'T':
      gcode_T(codenum);
      break;

    default: code_is_good = false;
  }

  KEEPALIVE_STATE(NOT_BUSY);

ExitUnknownCommand:

  // Still unknown command? Throw an error
  if (!code_is_good) {
    SERIAL_ECHO_START;
    SERIAL_ECHOPGM(MSG_UNKNOWN_COMMAND);
    SERIAL_ECHO(current_command);
    SERIAL_ECHOLNPGM("\"");
  }

  ok_to_send();
}
