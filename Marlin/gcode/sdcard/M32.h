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
 * M32: Select file and start SD Print
 */
inline void gcode_M32() {
  if (card.sdprinting)
    stepper.synchronize();

  char* namestartpos = strchr(current_command_args, '!');  // Find ! to indicate filename string start.
  if (!namestartpos)
    namestartpos = current_command_args; // Default name position, 4 letters after the M
  else
    namestartpos++; //to skip the '!'

  bool call_procedure = code_seen('P') && (seen_pointer < namestartpos);

  if (card.cardOK) {
    card.openFile(namestartpos, true, call_procedure);

    if (code_seen('S') && seen_pointer < namestartpos) // "S" (must occur _before_ the filename!)
      card.setIndex(code_value_long());

    card.startFileprint();

    // Procedure calls count as normal print time.
    if (!call_procedure) print_job_timer.start();
  }
}
