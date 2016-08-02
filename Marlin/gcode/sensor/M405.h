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
 * M405: Turn on filament sensor for control
 */
inline void gcode_M405() {
  // This is technically a linear measurement, but since it's quantized to centimeters and is a different unit than
  // everything else, it uses code_value_int() instead of code_value_linear_units().
  if (code_seen('D')) meas_delay_cm = code_value_int();
  NOMORE(meas_delay_cm, MAX_MEASUREMENT_DELAY);

  if (filwidth_delay_index2 == -1) { // Initialize the ring buffer if not done since startup
    int temp_ratio = thermalManager.widthFil_to_size_ratio();

    for (uint8_t i = 0; i < COUNT(measurement_delay); ++i)
      measurement_delay[i] = temp_ratio - 100;  // Subtract 100 to scale within a signed byte

    filwidth_delay_index1 = filwidth_delay_index2 = 0;
  }

  filament_sensor = true;

  //SERIAL_PROTOCOLPGM("Filament dia (measured mm):");
  //SERIAL_PROTOCOL(filament_width_meas);
  //SERIAL_PROTOCOLPGM("Extrusion ratio(%):");
  //SERIAL_PROTOCOL(extruder_multiplier[active_extruder]);
}
