// (c) 2019 Matthew Lakier (matthew.lakier@uwaterloo.ca)
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef LOGGING_H
#define LOGGING_H

/** To avoid reading garbage from the EEPROM */
#define LOGGING_HEADER_MAGIC 0xD02E

/** Event Types */
#define TYPE_INSERT 'I'
#define TYPE_REMOVE_ON 'R'
#define TYPE_REMOVE_SLEEP 'X'
#define TYPE_BUMP 'B'
#define TYPE_TIME_ELAPSED 'T'
#define TYPE_POWER_ON 'P'

/**
 * Should be less than 1024, the EEPROM size of the Arduino Pro Mini.
 */
#define LOGGING_EVENT_COUNT 512

/**
 * One of these should be stored (at the beginning) in EEPROM.
 */
struct PillowLog {

  /**
   * Set to 0xD02E, because we DOZE with the pillow.
   */
  unsigned int magic; // 2B

  /**
   * Starts at 0, and counts up each time the arduino is reset.
   * Emptying the log will not reset this value.
   */
  unsigned long count; // 4B

  /**
   * Default 0. Increment for each log. Indicates how many items
   * there are.
   */
  unsigned int idx;  // 2B

  /**
   * Default false. Set to true when all log events filled.
   */
  bool overflowed; // 1B

  /**
   * List of events in order of occurrence.
   * 'I': phone inserted
   * 'R': phone removed (while on)
   * 'X': phone removed (while sleep mode)
   * 'B': pillow bumped
   * 'T': 30 (configurable) minutes passed (while sleep mode)
   * 'P': Arduino powered on
   */
  unsigned char eventType[LOGGING_EVENT_COUNT]; // 1B ea.

};

#endif /* LOGGING_H */

