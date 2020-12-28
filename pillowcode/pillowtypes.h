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

#ifndef PILLOWTYPES_H
#define PILLOWTYPES_H

/** Defines all the states for the pillow state machine. */
typedef enum { INVALID_STATE, DEFAULT_STATE, NOPHONE_STATE, NOPHONE_SLEEP_STATE,
               PHONE_INSERTED_STATE, PHONEIN_SOUNDON_STATE, PHONEIN_STATE,
               PHONE_REMOVED_SOUNDOFF_STATE, PHONEIN_SLEEP_SOUNDOFF_STATE,
               PHONEIN_SLEEP_STATE, VIBRATED_STATE, PHONE_REMOVED_STATE } State;

/**
 * States for when the phone is removed for a brief period of time.
 * OUT means the phone was out temporarily at some point, whereas
 * REMOVED means the removed event should be triggered.
 */
typedef enum { PHONE_MARGIN_IN, PHONE_MARGIN_OUT, PHONE_MARGIN_REMOVED
               } PhoneMarginState;

#endif
