/*
 * This file is part of the TrinityCore Project. See AUTHORS file for Copyright information
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program. If not, see <http://www.gnu.org/licenses/>.
 */


#ifndef TRINITYCORE_MMAPS_RECAST_FILTER_FAST_H
#define TRINITYCORE_MMAPS_RECAST_FILTER_FAST_H

#include <Recast.h>

// Output-identical replacement for rcFilterLedgeSpans (dep/recastnavigation, item 535 G10): sets exactly the
// same spans to RC_NULL_AREA, but stops scanning a span's neighbours once its verdict is settled.
void rcFilterLedgeSpansFast(rcContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& solid);

#endif
