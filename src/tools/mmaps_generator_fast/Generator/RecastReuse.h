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


#ifndef TRINITYCORE_MMAPS_RECAST_REUSE_H
#define TRINITYCORE_MMAPS_RECAST_REUSE_H

#include <Recast.h>

// Per-ADT-tile scratch reused by all 25x25 sub-tiles (item 535 F6). Holds one heightfield and one compact
// heightfield whose buffers survive from one sub-tile to the next.
struct SubTileScratch
{
    rcHeightfield solid;
    rcCompactHeightfield chf;
    int chfCellCap = 0;
    int chfSpanCap = 0;
};

// Same postcondition as rcCreateHeightfield on a fresh heightfield: every column empty, header set. The spans
// array and the span pools are kept: the pools' items are re-threaded into one free list (all spans of the
// previous sub-tile are dead).
bool rcResetHeightfield(rcHeightfield& hf, int width, int height, const float* bmin, const float* bmax, float cs, float ch);

// rcBuildCompactHeightfield, copied from dep/recastnavigation with the three buffer allocations replaced by
// reuse of the buffers of the previous sub-tile in `scratch.chf` (grown when too small). `chf` must be
// scratch.chf.
bool rcBuildCompactHeightfieldReuse(rcContext* ctx, const int walkableHeight, const int walkableClimb,
    rcHeightfield& hf, SubTileScratch& scratch);

#endif
