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

#ifndef TRINITYCORE_MMAPS_RECAST_RASTER_FAST_H
#define TRINITYCORE_MMAPS_RECAST_RASTER_FAST_H

#include <Recast.h>

// Output-identical replacement for rcRasterizeTriangles(ctx, verts, nv, int* tris, areas, nt, hf, flagMergeThr)
// (item 535 G11 fallback). Same float expressions in the same order as dep/recastnavigation
// RecastRasterization.cpp rasterizeTri/dividePoly/addSpan; only the buffer copying and the unused
// remainder polygon of the last row/column are dropped. Returns false on span pool exhaustion.
bool rcRasterizeTrianglesFast(rcContext* ctx, const float* verts, const int* tris, const unsigned char* areas, int nt,
    rcHeightfield& solid, int flagMergeThr);

#endif
