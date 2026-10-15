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


#include "RecastReuse.h"
#include <RecastAlloc.h>
#include <RecastAssert.h>
#include <cstddef>
#include <cstring>

bool rcResetHeightfield(rcHeightfield& hf, int width, int height, const float* bmin, const float* bmax, float cs, float ch)
{
    // sub-tiles of one ADT tile all have the same grid size; a different size would need a new spans array
    if (hf.spans && (hf.width != width || hf.height != height))
    {
        rcFree(hf.spans);
        hf.spans = nullptr;
    }

    hf.width = width;
    hf.height = height;
    rcVcopy(hf.bmin, bmin);
    rcVcopy(hf.bmax, bmax);
    hf.cs = cs;
    hf.ch = ch;
    if (!hf.spans)
    {
        hf.spans = (rcSpan**)rcAlloc(sizeof(rcSpan*) * hf.width * hf.height, RC_ALLOC_PERM);
        if (!hf.spans)
            return false;
    }
    memset(hf.spans, 0, sizeof(rcSpan*) * hf.width * hf.height);

    // Rewind the pools: every span handed out before is dead now, so thread all items of all pools into the
    // free list again (allocSpan's own order: items of a pool ascend, a newer pool in front of an older one).
    // Which rcSpan struct a span lands in is not observable in any output.
    hf.freelist = nullptr;
    for (rcSpanPool* pool = hf.pools; pool; pool = pool->next)
    {
        rcSpan* head = &pool->items[0];
        rcSpan* it = &pool->items[RC_SPANS_PER_POOL];
        do
        {
            --it;
            it->next = hf.freelist;
            hf.freelist = it;
        }
        while (it != head);
    }
    return true;
}

bool rcBuildCompactHeightfieldReuse(rcContext* ctx, const int walkableHeight, const int walkableClimb,
    rcHeightfield& hf, SubTileScratch& scratch)
{
    rcAssert(ctx);

    rcScopedTimer timer(ctx, RC_TIMER_BUILD_COMPACTHEIGHTFIELD);

    rcCompactHeightfield& chf = scratch.chf;
    const int w = hf.width;
    const int h = hf.height;
    const int spanCount = rcGetHeightFieldSpanCount(ctx, hf);

    // Fill in header (also resets every field a later stage of the previous sub-tile wrote: a fresh
    // rcCompactHeightfield has them zero).
    chf.width = w;
    chf.height = h;
    chf.spanCount = spanCount;
    chf.walkableHeight = walkableHeight;
    chf.walkableClimb = walkableClimb;
    chf.borderSize = 0;
    chf.maxDistance = 0;
    chf.maxRegions = 0;
    rcVcopy(chf.bmin, hf.bmin);
    rcVcopy(chf.bmax, hf.bmax);
    chf.bmax[1] += walkableHeight * hf.ch;
    chf.cs = hf.cs;
    chf.ch = hf.ch;

    // the distance field is rebuilt by rcBuildDistanceField; a fresh chf has none
    rcFree(chf.dist);
    chf.dist = nullptr;

    // buffers: the memset below makes the first w*h cells / spanCount spans / areas identical to a fresh
    // allocation; nothing reads past those counts.
    if (!chf.cells || scratch.chfCellCap < w * h)
    {
        rcFree(chf.cells);
        chf.cells = nullptr;
        scratch.chfCellCap = 0;
        chf.cells = (rcCompactCell*)rcAlloc(sizeof(rcCompactCell) * w * h, RC_ALLOC_PERM);
        if (!chf.cells)
        {
            ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.cells' (%d)", w * h);
            return false;
        }
        scratch.chfCellCap = w * h;
    }
    memset(chf.cells, 0, sizeof(rcCompactCell) * w * h);

    // capacity at least 1: rcAlloc(0) is malloc(0), non-null in the original
    if (!chf.spans || !chf.areas || scratch.chfSpanCap < spanCount)
    {
        rcFree(chf.spans);
        rcFree(chf.areas);
        chf.spans = nullptr;
        chf.areas = nullptr;
        scratch.chfSpanCap = 0;
        int const cap = spanCount > 0 ? spanCount : 1;
        chf.spans = (rcCompactSpan*)rcAlloc(sizeof(rcCompactSpan) * cap, RC_ALLOC_PERM);
        if (!chf.spans)
        {
            ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.spans' (%d)", spanCount);
            return false;
        }
        chf.areas = (unsigned char*)rcAlloc(sizeof(unsigned char) * cap, RC_ALLOC_PERM);
        if (!chf.areas)
        {
            ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Out of memory 'chf.areas' (%d)", spanCount);
            return false;
        }
        scratch.chfSpanCap = cap;
    }
    memset(chf.spans, 0, sizeof(rcCompactSpan) * spanCount);
    memset(chf.areas, RC_NULL_AREA, sizeof(unsigned char) * spanCount);

    const int MAX_HEIGHT = 0xffff;

    // Fill in cells and spans.
    int idx = 0;
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            const rcSpan* s = hf.spans[x + y * w];
            // If there are no spans at this cell, just leave the data to index=0, count=0.
            if (!s) continue;
            rcCompactCell& c = chf.cells[x + y * w];
            c.index = idx;
            c.count = 0;
            while (s)
            {
                if (s->area != RC_NULL_AREA)
                {
                    const int bot = (int)s->smax;
                    const int top = s->next ? (int)s->next->smin : MAX_HEIGHT;
                    chf.spans[idx].y = (unsigned short)rcClamp(bot, 0, 0xffff);
                    chf.spans[idx].h = (unsigned char)rcClamp(top - bot, 0, 0xff);
                    chf.areas[idx] = s->area;
                    idx++;
                    c.count++;
                }
                s = s->next;
            }
        }
    }

    // Find neighbour connections.
    const int MAX_LAYERS = RC_NOT_CONNECTED - 1;
    int tooHighNeighbour = 0;
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            const rcCompactCell& c = chf.cells[x + y * w];
            for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
            {
                rcCompactSpan& s = chf.spans[i];

                for (int dir = 0; dir < 4; ++dir)
                {
                    rcSetCon(s, dir, RC_NOT_CONNECTED);
                    const int nx = x + rcGetDirOffsetX(dir);
                    const int ny = y + rcGetDirOffsetY(dir);
                    // First check that the neighbour cell is in bounds.
                    if (nx < 0 || ny < 0 || nx >= w || ny >= h)
                        continue;

                    // Iterate over all neighbour spans and check if any of the is
                    // accessible from current cell.
                    const rcCompactCell& nc = chf.cells[nx + ny * w];
                    for (int k = (int)nc.index, nk = (int)(nc.index + nc.count); k < nk; ++k)
                    {
                        const rcCompactSpan& ns = chf.spans[k];
                        const int bot = rcMax(s.y, ns.y);
                        const int top = rcMin(s.y + s.h, ns.y + ns.h);

                        // Check that the gap between the spans is walkable,
                        // and that the climb height between the gaps is not too high.
                        if ((top - bot) >= walkableHeight && rcAbs((int)ns.y - (int)s.y) <= walkableClimb)
                        {
                            // Mark direction as walkable.
                            const int lidx = k - (int)nc.index;
                            if (lidx < 0 || lidx > MAX_LAYERS)
                            {
                                tooHighNeighbour = rcMax(tooHighNeighbour, lidx);
                                continue;
                            }
                            rcSetCon(s, dir, lidx);
                            break;
                        }
                    }
                }
            }
        }
    }

    if (tooHighNeighbour > MAX_LAYERS)
    {
        ctx->log(RC_LOG_ERROR, "rcBuildCompactHeightfield: Heightfield has too many layers %d (max: %d)",
            tooHighNeighbour, MAX_LAYERS);
    }

    return true;
}
