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


#include "RecastFilterFast.h"
#include <cstddef>
#include <RecastAssert.h>

void rcFilterLedgeSpansFast(rcContext* ctx, const int walkableHeight, const int walkableClimb, rcHeightfield& solid)
{
    rcAssert(ctx);

    rcScopedTimer timer(ctx, RC_TIMER_FILTER_BORDER);

    const int w = solid.width;
    const int h = solid.height;
    const int MAX_HEIGHT = 0xffff;

    // Direction order and offsets as rcGetDirOffsetX/Y; the order is irrelevant to the verdict (see below)
    // but kept so a reader can diff against the original.
    static constexpr int dirX[4] = { -1, 0, 1, 0 };
    static constexpr int dirY[4] = { 0, 1, 0, -1 };

    // Original semantics: a span is turned into RC_NULL_AREA iff, after scanning all 4 neighbour columns,
    //   minh < -walkableClimb   or   asmax - asmin > walkableClimb.
    // minh only ever decreases and [asmin, asmax] only ever widens while scanning, and the scan only reads
    // smin/smax of spans (never area), so the final verdict is "true" iff it is already true at some point
    // of the scan. Stopping at that point therefore yields the same area for every span. A span whose area is
    // already RC_NULL_AREA is skipped, as in the original; nothing here reads another span's area, so the
    // in-place updates cannot influence later spans.
    for (int y = 0; y < h; ++y)
    {
        for (int x = 0; x < w; ++x)
        {
            for (rcSpan* s = solid.spans[x + y * w]; s; s = s->next)
            {
                if (s->area == RC_NULL_AREA)
                    continue;

                const int bot = (int)(s->smax);
                const int top = s->next ? (int)(s->next->smin) : MAX_HEIGHT;

                int minh = MAX_HEIGHT;
                int asmin = bot;
                int asmax = bot;
                bool ledge = false;

                for (int dir = 0; dir < 4 && !ledge; ++dir)
                {
                    int dx = x + dirX[dir];
                    int dy = y + dirY[dir];
                    if (dx < 0 || dy < 0 || dx >= w || dy >= h)
                    {
                        minh = rcMin(minh, -walkableClimb - bot);
                        ledge = minh < -walkableClimb;
                        continue;
                    }

                    const rcSpan* first = solid.spans[dx + dy * w];

                    // From minus infinity to the first span.
                    {
                        const int nbot = -walkableClimb;
                        const int ntop = first ? (int)first->smin : MAX_HEIGHT;
                        if (rcMin(top, ntop) - rcMax(bot, nbot) > walkableHeight)
                        {
                            minh = rcMin(minh, nbot - bot);
                            if (minh < -walkableClimb)
                            {
                                ledge = true;
                                break;
                            }
                        }
                    }

                    // Rest of the spans. Spans of a column ascend, so once top - nbot <= walkableHeight the gap
                    // test min(top,ntop) - max(bot,nbot) <= top - nbot <= walkableHeight fails for this and every
                    // later span: nothing more can be contributed by this column.
                    const int limit = top - walkableHeight;
                    for (const rcSpan* ns = first; ns; ns = ns->next)
                    {
                        const int nbot = (int)ns->smax;
                        if (nbot >= limit)
                            break;
                        const int ntop = ns->next ? (int)ns->next->smin : MAX_HEIGHT;
                        if (rcMin(top, ntop) - rcMax(bot, nbot) > walkableHeight)
                        {
                            minh = rcMin(minh, nbot - bot);
                            if (minh < -walkableClimb)
                            {
                                ledge = true;
                                break;
                            }

                            if (rcAbs(nbot - bot) <= walkableClimb)
                            {
                                if (nbot < asmin) asmin = nbot;
                                if (nbot > asmax) asmax = nbot;
                                if (asmax - asmin > walkableClimb)
                                {
                                    ledge = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                if (ledge)
                    s->area = RC_NULL_AREA;
            }
        }
    }
}
