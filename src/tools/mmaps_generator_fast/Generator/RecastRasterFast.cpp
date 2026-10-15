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

#include "RecastRasterFast.h"
#include <cmath>
#include <RecastAlloc.h>
#include <RecastAssert.h>

namespace
{
    inline bool overlapBounds(const float* amin, const float* amax, const float* bmin, const float* bmax)
    {
        bool overlap = true;
        overlap = (amin[0] > bmax[0] || amax[0] < bmin[0]) ? false : overlap;
        overlap = (amin[1] > bmax[1] || amax[1] < bmin[1]) ? false : overlap;
        overlap = (amin[2] > bmax[2] || amax[2] < bmin[2]) ? false : overlap;
        return overlap;
    }

    // verbatim copies of the static helpers in RecastRasterization.cpp
    rcSpan* allocSpan(rcHeightfield& hf)
    {
        if (!hf.freelist || !hf.freelist->next)
        {
            rcSpanPool* pool = (rcSpanPool*)rcAlloc(sizeof(rcSpanPool), RC_ALLOC_PERM);
            if (!pool) return 0;

            pool->next = hf.pools;
            hf.pools = pool;
            rcSpan* freelist = hf.freelist;
            rcSpan* head = &pool->items[0];
            rcSpan* it = &pool->items[RC_SPANS_PER_POOL];
            do
            {
                --it;
                it->next = freelist;
                freelist = it;
            }
            while (it != head);
            hf.freelist = it;
        }

        rcSpan* it = hf.freelist;
        hf.freelist = hf.freelist->next;
        return it;
    }

    inline void freeSpan(rcHeightfield& hf, rcSpan* ptr)
    {
        if (!ptr) return;
        ptr->next = hf.freelist;
        hf.freelist = ptr;
    }

    bool addSpan(rcHeightfield& hf, const int x, const int y,
                 const unsigned short smin, const unsigned short smax,
                 const unsigned char area, const int flagMergeThr)
    {
        int idx = x + y*hf.width;

        rcSpan* s = allocSpan(hf);
        if (!s)
            return false;
        s->smin = smin;
        s->smax = smax;
        s->area = area;
        s->next = 0;

        if (!hf.spans[idx])
        {
            hf.spans[idx] = s;
            return true;
        }
        rcSpan* prev = 0;
        rcSpan* cur = hf.spans[idx];

        while (cur)
        {
            if (cur->smin > s->smax)
            {
                break;
            }
            else if (cur->smax < s->smin)
            {
                prev = cur;
                cur = cur->next;
            }
            else
            {
                if (rcAbs((int)s->smax - (int)cur->smax) <= flagMergeThr)
                    s->area = rcMax(s->area, cur->area);

                if (cur->smin < s->smin)
                    s->smin = cur->smin;
                if (cur->smax > s->smax)
                    s->smax = cur->smax;

                rcSpan* next = cur->next;
                freeSpan(hf, cur);
                if (prev)
                    prev->next = next;
                else
                    hf.spans[idx] = next;
                cur = next;
            }
        }

        if (prev)
        {
            s->next = prev->next;
            prev->next = s;
        }
        else
        {
            s->next = hf.spans[idx];
            hf.spans[idx] = s;
        }

        return true;
    }

    inline void vcopy(float* __restrict dst, const float* __restrict src)
    {
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
    }

    // dividePoly with the axis fixed at compile time. out1 receives the points with in[axis] <= x (d >= 0),
    // out2 the others; with BOTH == false out2 is not produced (the caller discards it). Every value written to
    // out1 is computed by the same expression as the original, in the same order.
    template <int AXIS, bool BOTH>
    inline void dividePoly(const float* __restrict in, int nin, float* __restrict out1, int* nout1,
        float* __restrict out2, int* nout2, float x)
    {
        float d[12];
        for (int i = 0; i < nin; ++i)
            d[i] = x - in[i*3+AXIS];

        int m = 0, n = 0;
        for (int i = 0, j = nin-1; i < nin; j=i, ++i)
        {
            bool ina = d[j] >= 0;
            bool inb = d[i] >= 0;
            if (ina != inb)
            {
                float s = d[j] / (d[j] - d[i]);
                out1[m*3+0] = in[j*3+0] + (in[i*3+0] - in[j*3+0])*s;
                out1[m*3+1] = in[j*3+1] + (in[i*3+1] - in[j*3+1])*s;
                out1[m*3+2] = in[j*3+2] + (in[i*3+2] - in[j*3+2])*s;
                if constexpr (BOTH)
                {
                    vcopy(out2 + n*3, out1 + m*3);
                    n++;
                }
                m++;
                if (d[i] > 0)
                {
                    vcopy(out1 + m*3, in + i*3);
                    m++;
                }
                else if constexpr (BOTH)
                {
                    if (d[i] < 0)
                    {
                        vcopy(out2 + n*3, in + i*3);
                        n++;
                    }
                }
            }
            else
            {
                if (d[i] >= 0)
                {
                    vcopy(out1 + m*3, in + i*3);
                    m++;
                    if (d[i] != 0)
                        continue;
                }
                if constexpr (BOTH)
                {
                    vcopy(out2 + n*3, in + i*3);
                    n++;
                }
            }
        }

        *nout1 = m;
        if constexpr (BOTH)
            *nout2 = n;
    }

    bool rasterizeTri(const float* v0, const float* v1, const float* v2,
                      const unsigned char area, rcHeightfield& hf,
                      const float* bmin, const float* bmax,
                      const float cs, const float ics, const float ich,
                      const int flagMergeThr)
    {
        const int w = hf.width;
        const int h = hf.height;
        float tmin[3], tmax[3];
        const float by = bmax[1] - bmin[1];

        rcVcopy(tmin, v0);
        rcVcopy(tmax, v0);
        rcVmin(tmin, v1);
        rcVmin(tmin, v2);
        rcVmax(tmax, v1);
        rcVmax(tmax, v2);

        if (!overlapBounds(bmin, bmax, tmin, tmax))
            return true;

        int y0 = (int)((tmin[2] - bmin[2])*ics);
        int y1 = (int)((tmax[2] - bmin[2])*ics);
        y0 = rcClamp(y0, 0, h-1);
        y1 = rcClamp(y1, 0, h-1);

        float buf[7*3*4];
        float *in = buf, *inrow = buf+7*3, *p1 = inrow+7*3, *p2 = p1+7*3;

        rcVcopy(&in[0], v0);
        rcVcopy(&in[1*3], v1);
        rcVcopy(&in[2*3], v2);
        int nvrow, nvIn = 3;

        for (int y = y0; y <= y1; ++y)
        {
            const float cz = bmin[2] + y*cs;
            if (y == y1)
                dividePoly<2, false>(in, nvIn, inrow, &nvrow, nullptr, nullptr, cz+cs);
            else
            {
                dividePoly<2, true>(in, nvIn, inrow, &nvrow, p1, &nvIn, cz+cs);
                rcSwap(in, p1);
            }
            if (nvrow < 3) continue;

            float minX = inrow[0], maxX = inrow[0];
            for (int i=1; i<nvrow; ++i)
            {
                if (minX > inrow[i*3])	minX = inrow[i*3];
                if (maxX < inrow[i*3])	maxX = inrow[i*3];
            }
            int x0 = (int)((minX - bmin[0])*ics);
            int x1 = (int)((maxX - bmin[0])*ics);
            x0 = rcClamp(x0, 0, w-1);
            x1 = rcClamp(x1, 0, w-1);

            int nv, nv2 = nvrow;

            for (int x = x0; x <= x1; ++x)
            {
                const float cx = bmin[0] + x*cs;
                if (x == x1)
                    dividePoly<0, false>(inrow, nv2, p1, &nv, nullptr, nullptr, cx+cs);
                else
                {
                    dividePoly<0, true>(inrow, nv2, p1, &nv, p2, &nv2, cx+cs);
                    rcSwap(inrow, p2);
                }
                if (nv < 3) continue;

                float smin = p1[1], smax = p1[1];
                for (int i = 1; i < nv; ++i)
                {
                    smin = rcMin(smin, p1[i*3+1]);
                    smax = rcMax(smax, p1[i*3+1]);
                }
                smin -= bmin[1];
                smax -= bmin[1];
                if (smax < 0.0f) continue;
                if (smin > by) continue;
                if (smin < 0.0f) smin = 0;
                if (smax > by) smax = by;

                unsigned short ismin = (unsigned short)rcClamp((int)floorf(smin * ich), 0, RC_SPAN_MAX_HEIGHT);
                unsigned short ismax = (unsigned short)rcClamp((int)ceilf(smax * ich), (int)ismin+1, RC_SPAN_MAX_HEIGHT);

                if (!addSpan(hf, x, y, ismin, ismax, area, flagMergeThr))
                    return false;
            }
        }

        return true;
    }
}

bool rcRasterizeTrianglesFast(rcContext* ctx, const float* verts, const int* tris, const unsigned char* areas, int nt,
    rcHeightfield& solid, int flagMergeThr)
{
    rcAssert(ctx);

    rcScopedTimer timer(ctx, RC_TIMER_RASTERIZE_TRIANGLES);

    const float ics = 1.0f/solid.cs;
    const float ich = 1.0f/solid.ch;
    for (int i = 0; i < nt; ++i)
    {
        const float* v0 = &verts[tris[i*3+0]*3];
        const float* v1 = &verts[tris[i*3+1]*3];
        const float* v2 = &verts[tris[i*3+2]*3];
        if (!rasterizeTri(v0, v1, v2, areas[i], solid, solid.bmin, solid.bmax, solid.cs, ics, ich, flagMergeThr))
        {
            ctx->log(RC_LOG_ERROR, "rcRasterizeTriangles: Out of memory.");
            return false;
        }
    }

    return true;
}
