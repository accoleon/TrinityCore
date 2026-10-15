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

#include "TileBuilder.h"
#include "IntermediateValues.h"
#include "Log.h"
#include "MMapDefines.h"
#include "RecastFilterFast.h"
#include "RecastReuse.h"
#include "Memory.h"
#include "StringFormat.h"
#include "VMapManager.h"
#include <DetourNavMeshBuilder.h>
#include <initializer_list>
#include <vector>

namespace
{
    struct Tile
    {
        Tile() : chf(nullptr), solid(nullptr), cset(nullptr), pmesh(nullptr), dmesh(nullptr) {}
        ~Tile()
        {
            rcFreeCompactHeightfield(chf);
            rcFreeContourSet(cset);
            rcFreeHeightField(solid);
            rcFreePolyMesh(pmesh);
            rcFreePolyMeshDetail(dmesh);
        }
        rcCompactHeightfield* chf;
        rcHeightfield* solid;
        rcContourSet* cset;
        rcPolyMesh* pmesh;
        rcPolyMeshDetail* dmesh;
    };
}

namespace MMAP
{
    struct TileConfig
    {
        TileConfig(bool bigBaseUnit)
        {
            // these are WORLD UNIT based metrics
            // this are basic unit dimentions
            // value have to divide GRID_SIZE(533.3333f) ( aka: 0.5333, 0.2666, 0.3333, 0.1333, etc )
            BASE_UNIT_DIM = bigBaseUnit ? 0.5333333f : 0.2666666f;

            // All are in UNIT metrics!
            VERTEX_PER_MAP = int(GRID_SIZE / BASE_UNIT_DIM + 0.5f);
            VERTEX_PER_TILE = bigBaseUnit ? 40 : 80; // must divide VERTEX_PER_MAP
            TILES_PER_MAP = VERTEX_PER_MAP / VERTEX_PER_TILE;
        }

        float BASE_UNIT_DIM;
        int VERTEX_PER_MAP;
        int VERTEX_PER_TILE;
        int TILES_PER_MAP;
    };

    TileBuilder::TileBuilder(boost::filesystem::path const& inputDirectory, boost::filesystem::path const& outputDirectory,
        Optional<float> maxWalkableAngle, Optional<float> maxWalkableAngleNotSteep,
        bool skipLiquid, bool bigBaseUnit, bool debugOutput, std::vector<OffMeshData> const* offMeshConnections) :
        m_outputDirectory(outputDirectory),
        m_maxWalkableAngle(maxWalkableAngle),
        m_maxWalkableAngleNotSteep(maxWalkableAngleNotSteep),
        m_bigBaseUnit(bigBaseUnit),
        m_debugOutput(debugOutput),
        m_terrainBuilder(inputDirectory, skipLiquid),
        m_rcContext(false),
        m_offMeshConnections(offMeshConnections)
    {
    }

    TileBuilder::~TileBuilder() = default;

    /**************************************************************************/
    void TileBuilder::buildTile(uint32 mapID, uint32 tileX, uint32 tileY, dtNavMesh* navMesh)
    {
        if (shouldSkipTile(mapID, tileX, tileY))
        {
            OnTileDone();
            return;
        }

        TC_LOG_INFO("maps.mmapgen", "{} [Map {:04}] Building tile [{:02},{:02}]", GetProgressText(), mapID, tileX, tileY);

        MeshData meshData;

        // reuse this worker's manager while it keeps building the same map; on a map change drop the pinned models
        // first (they release themselves into their manager), then the manager
        if (!m_vmapManager || m_vmapManagerMapId != mapID)
        {
            m_terrainBuilder.clearModelCache();
            m_vmapManager = CreateVMapManager(mapID);
            m_vmapManagerMapId = mapID;
        }
        VMAP::VMapManager* vmapManager = m_vmapManager.get();

        // get heightmap data
        m_terrainBuilder.loadMap(mapID, tileX, tileY, meshData, vmapManager);

        // get model data
        m_terrainBuilder.loadVMap(mapID, tileX, tileY, meshData, vmapManager);

        // if there is no data, give up now
        if (meshData.solidVerts.empty() && meshData.liquidVerts.empty())
        {
            OnTileDone();
            return;
        }

        // remove unused vertices
        TerrainBuilder::cleanVertices(meshData.solidVerts, meshData.solidTris);
        TerrainBuilder::cleanVertices(meshData.liquidVerts, meshData.liquidTris);

        if (meshData.liquidVerts.empty() && meshData.solidVerts.empty())
        {
            OnTileDone();
            return;
        }

        // gather all mesh data for final data check, and bounds calculation
        std::vector<float> allVerts(meshData.liquidVerts.size() + meshData.solidVerts.size());
        std::ranges::copy(meshData.liquidVerts, allVerts.begin());
        std::ranges::copy(meshData.solidVerts, allVerts.begin() + std::ssize(meshData.liquidVerts));

        // get bounds of current tile
        float bmin[3], bmax[3];
        getTileBounds(tileX, tileY, allVerts.data(), allVerts.size() / 3, bmin, bmax);

        if (m_offMeshConnections)
            m_terrainBuilder.loadOffMeshConnections(mapID, tileX, tileY, meshData, *m_offMeshConnections);

        // build navmesh tile
        TileResult tileResult = buildMoveMapTile(mapID, tileX, tileY, meshData, bmin, bmax, navMesh->getParams());
        if (tileResult.data)
            saveMoveMapTileToFile(mapID, tileX, tileY, navMesh, tileResult);

        OnTileDone();
    }

    /**************************************************************************/
    TileBuilder::TileResult TileBuilder::buildMoveMapTile(uint32 mapID, uint32 tileX, uint32 tileY,
        MeshData& meshData, float (&bmin)[3], float (&bmax)[3],
        dtNavMeshParams const* navMeshParams, std::string_view fileNameSuffix)
    {
        // console output
        std::string tileString = Trinity::StringFormat("[Map {:04}] [{:02},{:02}]:", mapID, tileX, tileY);
        TC_LOG_INFO("maps.mmapgen", "{} Building movemap tile...", tileString);

        TileResult tileResult;

        IntermediateValues iv;

        float* tVerts = meshData.solidVerts.data();
        int tVertCount = meshData.solidVerts.size() / 3;
        int* tTris = meshData.solidTris.data();
        int tTriCount = meshData.solidTris.size() / 3;

        float* lVerts = meshData.liquidVerts.data();
        int lVertCount = meshData.liquidVerts.size() / 3;
        int* lTris = meshData.liquidTris.data();
        int lTriCount = meshData.liquidTris.size() / 3;
        uint8* lTriFlags = meshData.liquidType.data();

        const TileConfig tileConfig = TileConfig(m_bigBaseUnit);
        int TILES_PER_MAP = tileConfig.TILES_PER_MAP;
        float BASE_UNIT_DIM = tileConfig.BASE_UNIT_DIM;
        rcConfig config = GetMapSpecificConfig(mapID, bmin, bmax, tileConfig);

        // this sets the dimensions of the heightfield - should maybe happen before border padding
        rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

        // allocate subregions : tiles
        std::unique_ptr<Tile[]> tiles = std::make_unique<Tile[]>(TILES_PER_MAP * TILES_PER_MAP);

        // Initialize per tile config.
        rcConfig tileCfg = config;
        tileCfg.width = config.tileSize + config.borderSize * 2;
        tileCfg.height = config.tileSize + config.borderSize * 2;

        // merge per tile poly and detail meshes
        std::unique_ptr<rcPolyMesh*[]> pmmerge = std::make_unique<rcPolyMesh*[]>(TILES_PER_MAP * TILES_PER_MAP);
        std::unique_ptr<rcPolyMeshDetail*[]> dmmerge = std::make_unique<rcPolyMeshDetail*[]>(TILES_PER_MAP * TILES_PER_MAP);
        int nmerge = 0;

        // mark all walkable tiles, both liquids and solids

        /* we want to have triangles with slope less than walkableSlopeAngleNotSteep (<= 55) to have NAV_AREA_GROUND
         * and with slope between walkableSlopeAngleNotSteep and walkableSlopeAngle (55 < .. <= 70) to have NAV_AREA_GROUND_STEEP.
         * we achieve this using recast API: memset everything to NAV_AREA_GROUND_STEEP, call rcClearUnwalkableTriangles with 70 so
         * any area above that will get RC_NULL_AREA (unwalkable), then call rcMarkWalkableTriangles with 55 to set NAV_AREA_GROUND
         * on anything below 55 . Players and idle Creatures can use NAV_AREA_GROUND, while Creatures in combat can use NAV_AREA_GROUND_STEEP.
         */
        std::unique_ptr<unsigned char[]> triFlags = std::make_unique<unsigned char[]>(tTriCount);
        memset(triFlags.get(), NAV_AREA_GROUND_STEEP, tTriCount * sizeof(unsigned char));
        rcClearUnwalkableTriangles(&m_rcContext, tileCfg.walkableSlopeAngle, tVerts, tVertCount, tTris, tTriCount, triFlags.get());
        rcMarkWalkableTriangles(&m_rcContext, tileCfg.walkableSlopeAngleNotSteep, tVerts, tVertCount, tTris, tTriCount, triFlags.get(), NAV_AREA_GROUND);

        // Calculate the per tile bounding boxes (including border padding), exactly as the heightfield of each
        // sub-tile is created below. X depends only on the column and Z only on the row.
        std::vector<float> subTileMinX(TILES_PER_MAP), subTileMaxX(TILES_PER_MAP);
        std::vector<float> subTileMinZ(TILES_PER_MAP), subTileMaxZ(TILES_PER_MAP);
        for (int i = 0; i < TILES_PER_MAP; ++i)
        {
            float minX = config.bmin[0] + i * float(config.tileSize * config.cs);
            float minZ = config.bmin[2] + i * float(config.tileSize * config.cs);
            float maxX = config.bmin[0] + (i + 1) * float(config.tileSize * config.cs);
            float maxZ = config.bmin[2] + (i + 1) * float(config.tileSize * config.cs);

            minX -= tileCfg.borderSize * tileCfg.cs;
            minZ -= tileCfg.borderSize * tileCfg.cs;
            maxX += tileCfg.borderSize * tileCfg.cs;
            maxZ += tileCfg.borderSize * tileCfg.cs;

            subTileMinX[i] = minX;
            subTileMaxX[i] = maxX;
            subTileMinZ[i] = minZ;
            subTileMaxZ[i] = maxZ;
        }

        // Bin triangle indices per sub-tile. rasterizeTri() skips a triangle whose bounding box does not overlap
        // the heightfield bounds (overlapBounds: reject when amin > bmax || amax < bmin, per axis), so a triangle
        // can only contribute to the sub-tiles it overlaps in X and Z. Use the same inclusive test (and the same
        // negated form, so NaN is kept) and keep ascending triangle order inside every bin.
        // The Y axis is not binned: the Recast test still runs for the triangles that are rasterized.
        auto binTriangles = [&](float const* verts, int const* tris, int triCount)
        {
            std::vector<std::vector<int>> bins(TILES_PER_MAP * TILES_PER_MAP);
            std::vector<char> inColumn(TILES_PER_MAP), inRow(TILES_PER_MAP);
            for (int i = 0; i < triCount; ++i)
            {
                float const* v0 = &verts[tris[i * 3 + 0] * 3];
                float const* v1 = &verts[tris[i * 3 + 1] * 3];
                float const* v2 = &verts[tris[i * 3 + 2] * 3];

                // same min/max selection as rcVmin/rcVmax in rasterizeTri
                float tminX = v0[0], tmaxX = v0[0], tminZ = v0[2], tmaxZ = v0[2];
                for (float const* v : { v1, v2 })
                {
                    if (v[0] < tminX) tminX = v[0];
                    if (v[2] < tminZ) tminZ = v[2];
                    if (v[0] > tmaxX) tmaxX = v[0];
                    if (v[2] > tmaxZ) tmaxZ = v[2];
                }

                for (int k = 0; k < TILES_PER_MAP; ++k)
                {
                    inColumn[k] = !(subTileMinX[k] > tmaxX || subTileMaxX[k] < tminX);
                    inRow[k] = !(subTileMinZ[k] > tmaxZ || subTileMaxZ[k] < tminZ);
                }

                for (int ty = 0; ty < TILES_PER_MAP; ++ty)
                    if (inRow[ty])
                        for (int tx = 0; tx < TILES_PER_MAP; ++tx)
                            if (inColumn[tx])
                                bins[tx + ty * TILES_PER_MAP].push_back(i);
            }
            return bins;
        };

        std::vector<std::vector<int>> solidBins = binTriangles(tVerts, tTris, tTriCount);
        std::vector<std::vector<int>> liquidBins = binTriangles(lVerts, lTris, lTriCount);

        // compacted per sub-tile triangle lists (reused between sub-tiles)
        std::vector<int> binTris;
        std::vector<unsigned char> binAreas;
        auto gatherBin = [&](std::vector<int> const& bin, int const* tris, unsigned char const* areas)
        {
            binTris.clear();
            binAreas.clear();
            for (int triIndex : bin)
            {
                binTris.push_back(tris[triIndex * 3 + 0]);
                binTris.push_back(tris[triIndex * 3 + 1]);
                binTris.push_back(tris[triIndex * 3 + 2]);
                binAreas.push_back(areas[triIndex]);
            }
        };

        // heightfield and compact heightfield buffers shared by all sub-tiles of this ADT tile
        SubTileScratch scratch;
        rcHeightfield& solid = scratch.solid;
        rcCompactHeightfield& chf = scratch.chf;

        // build all tiles
        for (int y = 0; y < TILES_PER_MAP; ++y)
        {
            for (int x = 0; x < TILES_PER_MAP; ++x)
            {
                Tile& tile = tiles[x + y * TILES_PER_MAP];

                // per tile bounding box (precomputed above, shared with the triangle binning)
                tileCfg.bmin[0] = subTileMinX[x];
                tileCfg.bmin[2] = subTileMinZ[y];
                tileCfg.bmax[0] = subTileMaxX[x];
                tileCfg.bmax[2] = subTileMaxZ[y];

                // An empty sub-tile (no solid and no liquid triangle overlaps it) rasterizes nothing: the heightfield has
                // no spans, so the original pipeline ends in a polymesh with nverts == npolys == 0 and an empty detail
                // mesh. The only thing such a mesh contributes to rcMergePolyMeshes is its header (bmin/bmax feed the
                // merged bounds and so every other sub-tile's ox/oz offset and the border flags; nvp/cs/ch come from
                // meshes[0]); rcMergePolyMeshDetails adds nothing. So push a header-only stub with exactly the values
                // rcBuildContours/rcBuildPolyMesh would have derived, and skip the rest.
                if (solidBins[x + y * TILES_PER_MAP].empty() && liquidBins[x + y * TILES_PER_MAP].empty())
                {
                    tile.pmesh = rcAllocPolyMesh();
                    if (tile.pmesh)
                    {
                        // chf.bmin/bmax = hf.bmin/bmax (bmax[1] += walkableHeight * ch); cset applies the border pad to x/z
                        float const pad = tileCfg.borderSize * tileCfg.cs;
                        tile.pmesh->bmin[0] = tileCfg.bmin[0];
                        tile.pmesh->bmin[1] = tileCfg.bmin[1];
                        tile.pmesh->bmin[2] = tileCfg.bmin[2];
                        tile.pmesh->bmax[0] = tileCfg.bmax[0];
                        tile.pmesh->bmax[1] = tileCfg.bmax[1];
                        tile.pmesh->bmax[1] += tileCfg.walkableHeight * tileCfg.ch;
                        tile.pmesh->bmax[2] = tileCfg.bmax[2];
                        tile.pmesh->bmin[0] += pad;
                        tile.pmesh->bmin[2] += pad;
                        tile.pmesh->bmax[0] -= pad;
                        tile.pmesh->bmax[2] -= pad;
                        tile.pmesh->cs = tileCfg.cs;
                        tile.pmesh->ch = tileCfg.ch;
                        tile.pmesh->borderSize = tileCfg.borderSize;
                        tile.pmesh->maxEdgeError = tileCfg.maxSimplificationError;
                        tile.pmesh->nvp = tileCfg.maxVertsPerPoly;
                        pmmerge[nmerge] = tile.pmesh;
                        dmmerge[nmerge] = nullptr;
                        nmerge++;
                        continue;
                    }
                    // allocation failure: fall through to the full pipeline
                }

                // build heightfield
                if (!rcResetHeightfield(solid, tileCfg.width, tileCfg.height, tileCfg.bmin, tileCfg.bmax, tileCfg.cs, tileCfg.ch))
                {
                    TC_LOG_ERROR("maps.mmapgen", "{} Failed building heightfield!", tileString);
                    continue;
                }

                gatherBin(solidBins[x + y * TILES_PER_MAP], tTris, triFlags.get());
                rcRasterizeTriangles(&m_rcContext, tVerts, tVertCount, binTris.data(), binAreas.data(), int(binAreas.size()), solid, config.walkableClimb);

                rcFilterLowHangingWalkableObstacles(&m_rcContext, config.walkableClimb, solid);
                rcFilterLedgeSpansFast(&m_rcContext, tileCfg.walkableHeight, tileCfg.walkableClimb, solid);
                rcFilterWalkableLowHeightSpans(&m_rcContext, tileCfg.walkableHeight, solid);

                // add liquid triangles
                gatherBin(liquidBins[x + y * TILES_PER_MAP], lTris, lTriFlags);
                rcRasterizeTriangles(&m_rcContext, lVerts, lVertCount, binTris.data(), binAreas.data(), int(binAreas.size()), solid, config.walkableClimb);

                // compact heightfield spans
                if (!rcBuildCompactHeightfieldReuse(&m_rcContext, tileCfg.walkableHeight, tileCfg.walkableClimb, solid, scratch))
                {
                    TC_LOG_ERROR("maps.mmapgen", "{} Failed compacting heightfield!", tileString);
                    continue;
                }

                // build polymesh intermediates
                if (!rcErodeWalkableArea(&m_rcContext, config.walkableRadius, chf))
                {
                    TC_LOG_ERROR("maps.mmapgen", "{} Failed eroding area!", tileString);
                    continue;
                }

                if (!rcMedianFilterWalkableArea(&m_rcContext, chf))
                {
                    TC_LOG_ERROR("maps.mmapgen", "{} Failed filtering area!", tileString);
                    continue;
                }

                if (!rcBuildDistanceField(&m_rcContext, chf))
                {
                    TC_LOG_ERROR("maps.mmapgen", "{} Failed building distance field!", tileString);
                    continue;
                }

                if (!rcBuildRegions(&m_rcContext, chf, tileCfg.borderSize, tileCfg.minRegionArea, tileCfg.mergeRegionArea))
                {
                    TC_LOG_ERROR("maps.mmapgen", "{} Failed building regions!", tileString);
                    continue;
                }

                tile.cset = rcAllocContourSet();
                if (!tile.cset || !rcBuildContours(&m_rcContext, chf, tileCfg.maxSimplificationError, tileCfg.maxEdgeLen, *tile.cset))
                {
                    TC_LOG_ERROR("maps.mmapgen", "{} Failed building contours!", tileString);
                    continue;
                }

                // build polymesh
                tile.pmesh = rcAllocPolyMesh();
                if (!tile.pmesh || !rcBuildPolyMesh(&m_rcContext, *tile.cset, tileCfg.maxVertsPerPoly, *tile.pmesh))
                {
                    TC_LOG_ERROR("maps.mmapgen", "{} Failed building polymesh!", tileString);
                    continue;
                }

                tile.dmesh = rcAllocPolyMeshDetail();
                if (!tile.dmesh || !rcBuildPolyMeshDetail(&m_rcContext, *tile.pmesh, chf, tileCfg.detailSampleDist, tileCfg.detailSampleMaxError, *tile.dmesh))
                {
                    TC_LOG_ERROR("maps.mmapgen", "{} Failed building polymesh detail!", tileString);
                    continue;
                }

                // free those up
                // we may want to keep them in the future for debug
                // but right now, we don't have the code to merge them
                // (heightfield and compact heightfield are kept in `scratch` for the next sub-tile)
                rcFreeContourSet(tile.cset);
                tile.cset = nullptr;

                pmmerge[nmerge] = tile.pmesh;
                dmmerge[nmerge] = tile.dmesh;
                nmerge++;
            }
        }

        iv.polyMesh = rcAllocPolyMesh();
        if (!iv.polyMesh)
        {
            TC_LOG_ERROR("maps.mmapgen", "{} alloc iv.polyMesh FAILED!", tileString);
            return tileResult;
        }
        rcMergePolyMeshes(&m_rcContext, pmmerge.get(), nmerge, *iv.polyMesh);

        iv.polyMeshDetail = rcAllocPolyMeshDetail();
        if (!iv.polyMeshDetail)
        {
            TC_LOG_ERROR("maps.mmapgen", "{} alloc m_dmesh FAILED!", tileString);
            return tileResult;
        }
        rcMergePolyMeshDetails(&m_rcContext, dmmerge.get(), nmerge, *iv.polyMeshDetail);

        // free things up
        pmmerge = nullptr;
        dmmerge = nullptr;
        tiles = nullptr;

        // set polygons as walkable
        // TODO: special flags for DYNAMIC polygons, ie surfaces that can be turned on and off
        for (int i = 0; i < iv.polyMesh->npolys; ++i)
        {
            if (uint8 area = iv.polyMesh->areas[i] & NAV_AREA_ALL_MASK)
            {
                if (area >= NAV_AREA_MIN_VALUE)
                    iv.polyMesh->flags[i] = 1 << (NAV_AREA_MAX_VALUE - area);
                else
                    iv.polyMesh->flags[i] = NAV_GROUND; // TODO: these will be dynamic in future
            }
        }

        // setup mesh parameters
        dtNavMeshCreateParams params = {};
        params.verts = iv.polyMesh->verts;
        params.vertCount = iv.polyMesh->nverts;
        params.polys = iv.polyMesh->polys;
        params.polyAreas = iv.polyMesh->areas;
        params.polyFlags = iv.polyMesh->flags;
        params.polyCount = iv.polyMesh->npolys;
        params.nvp = iv.polyMesh->nvp;
        params.detailMeshes = iv.polyMeshDetail->meshes;
        params.detailVerts = iv.polyMeshDetail->verts;
        params.detailVertsCount = iv.polyMeshDetail->nverts;
        params.detailTris = iv.polyMeshDetail->tris;
        params.detailTriCount = iv.polyMeshDetail->ntris;

        params.offMeshConVerts = meshData.offMeshConnections.data();
        params.offMeshConCount = meshData.offMeshConnections.size() / 6;
        params.offMeshConRad = meshData.offMeshConnectionRads.data();
        params.offMeshConDir = meshData.offMeshConnectionDirs.data();
        params.offMeshConAreas = meshData.offMeshConnectionsAreas.data();
        params.offMeshConFlags = meshData.offMeshConnectionsFlags.data();

        params.walkableHeight = BASE_UNIT_DIM * config.walkableHeight;    // agent height
        params.walkableRadius = BASE_UNIT_DIM * config.walkableRadius;    // agent radius
        params.walkableClimb = BASE_UNIT_DIM * config.walkableClimb;      // keep less that walkableHeight (aka agent height)!
        params.tileX = (((bmin[0] + bmax[0]) / 2) - navMeshParams->orig[0]) / GRID_SIZE;
        params.tileY = (((bmin[2] + bmax[2]) / 2) - navMeshParams->orig[2]) / GRID_SIZE;
        rcVcopy(params.bmin, bmin);
        rcVcopy(params.bmax, bmax);
        params.cs = config.cs;
        params.ch = config.ch;
        params.tileLayer = 0;
        params.buildBvTree = true;

        // will hold final navmesh
        unsigned char* navData = nullptr;

        auto debugOutputWriter = Trinity::make_unique_ptr_with_deleter(m_debugOutput ? &iv : nullptr,
            [borderSize = static_cast<unsigned short>(config.borderSize),
            outputDir = &m_outputDirectory, fileNameSuffix,
            mapID, tileX, tileY, &meshData](IntermediateValues* intermediate)
        {
            // restore padding so that the debug visualization is correct
            for (std::ptrdiff_t i = 0; i < intermediate->polyMesh->nverts; ++i)
            {
                unsigned short* v = &intermediate->polyMesh->verts[i * 3];
                v[0] += borderSize;
                v[2] += borderSize;
            }

            intermediate->generateObjFile(*outputDir, fileNameSuffix, mapID, tileX, tileY, meshData);
            intermediate->writeIV(*outputDir, fileNameSuffix, mapID, tileX, tileY);
        });

        // these values are checked within dtCreateNavMeshData - handle them here
        // so we have a clear error message
        if (params.nvp > DT_VERTS_PER_POLYGON)
        {
            TC_LOG_ERROR("maps.mmapgen", "{} Invalid verts-per-polygon value!", tileString);
            return tileResult;
        }

        if (params.vertCount >= 0xffff)
        {
            TC_LOG_ERROR("maps.mmapgen", "{} Too many vertices!", tileString);
            return tileResult;
        }

        if (!params.vertCount || !params.verts)
        {
            // occurs mostly when adjacent tiles have models
            // loaded but those models don't span into this tile

            // message is an annoyance
            //TC_LOG_ERROR("maps.mmapgen", "{} No vertices to build tile!", tileString);
            return tileResult;
        }

        if (!params.polyCount || !params.polys)
        {
            // we have flat tiles with no actual geometry - don't build those, its useless
            // keep in mind that we do output those into debug info
            TC_LOG_ERROR("maps.mmapgen", "{} No polygons to build on tile!", tileString);
            return tileResult;
        }

        if (!params.detailMeshes || !params.detailVerts || !params.detailTris)
        {
            TC_LOG_ERROR("maps.mmapgen", "{} No detail mesh to build tile!", tileString);
            return tileResult;
        }

        TC_LOG_DEBUG("maps.mmapgen", "{} Building navmesh tile...", tileString);
        if (!dtCreateNavMeshData(&params, &navData, &tileResult.size))
        {
            TC_LOG_ERROR("maps.mmapgen", "{} Failed building navmesh tile!", tileString);
            return tileResult;
        }

        tileResult.data.reset(navData);
        return tileResult;
    }

    void TileBuilder::saveMoveMapTileToFile(uint32 mapID, uint32 tileX, uint32 tileY, dtNavMesh* navMesh,
        TileResult const& tileResult, std::string_view fileNameSuffix)
    {
        dtTileRef tileRef = 0;
        auto navMeshTile = Trinity::make_unique_ptr_with_deleter<dtTileRef*>(nullptr, [navMesh](dtTileRef const* ref)
        {
            navMesh->removeTile(*ref, nullptr, nullptr);
        });

        if (navMesh)
        {
            TC_LOG_DEBUG("maps.mmapgen", "[Map {:04}] [{:02},{:02}]: Adding tile to navmesh...", mapID, tileX, tileY);
            // DT_TILE_FREE_DATA tells detour to unallocate memory when the tile
            // is removed via removeTile()
            dtStatus dtResult = navMesh->addTile(tileResult.data.get(), tileResult.size, 0, 0, &tileRef);
            if (!tileRef || !dtStatusSucceed(dtResult))
            {
                TC_LOG_ERROR("maps.mmapgen", "[Map {:04}] [{:02},{:02}]: Failed adding tile to navmesh!", mapID, tileX, tileY);
                return;
            }

            navMeshTile.reset(&tileRef);
        }

        // file output
        std::string fileName = Trinity::StringFormat("{}/mmaps/{:04}_{:02}_{:02}{}.mmtile", m_outputDirectory.generic_string(), mapID, tileX, tileY, fileNameSuffix);
        auto file = Trinity::make_unique_ptr_with_deleter<&::fclose>(fopen(fileName.c_str(), "wb"));
        if (!file)
        {
            TC_LOG_ERROR("maps.mmapgen", "[Map {:04}] [{:02},{:02}]: {}: Failed to open {} for writing!", mapID, tileX, tileY, strerror(errno), fileName);
            return;
        }

        TC_LOG_DEBUG("maps.mmapgen", "[Map {:04}] [{:02},{:02}]: Writing to file...", mapID, tileX, tileY);

        // write header
        MmapTileHeader header;
        header.usesLiquids = m_terrainBuilder.usesLiquids();
        header.size = uint32(tileResult.size);
        fwrite(&header, sizeof(MmapTileHeader), 1, file.get());

        // write data
        fwrite(tileResult.data.get(), sizeof(unsigned char), tileResult.size, file.get());
    }

    /**************************************************************************/
    void TileBuilder::getTileBounds(uint32 tileX, uint32 tileY, float const* verts, std::size_t vertCount, float* bmin, float* bmax)
    {
        // this is for elevation
        if (verts && vertCount)
            rcCalcBounds(verts, int(vertCount), bmin, bmax);
        else
        {
            bmin[1] = FLT_MIN;
            bmax[1] = FLT_MAX;
        }

        // this is for width and depth
        bmax[0] = (32 - int(tileY)) * GRID_SIZE;
        bmax[2] = (32 - int(tileX)) * GRID_SIZE;
        bmin[0] = bmax[0] - GRID_SIZE;
        bmin[2] = bmax[2] - GRID_SIZE;
    }

    /**************************************************************************/
    bool TileBuilder::shouldSkipTile(uint32 /*mapID*/, uint32 /*tileX*/, uint32 /*tileY*/) const
    {
        if (m_debugOutput)
            return false;

        return true;
    }

    rcConfig TileBuilder::GetMapSpecificConfig(uint32 mapID, float const (&bmin)[3], float const (&bmax)[3], TileConfig const& tileConfig) const
    {
        rcConfig config { };

        rcVcopy(config.bmin, bmin);
        rcVcopy(config.bmax, bmax);

        config.maxVertsPerPoly = DT_VERTS_PER_POLYGON;
        config.cs = tileConfig.BASE_UNIT_DIM;
        config.ch = tileConfig.BASE_UNIT_DIM;
        // Keeping these 2 slope angles the same reduces a lot the number of polys.
        // 55 should be the minimum, maybe 70 is ok (keep in mind blink uses mmaps), 85 is too much for players
        config.walkableSlopeAngle = m_maxWalkableAngle.value_or(55.0f);
        config.walkableSlopeAngleNotSteep = m_maxWalkableAngleNotSteep.value_or(55.0f);
        config.tileSize = tileConfig.VERTEX_PER_TILE;
        config.walkableRadius = m_bigBaseUnit ? 1 : 2;
        config.borderSize = config.walkableRadius + 3;
        config.maxEdgeLen = tileConfig.VERTEX_PER_TILE + 1;        // anything bigger than tileSize
        config.walkableHeight = m_bigBaseUnit ? 3 : 6;
        // a value >= 3|6 allows npcs to walk over some fences
        // a value >= 4|8 allows npcs to walk over all fences
        config.walkableClimb = m_bigBaseUnit ? 3 : 6;
        config.minRegionArea = rcSqr(60);
        config.mergeRegionArea = rcSqr(50);
        config.maxSimplificationError = 1.8f;           // eliminates most jagged edges (tiny polygons)
        config.detailSampleDist = config.cs * 16;
        config.detailSampleMaxError = config.ch * 1;

        switch (mapID)
        {
            // Blade's Edge Arena
            case 562:
                // This allows to walk on the ropes to the pillars
                config.walkableRadius = 0;
                break;
            // Blackfathom Deeps
            case 48:
                // Reduce the chance to have underground levels
                config.ch *= 2;
                break;
            default:
                break;
        }

        return config;
    }

    std::string TileBuilder::GetProgressText() const
    {
        return "";
    }
}
