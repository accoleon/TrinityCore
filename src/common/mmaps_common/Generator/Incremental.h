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

#ifndef TRINITYCORE_MMAPS_INCREMENTAL_H
#define TRINITYCORE_MMAPS_INCREMENTAL_H

#include "Define.h"
#include "MMapDefines.h"
#include <DetourNavMesh.h>
#include <boost/filesystem/path.hpp>
#include <atomic>
#include <cstdio>
#include <functional>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

// Incremental regeneration. A tile's .mmtile is a pure function of the inputs listed in
// Incremental.cpp (ComputeTileDigest). The digest of every tile is kept in a sidecar, mmaps/.inputs.tsv; with
// --incremental <prev_dir> a tile whose digest equals the one in prev_dir's sidecar (and whose prev .mmtile exists,
// or was recorded as "no tile") is cloned from prev instead of built.
namespace MMAP
{
    struct IncrementalOptions
    {
        bool incremental = false;       // --incremental <prev_dir>
        bool writeSidecar = false;      // --write-inputs (implied by --incremental)
        bool dumpInputs = false;        // --dump-inputs: print every requested tile's input list, build nothing
        boost::filesystem::path prevDir;
    };

    TC_MMAPS_COMMON_API extern IncrementalOptions sIncrementalOptions;

    /// The Map.db2 fields that go into a tile's digest; the tool supplies them (they live in the tool's map store)
    struct IncrementalMapInfo
    {
        int MapType = -1;
        int InstanceType = -1;
        int32 ParentMapID = -1;
        int32 Flags = 0;
    };
    using IncrementalMapLookup = std::function<bool(uint32 mapId, IncrementalMapInfo& info)>;

    class TC_MMAPS_COMMON_API IncrementalState
    {
    public:
        // process-wide instance, null unless one of the options is set and Init() ran
        static IncrementalState* Get();
        // globalInputs: text covering everything that is not per tile (tool args, LiquidType map, ...)
        static void Init(boost::filesystem::path const& inputDir, boost::filesystem::path const& outputDir, std::string const& globalInputs, IncrementalMapLookup mapLookup);
        // rewrites the sidecar sorted (rows are appended while tiles complete)
        static void Finish();

        // digest + the human readable input list (dump != nullptr)
        std::string ComputeTileDigest(uint32 mapID, uint32 tileX, uint32 tileY, dtNavMeshParams const& params,
            std::vector<OffMeshData> const* offMeshConnections, std::string* dump);

        // true: the tile is settled from prev (its .mmtile cloned, or prev recorded that no tile results) and was recorded
        bool TryReuse(uint32 mapID, uint32 tileX, uint32 tileY, std::string const& digest);
        // a tile was built (hasTile: an .mmtile was written) or found empty
        void Record(uint32 mapID, uint32 tileX, uint32 tileY, std::string const& digest, bool hasTile);

        uint32 Reused() const { return _reused; }
        uint32 Built() const { return _built; }

    private:
        struct PrevRow { std::string Digest; bool HasTile; };

        std::string FileDigest(std::string const& path);
        void AppendRow(uint32 mapID, uint32 tileX, uint32 tileY, std::string const& digest, bool hasTile);

        boost::filesystem::path _inputDir;
        boost::filesystem::path _outputDir;
        boost::filesystem::path _prevDir;
        std::string _globalDigest;
        IncrementalMapLookup _mapLookup;
        bool _inPlace = false;
        std::unordered_map<uint64, PrevRow> _prev;
        std::mutex _fileCacheLock;
        std::unordered_map<std::string, std::string> _fileCache;
        std::mutex _sidecarLock;
        std::FILE* _sidecar = nullptr;
        std::atomic<uint32> _reused{ 0 };
        std::atomic<uint32> _built{ 0 };
        std::string _sidecarPath;
    };
}

#endif
