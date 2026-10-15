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

#include "Incremental.h"
#include "Log.h"
#include "PathCommon.h"
#include "StringFormat.h"
#include <boost/filesystem/operations.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <memory>
#include <tuple>
#if defined(__linux__)
#include <fcntl.h>
#include <linux/fs.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace
{
    // bump when anything in the tile pipeline changes that the digest inputs do not see (Recast/Detour code, config
    // constants such as walkableHeight, the per-map overrides in GetMapSpecificConfig, mesh assembly)
    constexpr char kToolVersion[] = "mmaps_generator_fast-g9-1";

    class Sha256
    {
    public:
        Sha256() { Reset(); }

        void Update(void const* data, std::size_t len)
        {
            uint8_t const* p = static_cast<uint8_t const*>(data);
            _total += len;
            while (len)
            {
                std::size_t take = std::min<std::size_t>(len, 64 - _fill);
                std::memcpy(_buf + _fill, p, take);
                _fill += take; p += take; len -= take;
                if (_fill == 64)
                {
                    Block(_buf);
                    _fill = 0;
                }
            }
        }

        void Update(std::string const& s) { Update(s.data(), s.size()); }

        std::string Final()
        {
            uint64_t bits = _total * 8;
            uint8_t pad = 0x80;
            Update(&pad, 1);
            uint8_t zero = 0;
            while (_fill != 56)
                Update(&zero, 1);
            uint8_t lenBytes[8];
            for (int i = 0; i < 8; ++i)
                lenBytes[i] = uint8_t(bits >> (56 - 8 * i));
            Update(lenBytes, 8);
            static char const* hex = "0123456789abcdef";
            std::string out;
            for (uint32_t h : _h)
                for (int i = 3; i >= 0; --i)
                {
                    uint8_t b = uint8_t(h >> (8 * i));
                    out.push_back(hex[b >> 4]);
                    out.push_back(hex[b & 15]);
                }
            return out;
        }

    private:
        void Reset()
        {
            static constexpr uint32_t init[8] = { 0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19 };
            std::memcpy(_h, init, sizeof(_h));
            _fill = 0;
            _total = 0;
        }

        static uint32_t Rotr(uint32_t x, int n) { return (x >> n) | (x << (32 - n)); }

        void Block(uint8_t const* p)
        {
            static constexpr uint32_t K[64] =
            {
                0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
                0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
                0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
                0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
                0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
                0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
                0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
                0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
            };
            uint32_t w[64];
            for (int i = 0; i < 16; ++i)
                w[i] = (uint32_t(p[i*4]) << 24) | (uint32_t(p[i*4+1]) << 16) | (uint32_t(p[i*4+2]) << 8) | uint32_t(p[i*4+3]);
            for (int i = 16; i < 64; ++i)
            {
                uint32_t s0 = Rotr(w[i-15], 7) ^ Rotr(w[i-15], 18) ^ (w[i-15] >> 3);
                uint32_t s1 = Rotr(w[i-2], 17) ^ Rotr(w[i-2], 19) ^ (w[i-2] >> 10);
                w[i] = w[i-16] + s0 + w[i-7] + s1;
            }
            uint32_t a = _h[0], b = _h[1], c = _h[2], d = _h[3], e = _h[4], f = _h[5], g = _h[6], h = _h[7];
            for (int i = 0; i < 64; ++i)
            {
                uint32_t S1 = Rotr(e, 6) ^ Rotr(e, 11) ^ Rotr(e, 25);
                uint32_t ch = (e & f) ^ (~e & g);
                uint32_t t1 = h + S1 + ch + K[i] + w[i];
                uint32_t S0 = Rotr(a, 2) ^ Rotr(a, 13) ^ Rotr(a, 22);
                uint32_t mj = (a & b) ^ (a & c) ^ (b & c);
                uint32_t t2 = S0 + mj;
                h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
            }
            _h[0] += a; _h[1] += b; _h[2] += c; _h[3] += d; _h[4] += e; _h[5] += f; _h[6] += g; _h[7] += h;
        }

        uint32_t _h[8];
        uint8_t _buf[64];
        std::size_t _fill = 0;
        uint64_t _total = 0;
    };

    std::string Sha256Hex(std::string const& s)
    {
        Sha256 h;
        h.Update(s);
        return h.Final();
    }

    uint64 TileKey(uint32 mapID, uint32 tileX, uint32 tileY)
    {
        return (uint64(mapID) << 24) | (uint64(tileX & 0xFFF) << 12) | uint64(tileY & 0xFFF);
    }

    std::string Mmtile(boost::filesystem::path const& root, uint32 mapID, uint32 tileX, uint32 tileY)
    {
        return Trinity::StringFormat("{}/mmaps/{:04}_{:02}_{:02}.mmtile", root.generic_string(), mapID, tileX, tileY);
    }

    bool CloneFile(std::string const& src, std::string const& dst)
    {
#if defined(__linux__)
        int in = open(src.c_str(), O_RDONLY);
        if (in < 0)
            return false;
        int out = open(dst.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0666);
        if (out < 0)
        {
            close(in);
            return false;
        }

        bool ok = ioctl(out, FICLONE, in) == 0;     // reflink: shares the extents (btrfs, xfs)
        if (!ok)
        {
            // normal copy
            ok = true;
            std::array<char, 1 << 16> buf;
            for (;;)
            {
                ssize_t n = read(in, buf.data(), buf.size());
                if (n < 0) { ok = false; break; }
                if (n == 0) break;
                char const* p = buf.data();
                while (n > 0)
                {
                    ssize_t w = write(out, p, std::size_t(n));
                    if (w <= 0) { ok = false; break; }
                    p += w; n -= w;
                }
                if (!ok) break;
            }
        }
        close(in);
        if (close(out) != 0)
            ok = false;
        if (!ok)
            unlink(dst.c_str());
        return ok;
#else
        boost::system::error_code ec;
        boost::filesystem::copy_file(src, dst, boost::filesystem::copy_options::overwrite_existing, ec);
        return !ec;
#endif
    }

    using FilePtr = std::unique_ptr<std::FILE, int(*)(std::FILE*)>;

    // %a hex floats are exact, so the text is a lossless, human readable form of the bits
    std::string Floats(float const* f, int n)
    {
        std::string s;
        for (int i = 0; i < n; ++i)
            s += Trinity::StringFormat("{}{:a}", i ? "," : "", f[i]);
        return s;
    }
}

namespace MMAP
{
    IncrementalOptions sIncrementalOptions;

    namespace { IncrementalState* sState = nullptr; }

    IncrementalState* IncrementalState::Get() { return sState; }

    void IncrementalState::Init(boost::filesystem::path const& inputDir, boost::filesystem::path const& outputDir, std::string const& globalInputs)
    {
        if (!sIncrementalOptions.incremental && !sIncrementalOptions.writeSidecar && !sIncrementalOptions.dumpInputs)
            return;

        sState = new IncrementalState();
        IncrementalState& s = *sState;
        s._inputDir = inputDir;
        s._outputDir = outputDir;
        s._globalDigest = Sha256Hex(std::string(kToolVersion) + "\n" + globalInputs);
        s._sidecarPath = Trinity::StringFormat("{}/mmaps/.inputs.tsv", outputDir.generic_string());

        if (sIncrementalOptions.incremental)
        {
            s._prevDir = sIncrementalOptions.prevDir;
            boost::system::error_code e1;
            s._inPlace = boost::filesystem::equivalent(s._prevDir / "mmaps", outputDir / "mmaps", e1) && !e1;

            std::string prevSidecar = Trinity::StringFormat("{}/mmaps/.inputs.tsv", s._prevDir.generic_string());
            FilePtr f(std::fopen(prevSidecar.c_str(), "rb"), &std::fclose);
            if (!f)
                TC_LOG_ERROR("maps.mmapgen", "--incremental: cannot read {}; every tile is rebuilt", prevSidecar);
            else
            {
                char line[512];
                uint32 rows = 0;
                while (std::fgets(line, sizeof(line), f.get()))
                {
                    if (line[0] == '#')
                        continue;
                    unsigned map, x, y;
                    char digest[80], kind[16];
                    if (std::sscanf(line, "%u\t%u\t%u\t%79s\t%15s", &map, &x, &y, digest, kind) != 5)
                        continue;
                    s._prev[TileKey(map, x, y)] = { digest, std::strcmp(kind, "tile") == 0 };
                    ++rows;
                }
                TC_LOG_INFO("maps.mmapgen", "--incremental: {} tile digests read from {}{}", rows, prevSidecar, s._inPlace ? " (in place)" : "");
            }
        }

        if (sIncrementalOptions.dumpInputs)
            return;     // nothing is built or written

        // fresh sidecar for this run (prev, if it is this very file, is already in memory)
        s._sidecar = std::fopen(s._sidecarPath.c_str(), "wb");
        if (!s._sidecar)
        {
            TC_LOG_ERROR("maps.mmapgen", "cannot write {}", s._sidecarPath);
            return;
        }
        std::fprintf(s._sidecar, "#mmaps_inputs\tv1\t%s\t%s\n", kToolVersion, s._globalDigest.c_str());
        std::fflush(s._sidecar);
    }

    void IncrementalState::Finish()
    {
        IncrementalState* s = sState;
        if (!s)
            return;

        TC_LOG_INFO("maps.mmapgen", "incremental: {} tiles reused from prev, {} built or empty", s->_reused.load(), s->_built.load());

        if (!s->_sidecar)
            return;

        std::fclose(s->_sidecar);
        s->_sidecar = nullptr;

        // rewrite sorted so two runs produce the same sidecar
        std::vector<std::string> rows;
        {
            FilePtr f(std::fopen(s->_sidecarPath.c_str(), "rb"), &std::fclose);
            if (!f)
                return;
            char line[512];
            std::string header;
            while (std::fgets(line, sizeof(line), f.get()))
            {
                if (line[0] == '#')
                    header = line;
                else
                    rows.emplace_back(line);
            }
            std::sort(rows.begin(), rows.end(), [](std::string const& a, std::string const& b)
            {
                unsigned am, ax, ay, bm, bx, by;
                std::sscanf(a.c_str(), "%u\t%u\t%u", &am, &ax, &ay);
                std::sscanf(b.c_str(), "%u\t%u\t%u", &bm, &bx, &by);
                return std::tie(am, ax, ay) < std::tie(bm, bx, by);
            });
            std::string tmp = s->_sidecarPath + ".tmp";
            FilePtr out(std::fopen(tmp.c_str(), "wb"), &std::fclose);
            if (!out)
                return;
            std::fputs(header.c_str(), out.get());
            for (std::string const& r : rows)
                std::fputs(r.c_str(), out.get());
            out.reset();
            boost::system::error_code ec;
            boost::filesystem::rename(tmp, s->_sidecarPath, ec);
        }
    }

    std::string IncrementalState::FileDigest(std::string const& path)
    {
        {
            std::lock_guard<std::mutex> lock(_fileCacheLock);
            auto itr = _fileCache.find(path);
            if (itr != _fileCache.end())
                return itr->second;
        }

        std::string digest;
        FilePtr f(std::fopen(path.c_str(), "rb"), &std::fclose);
        if (!f)
            digest = "missing";
        else
        {
            Sha256 h;
            std::array<char, 1 << 16> buf;
            std::size_t n;
            while ((n = std::fread(buf.data(), 1, buf.size(), f.get())) > 0)
                h.Update(buf.data(), n);
            digest = h.Final();
        }

        std::lock_guard<std::mutex> lock(_fileCacheLock);
        _fileCache.emplace(path, digest);
        return digest;
    }

    std::string IncrementalState::ComputeTileDigest(uint32 mapID, uint32 tileX, uint32 tileY, dtNavMeshParams const& params,
        std::vector<OffMeshData> const* offMeshConnections, std::string* dump)
    {
        Sha256 hash;
        auto add = [&](std::string const& kind, std::string const& value)
        {
            std::string line = kind + "\t" + value + "\n";
            hash.Update(line);
            if (dump)
                *dump += line;
        };

        // 1. the tool and its global configuration (args, LiquidType SoundBank map, tool version)
        add("tool", _globalDigest);

        // 2. the navmesh params of the map the tile is added to: orig comes from the tile set of the root parent map
        add("navmesh", Trinity::StringFormat("orig={} tileW={:a} tileH={:a} maxTiles={} maxPolys={}", Floats(params.orig, 3),
            params.tileWidth, params.tileHeight, params.maxTiles, params.maxPolys));
        add("tile", Trinity::StringFormat("map={} x={} y={}", mapID, tileX, tileY));

        // 3. the Map.db2 fields used (MapType/InstanceType/ParentMapID(with the Cosmetic fallback)/Flags1) along the parent chain
        std::vector<uint32> chain;
        for (uint32 id = mapID;;)
        {
            chain.push_back(id);
            auto itr = sMapStore.find(id);
            int32 parent = itr != sMapStore.end() ? itr->second.ParentMapID : -1;
            add("mapdb2", Trinity::StringFormat("id={} type={} instance={} parent={} flags={}", id,
                itr != sMapStore.end() ? int(itr->second.MapType) : -1, itr != sMapStore.end() ? int(itr->second.InstanceType) : -1,
                parent, itr != sMapStore.end() ? itr->second.Flags : 0));
            if (parent == -1 || chain.size() > 16)
                break;
            id = uint32(parent);
        }

        // 4. the .map file that loadMap reads for the tile and its 4 neighbours (own map, else the first parent that has it);
        //    the order/wrapping matches TerrainBuilder::loadMap
        struct Neighbour { uint32 X, Y; char const* Name; };
        Neighbour const neighbours[] = { { tileX, tileY, "self" }, { tileX, tileY + 1, "y+1" }, { tileX, tileY - 1, "y-1" },
            { tileX + 1, tileY, "x+1" }, { tileX - 1, tileY, "x-1" } };
        for (Neighbour const& n : neighbours)
        {
            std::string used = "none";
            std::string digest = "missing";
            for (uint32 id : chain)
            {
                std::string path = Trinity::StringFormat("{}/maps/{:04}_{:02}_{:02}.map", _inputDir.generic_string(), id, n.X, n.Y);
                std::string d = FileDigest(path);
                if (d != "missing")
                {
                    used = Trinity::StringFormat("{:04}_{:02}_{:02}.map", id, n.X, n.Y);
                    digest = d;
                    break;
                }
            }
            add("map", Trinity::StringFormat("{} {} {}", n.Name, used, digest));
        }

        // 5. the vmaps: the map's .vmtree (only its object count is used: the bound of a tile's tree indices), the .vmtileidx of
        //    the map, the .vmtile (own map, else the first parent that has it), and every .vmo the tile's spawns name.
        //    A .vmtile is digested by content: the spawns are taken in tree-index order (the order loadVMap meshes them in),
        //    without the unique id and the index values, which a re-extraction renumbers (vmap4extractor hands out the ids in
        //    thread completion order) without changing the geometry.
        std::string vmapBase = Trinity::StringFormat("{}/vmaps/", _inputDir.generic_string());
        {
            std::string treePath = Trinity::StringFormat("{}{:04}/{:04}.vmtree", vmapBase, mapID, mapID);
            FilePtr f(std::fopen(treePath.c_str(), "rb"), &std::fclose);
            std::string info = "missing";
            if (f)
            {
                char magic[8], node[4];
                float bounds[6];
                uint32 treeSize = 0, count = 0;
                bool ok = std::fread(magic, 1, 8, f.get()) == 8 && std::fread(node, 1, 4, f.get()) == 4 && std::memcmp(node, "NODE", 4) == 0
                    && std::fread(bounds, sizeof(float), 6, f.get()) == 6 && std::fread(&treeSize, 4, 1, f.get()) == 1
                    && std::fseek(f.get(), long(treeSize) * 4, SEEK_CUR) == 0 && std::fread(&count, 4, 1, f.get()) == 1;
                info = ok ? Trinity::StringFormat("magic={} primCount={}", std::string(magic, 8), count) : "unreadable:" + FileDigest(treePath);
            }
            add("vmtree", info);
        }

        std::string idxPath = Trinity::StringFormat("{}{:04}/{:04}_{:02}_{:02}.vmtileidx", vmapBase, mapID, mapID, tileX, tileY);
        std::string tilePath, tileName = "none";
        for (uint32 id : chain)
        {
            std::string path = Trinity::StringFormat("{}{:04}/{:04}_{:02}_{:02}.vmtile", vmapBase, id, id, tileX, tileY);
            if (boost::filesystem::exists(path))
            {
                tilePath = path;
                tileName = Trinity::StringFormat("{:04}", id);
                break;
            }
        }

        std::vector<std::string> modelNames;
        FilePtr tf(tilePath.empty() ? nullptr : std::fopen(tilePath.c_str(), "rb"), &std::fclose);
        FilePtr idx(std::fopen(idxPath.c_str(), "rb"), &std::fclose);
        if (!tf || !idx)
            add("vmtile", Trinity::StringFormat("from={} tile={} idx={}", tileName, tf ? "present" : "missing", idx ? "present" : "missing"));
        else
        {
            struct Spawn { uint32 Index; uint32 Order; std::string Text; };
            std::vector<Spawn> spawns;
            bool ok = true;
            char magic[8], magic2[8];
            uint32 n1 = 0, n2 = 0;
            ok = std::fread(magic, 1, 8, tf.get()) == 8 && std::fread(magic2, 1, 8, idx.get()) == 8
                && std::fread(&n1, 4, 1, tf.get()) == 1 && std::fread(&n2, 4, 1, idx.get()) == 1 && n1 == n2 && n1 < (1u << 24);
            for (uint32 i = 0; ok && i < n1; ++i)
            {
                uint8 flags = 0, adtId = 0;
                uint32 id = 0, nameLen = 0, referenced = 0;
                float pos[3], rot[3], scale, bound[6] = { };
                ok = std::fread(&flags, 1, 1, tf.get()) == 1 && std::fread(&adtId, 1, 1, tf.get()) == 1 && std::fread(&id, 4, 1, tf.get()) == 1
                    && std::fread(pos, 4, 3, tf.get()) == 3 && std::fread(rot, 4, 3, tf.get()) == 3 && std::fread(&scale, 4, 1, tf.get()) == 1;
                bool hasBound = (flags & 0x01) != 0;     // MOD_HAS_BOUND, see ModelInstance.h
                if (ok && hasBound)
                    ok = std::fread(bound, 4, 6, tf.get()) == 6;
                ok = ok && std::fread(&nameLen, 4, 1, tf.get()) == 1 && nameLen <= 500;
                std::string name(nameLen, '\0');
                ok = ok && (nameLen == 0 || std::fread(name.data(), 1, nameLen, tf.get()) == nameLen);
                ok = ok && std::fread(&referenced, 4, 1, idx.get()) == 1;
                if (!ok)
                    break;
                // adtId and id deliberately left out (see above); flags/pos/rot/scale/bound/name are what ModelInstance keeps
                spawns.push_back({ referenced, i, Trinity::StringFormat("flags={} pos={} rot={} scale={:a} bound={} name={}", unsigned(flags),
                    Floats(pos, 3), Floats(rot, 3), scale, hasBound ? Floats(bound, 6) : "-", name) });
                modelNames.push_back(name);
            }

            if (!ok)
            {
                // anything irregular: fall back to the raw bytes of both files (a re-extraction then rebuilds the tile)
                add("vmtile", Trinity::StringFormat("raw from={} tile={} idx={}", tileName, FileDigest(tilePath), FileDigest(idxPath)));
                modelNames.clear();
            }
            else
            {
                std::sort(spawns.begin(), spawns.end(), [](Spawn const& a, Spawn const& b) { return std::tie(a.Index, a.Order) < std::tie(b.Index, b.Order); });
                add("vmtile", Trinity::StringFormat("from={} spawns={} magic={}", tileName, spawns.size(), std::string(magic, 8)));
                for (Spawn const& s : spawns)
                    add("spawn", s.Text);
            }
        }

        std::sort(modelNames.begin(), modelNames.end());
        modelNames.erase(std::unique(modelNames.begin(), modelNames.end()), modelNames.end());
        for (std::string const& name : modelNames)
            add("vmo", name + " " + FileDigest(vmapBase + name + ".vmo"));

        // 6. off-mesh connections of the tile
        if (offMeshConnections)
            for (OffMeshData const& o : *offMeshConnections)
                if (o.MapId == mapID && o.TileX == tileX && o.TileY == tileY)
                    add("offmesh", Trinity::StringFormat("from={} to={} radius={:a} conn={} area={} flags={}", Floats(o.From, 3), Floats(o.To, 3),
                        o.Radius, unsigned(o.ConnectionFlags), unsigned(o.AreaId), unsigned(o.Flags)));

        return hash.Final();
    }

    void IncrementalState::AppendRow(uint32 mapID, uint32 tileX, uint32 tileY, std::string const& digest, bool hasTile)
    {
        std::lock_guard<std::mutex> lock(_sidecarLock);
        if (!_sidecar)
            return;
        std::fprintf(_sidecar, "%u\t%u\t%u\t%s\t%s\n", mapID, tileX, tileY, digest.c_str(), hasTile ? "tile" : "none");
        std::fflush(_sidecar);
    }

    bool IncrementalState::TryReuse(uint32 mapID, uint32 tileX, uint32 tileY, std::string const& digest)
    {
        if (!sIncrementalOptions.incremental)
            return false;

        auto itr = _prev.find(TileKey(mapID, tileX, tileY));
        if (itr == _prev.end() || itr->second.Digest != digest)
            return false;

        std::string out = Mmtile(_outputDir, mapID, tileX, tileY);
        boost::system::error_code ec;
        if (itr->second.HasTile)
        {
            std::string src = Mmtile(_prevDir, mapID, tileX, tileY);
            if (!boost::filesystem::is_regular_file(src, ec))
                return false;
            if (!_inPlace && !CloneFile(src, out))
                return false;
        }
        else
            boost::filesystem::remove(out, ec);     // a full build would not have produced a tile either

        AppendRow(mapID, tileX, tileY, digest, itr->second.HasTile);
        ++_reused;
        return true;
    }

    void IncrementalState::Record(uint32 mapID, uint32 tileX, uint32 tileY, std::string const& digest, bool hasTile)
    {
        boost::system::error_code ec;
        std::string out = Mmtile(_outputDir, mapID, tileX, tileY);
        if (hasTile && !boost::filesystem::is_regular_file(out, ec))
            return;     // the write failed: leave the tile unrecorded so it is rebuilt next time
        if (!hasTile)
            boost::filesystem::remove(out, ec);

        AppendRow(mapID, tileX, tileY, digest, hasTile);
        ++_built;
    }
}
