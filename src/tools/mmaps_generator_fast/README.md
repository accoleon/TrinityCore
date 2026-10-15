# mmaps_generator_fast (experiment, item 535)

`src/tools/mmaps_generator/` plus `src/common/mmaps_common/Generator/` (and `MMapDefines.h`) copied
verbatim at upstream 34fd0ebd83, built as its own executable that does not link `mmaps_common`.
Commit 1 is the pure copy: its output must be byte-identical to `mmaps_generator`. The optimisations
below land one per commit, each a candidate small upstream PR (numbering from
`docs/history/mmaps-perf-535-2026-10-14.md` section 3 of the wowmidnight repo). Every commit is gated
by `mmapbench compare` (byte-identical .mmtile/.mmap).

Status: G1 done (commit "G1"), G2 done (commit labelled "G3": per-sub-tile triangle binning, solid and
liquid lists, XZ bbox vs sub-tile bounds incl. border, original triangle order kept; empty bins still run
the pipeline). G3/G4/G5 follow below. Numbering is the doc's; the commit subject "G3 bin triangles..." is
really doc G2.

- G1 `TileBuilder::buildMoveMapTile`: compute `triFlags` (and liquid flags) once per ADT tile, before
  the 25x25 sub-tile loop, and reuse them.
- G2 same function: bin solid and liquid triangles into per-sub-tile index lists once (bbox vs the
  sub-tile bounds incl. border) and rasterize only that list; keep triangle order within a sub-tile.
- G3 same loop (done): a sub-tile whose binned solid and liquid lists are both empty skips the Recast pipeline
  and pushes a header-only polymesh stub (bmin/bmax/cs/ch/nvp as rcBuildContours/PolyMesh derive them) so the
  merged bounds, and therefore every other sub-tile's offsets and border flags, are unchanged; its detail
  mesh entry is nullptr (rcMergePolyMeshDetails skips those). Sub-tiles with triangles that yield no spans
  still run the full pipeline.
- G4 (done) `TileBuilder::buildTile` + `TerrainBuilder::loadVMap`: one `VMapManager` per worker (TileBuilder) for
  the map it is building, dropped when the worker moves to another map. The manager only holds weak references
  to loaded models, so `loadVMap` pins each tile's `WorldModel` shared_ptr in a per-worker cache (bounded at 4096
  distinct models, then cleared) and the next tile finds the parsed model. Per-tile `unloadMap(map,x,y)` is unchanged.
- G5 (done) `TerrainBuilder::loadMap`: the .map read + decode is split from the mesh assembly; decoded tiles
  (V9/V8 heights, holes, liquid header/flags/heights) live in an immutable, process-wide LRU (1024 entries, key
  input dir|skipLiquid|map|x|y, mutex-guarded, decode outside the lock). Shared rather than per worker because
  neighbouring tiles go to different workers. Missing/invalid files are not cached. Assembly order is unchanged.
