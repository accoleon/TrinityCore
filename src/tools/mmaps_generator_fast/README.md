# mmaps_generator_fast (experiment, item 535)

`src/tools/mmaps_generator/` plus `src/common/mmaps_common/Generator/` (and `MMapDefines.h`) copied
verbatim at upstream 34fd0ebd83, built as its own executable that does not link `mmaps_common`.
Commit 1 is the pure copy: its output must be byte-identical to `mmaps_generator`. The optimisations
below land one per commit, each a candidate small upstream PR (numbering from
`docs/history/mmaps-perf-535-2026-10-14.md` section 3 of the wowmidnight repo). Every commit is gated
by `mmapbench compare` (byte-identical .mmtile/.mmap).

- G1 `TileBuilder::buildMoveMapTile`: compute `triFlags` (and liquid flags) once per ADT tile, before
  the 25x25 sub-tile loop, and reuse them.
- G2 same function: bin solid and liquid triangles into per-sub-tile index lists once (bbox vs the
  sub-tile bounds incl. border) and rasterize only that list; keep triangle order within a sub-tile.
- G3 same loop: skip a sub-tile entirely when its binned lists are empty / its heightfield has no spans.
- G4 `PathGenerator.cpp:CreateVMapManager` + `TileBuilder::buildTile`: one `VMapManager` per worker
  per map so `.vmo` parses are cached across tiles (needs a keep-loaded flag for the model refcount).
- G5 `TerrainBuilder::loadMap`: cache the decoded neighbour `.map` height/liquid blocks per worker
  (or read each file once) instead of ~5 decodes per tile.
