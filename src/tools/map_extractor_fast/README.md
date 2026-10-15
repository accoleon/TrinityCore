# mapextractor_fast (experiment, item 535 G7)

Copy of `src/tools/map_extractor` (separate executable `mapextractor_fast`, same CLI plus `-t <threads>`, 0 = all cores).
Only the MAP step differs: all WDTs are read serially first, then every ADT tile of every map is converted by a thread
pool (atomic task index), then the per-map `.tilelist` files are written serially. dbc/camera/gt extraction is untouched.
CASC: the serial phases (WDT, dbc, camera, gt) use the main storage; every worker opens its own `CASC::Storage` lazily on its
first task (independent CascLib handles, no lock; if a worker's open fails it falls back to the main storage under `CascLock`).
Each storage holds its own index/encoding/root tables, so the default worker count is capped at 8 (`-t N` overrides). After the
pool joins, the tool prints the summed open / CASC load / convert+write time per worker. Each task touches only its own `.map`
file and its own tile-exists byte. The converter's
other inputs (map_ids, Liquid* tables, CONF_*) are immutable once the pool starts. Output must be byte-identical to
`mapextractor` (same file set, `.map` + `.tilelist`); console order differs.
