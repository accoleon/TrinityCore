# mapextractor_fast (experiment, item 535 G7)

Copy of `src/tools/map_extractor` (separate executable `mapextractor_fast`, same CLI plus `-t <threads>`, 0 = all cores).
Only the MAP step differs: all WDTs are read serially first, then every ADT tile of every map is converted by a thread
pool (atomic task index), then the per-map `.tilelist` files are written serially. dbc/camera/gt extraction is untouched.
CASC: one shared `CASC::Storage`, but each ADT's `loadFile` (open + read) runs under one mutex; the decode, conversion
and file write run unlocked, each task touching only its own `.map` file and its own tile-exists byte. The converter's
other inputs (map_ids, Liquid* tables, CONF_*) are immutable once the pool starts. Output must be byte-identical to
`mapextractor` (same file set, `.map` + `.tilelist`); console order differs.
