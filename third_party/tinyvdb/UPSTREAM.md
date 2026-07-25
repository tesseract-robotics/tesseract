# TinyVDB source snapshot

- Upstream: https://github.com/syoyo/tinyvdb
- Commit: `c0ecb7d0ea970c413b9a276c92568646e562ec12`
- Retrieved: 2026-07-23
- License: Apache-2.0; see `LICENSE` and the retained source-file notices.

This directory contains only the TinyVDB sources needed to load, materialize, and write scalar OpenVDB and NanoVDB grids. The NanoVDB I/O support includes TinyVDB's `tinyvdb_nanovdb.h`, `tinyvdb_to_nanovdb.c`, and the upstream `PNanoVDB.h` portable accessor.

The retained NanoVDB files contain small local interoperability fixes: parse NanoVDB's standard map/name layout and preserve uniform-transform translation during VDB-to-NanoVDB conversion. Keep any further vendor changes documented here.
