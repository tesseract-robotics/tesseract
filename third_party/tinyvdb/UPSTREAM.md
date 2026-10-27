# TinyVDB source snapshot

- Upstream: https://github.com/syoyo/tinyvdb
- Commit: `e68fdc5b2c80ea8051ad1ab57cb8adf21a3321f0` (`main`)
- Retrieved: 2026-10-27
- License: Apache-2.0; see `LICENSE` and the retained source-file notices.

This directory contains only the TinyVDB sources compiled by `CMakeLists.txt` and the headers they include, copied from upstream `src/` at the same relative paths (so `third_party/nanovdb/PNanoVDB.h` stays nested). The `.h` files are clang-formatted to this repository's style; `.c` and `.inl` files are verbatim. There are no local changes.

To update, copy the same files from upstream, add whatever new sources the link needs, and run `.run-clang-format`.
