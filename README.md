# vg_core

Core Victory Garden software and tools.

vg_core is the platform-agnostic C++ core of Victory Garden. It builds a map of a
garden from sensor input (depth, IMU, GPS) supplied by a host app such as
[vg_android](https://github.com/jlizewski/vg_android), and will later track
entities in that map (growth, sunshine, temperature, wind) and compute a sun map.
Host apps only provide an interface; all the logic lives here so it can be reused
on any platform.

## Layout

| Path | Contents |
| --- | --- |
| `include/vg_core/` | Public C++ headers (namespace `vg`) |
| `src/` | Library implementation |
| `tests/` | C++ unit tests (GoogleTest) |
| `cmake/` | CMake helper modules |
| `src/io/`, `include/vg_core/io/` | `vg_core_io`: capture session recording and playback (MCAP) |
| `apps/` | Command-line apps (`vg_replay`) |
| `tools/` | Python development tools (`vg` CLI) |
| `docs/` | Design docs, including the [capture format](docs/capture-format.md) |
| `schemas/` | Protobuf schemas for vg-specific capture messages |

## Building (C++17, CMake ≥ 3.20)

```sh
cmake --preset debug
cmake --build --preset debug
ctest --preset debug
```

Eigen 3.4 and GoogleTest are used from the system if found, otherwise fetched at
configure time. `vg_core_io` also fetches zstd and the MCAP C++ library; turn it
off with `-DVG_CORE_BUILD_IO=OFF` to build only the mapping library.

## Replaying a recording

`vg_replay` rebuilds a map from a capture session
([format](docs/capture-format.md)) and writes its height map:

```sh
build/debug/vg_replay session.mcap garden.asc            # as fast as possible
build/debug/vg_replay session.mcap garden.asc --rate 1   # at recorded speed
build/debug/vg_replay session.mcap garden.asc --voxel 0.01 --trunc 0.04 --cell 0.02
```

The same session opens in [Foxglove](https://foxglove.dev) for inspection.
In code, `vg::io::SessionReader` plus `vg::io::play()` give the same playback
with a callback per message, and `vg::io::SessionMapper` feeds it into a
`vg::TsdfVolume`.

To consume vg_core from another CMake project:

```cmake
add_subdirectory(vg_core)
target_link_libraries(my_app PRIVATE vg_core::vg_core)
```

Tests are off by default when vg_core is not the top-level project
(`-DVG_CORE_BUILD_TESTS=ON` to force them).

## Development tools (Python ≥ 3.10)

```sh
python -m venv .venv && source .venv/bin/activate
pip install -e "tools[dev]"

vg build            # configure + build (debug preset)
vg test             # build and run C++ tests
vg format [--check] # clang-format C++ sources
pytest tools        # test the tools themselves
vg schemas          # regenerate src/io/schema_descriptors.cpp after editing schemas/
                    # (needs: pip install -e "tools[schemas]")
```

## License

vg_core is licensed under the [Apache License 2.0](LICENSE).
