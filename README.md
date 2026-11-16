# VoxBox

VoxBox builds real places as small (10 cm) voxels from open map and elevation data,
and comes with a Sandbox where you walk around them.

- `engine/` – the world builder (Rust). A library plus the `voxbox` command.
- `data/rules.json` – every building rule that is data: road types, materials, building
  styles, regional colour palettes, props. A game or mod can load its own copy.
- `sandbox/` – the Sandbox (Godot 4.5): pick a built place and walk around.
- `build_places.sh` – builds the four test places and runs every check on them.

## Rules the builder follows

1. One ground height. Roads, buildings, props and collision all ask the same function.
2. No flat world. Real terrain is in from the start; every check also runs with the hills 3x steeper.
3. Elevation data gives the broad shape of the land; roads define the detailed ground:
   smooth along their length, level across, kerbs one fixed height.
4. Plan first, write once. Every wall gets a full, validated plan of its doors and windows
   before a voxel is written. An opening that does not fit is dropped in the plan, never half made.
5. One owner per column, and reserved space: travel lanes and the space in front of doors
   can never be written into.
6. Ground beside a building is cut back so no building is buried; tall steps become retaining walls.
7. The same input always gives the same world, and the result never depends on where a built area ends.
8. Bad or missing data falls back to a default and never stops a build.

## Checks

`voxbox build ... --check` builds a place and checks the finished world: every planned window
exists as glass, every door is clear with the ground at its threshold, walls reach below the
ground, nothing floats, nothing stands in a travel lane, roads are level across and smooth
along, kerbs are one height, the ground has no unmarked steps. It then repeats the build,
builds a shifted area and compares, builds with 3x hills, with a third of the map data
damaged, and with no elevation data.

The Sandbox has its own test (`--walk-test`): it walks every road and path, stands at every
door and checks that walls stop it.

## Building a place

    cd engine && cargo build --release
    ./target/release/voxbox build <name> <lat> <lon> <size_m> <map.osm.pbf> <dem_dir> <out_dir> [--check]

`<map.osm.pbf>` is an OpenStreetMap extract (for example from download.bbbike.org or
download.openstreetmap.fr). `<dem_dir>` holds Copernicus GLO-30 tiles named like
`N55_00_E012_00.tif` (from the copernicus-dem-30m open data bucket).

Put the built places in a `places` folder next to the Sandbox program (or next to the
`sandbox` folder when running from the Godot editor).

## Known limits

- Places are built beforehand; the world does not yet load around you as you walk.
- Bridges are solid causeways, building parts are not used, kerbside parking is not built.
- Diagonal walls show vertical stripes when the sun runs almost along them.
- Built places are large (a dense 256 m square is several million triangles).

## Credits

Map data (c) OpenStreetMap contributors, Open Database License.
Terrain produced using Copernicus WorldDEM-30 (c) DLR e.V. 2010-2014 and (c) Airbus Defence and
Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved.

Code: MIT licence.
