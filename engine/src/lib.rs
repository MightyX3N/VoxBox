//! VoxBox: real-world places as small voxels.
//!
//! Games use: `build::build` to make a place, `World::ground_height` for the
//! ground, `World::travel` for the clear line along every road and path,
//! `World::barriers` for real barriers on them, `export::export` to write a
//! place to disk and `checks::run` to verify a built world.
pub mod build;
pub mod buildings;
pub mod checks;
pub mod dem;
pub mod export;
pub mod geo;
pub mod ground;
pub mod mesh;
pub mod osm;
pub mod props;
pub mod world;
