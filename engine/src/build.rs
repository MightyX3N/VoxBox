//! The front door of the engine: build a place from a map file and an elevation file.

use crate::dem::Dem;
use crate::geo::{edt, Proj, CELL, P2};
use crate::osm::MapData;
use crate::world::*;
use std::collections::HashMap;

pub struct PlaceSpec {
    pub name: String,
    pub lat: f64,
    pub lon: f64,
    /// core area in cells relative to the origin (lat, lon)
    pub core: Bounds,
    /// 1 = real heights, 3 = exaggerated-hills stress test
    pub exaggeration: f64,
}

impl PlaceSpec {
    pub fn centred(name: &str, lat: f64, lon: f64, size_m: f64) -> PlaceSpec {
        let h = ((size_m / CELL / 2.0) as i32 / 32) * 32;
        PlaceSpec { name: name.to_string(), lat, lon, core: Bounds { i0: -h, j0: -h, i1: h, j1: h }, exaggeration: 1.0 }
    }
    pub fn proj(&self) -> Proj {
        Proj::new(self.lat, self.lon)
    }
}

pub const MAP_MARGIN: f64 = 260.0;
pub const BUILD_MARGIN_CELLS: i32 = 608;

pub fn load_map(path: &str, spec: &PlaceSpec) -> Result<MapData, String> {
    crate::osm::load(path, &spec.proj(), spec.core.min(), spec.core.max(), MAP_MARGIN)
}

/// Builds the whole place. Same inputs always give the same world.
pub fn build(spec: &PlaceSpec, rules: &Rules, map: &MapData, dem: &mut Dem) -> World {
    let proj = spec.proj();
    dem.exaggeration = 1.0;
    dem.base = 0.0;
    let centre_h = dem.height(&proj, P2::new(0.0, 0.0));
    dem.base = centre_h;
    dem.exaggeration = spec.exaggeration;
    let bounds = spec.core.grow(BUILD_MARGIN_CELLS);
    let mut palette: Vec<[u8; 3]> = vec![[0, 0, 0]];
    let mut mat_ids: HashMap<String, u8> = HashMap::new();
    for (name, c) in &rules.materials {
        palette.push(*c);
        mat_ids.insert(name.clone(), (palette.len() - 1) as u8);
    }
    let mut w = World {
        name: spec.name.clone(),
        proj,
        bounds,
        core: spec.core,
        rules: rules.clone(),
        base_height: 0.0,
        exaggeration: spec.exaggeration,
        roof_h: Grid::new(bounds, f32::NAN),
        roof_mat: Grid::new(bounds, 0),
        wall_dir: Grid::new(bounds, 0),
        terrain: Grid::new(bounds, 0.0),
        ground: Grid::new(bounds, 0.0),
        mat: Grid::new(bounds, 1),
        zone: Grid::new(bounds, Z_NONE),
        owner: Grid::new(bounds, O_NONE),
        reserved_top: Grid::new(bounds, 0),
        reserved_for: Grid::new(bounds, O_NONE),
        palette,
        mat_ids,
        chunks: HashMap::new(),
        buildings: Vec::new(),
        props: Vec::new(),
        travel: Vec::new(),
        barriers: Vec::new(),
        stats: Stats::default(),
    };
    let gout = crate::ground::build_ground(&mut w, map, dem);
    // lowest ground in the core, rounded down to whole metres: keeps game coordinates small
    let mut lo = f64::MAX;
    for j in w.core.j0..w.core.j1 {
        for i in w.core.i0..w.core.i1 {
            lo = lo.min(*w.ground.at(i, j) as f64);
        }
    }
    w.base_height = lo.floor() - 4.0;

    // distance from every cell to the nearest travel lane (metres, capped)
    let lane_mask: Vec<bool> = w.zone.data.iter().map(|z| *z == Z_ROAD || *z == Z_PATH).collect();
    let d = edt(&lane_mask, bounds.w(), bounds.h());
    let lane_dist = Grid { b: bounds, data: d.into_iter().map(|x| (x as f64 * CELL).min(99.0) as f32).collect() };

    let probe = |w: &World, tag: &str| {
        if let Ok(d) = std::env::var("VOXPROBE") {
            let v: Vec<i32> = d.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            if v.len() == 2 {
                let row: Vec<String> = (-8..=8).map(|di| format!("{:.2}", *w.ground.at(v[0] + di, v[1]) as f64 - *w.terrain.at(v[0] + di, v[1]) as f64)).collect();
                eprintln!("probe {tag}: ground minus terrain: {}", row.join(" "));
            }
        }
    };
    probe(&w, "after roads and paths");
    crate::buildings::plan_buildings(&mut w, map, &gout, &lane_dist);
    probe(&w, "after building plans");
    crate::buildings::write_buildings(&mut w);
    crate::props::build_barriers(&mut w, map, &gout.clearance);
    crate::props::build_props(&mut w, map, &gout, &lane_dist);
    w
}
