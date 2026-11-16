//! The world model: rules, the ground grids, the voxel store and the plans.
//! Every stage reads the ground through `World::ground_height` so there is
//! exactly one source of truth for height.

use crate::geo::{Proj, CELL, P2};
use serde::Deserialize;
use std::collections::{BTreeMap, HashMap};

pub const ROW: f64 = 0.1;
pub const CH: i32 = 32;
/// A voxel that is empty to the eye but solid to walk against (window and door recesses).
pub const M_SOLID_AIR: u8 = 255;

#[derive(Deserialize, Clone, Debug)]
pub struct GroundRules {
    pub kerb_height: f64,
    pub pavement_width: f64,
    pub min_pavement_width: f64,
    pub wall_gap: f64,
    pub verge_blend: f64,
    pub path_blend: f64,
    pub urban_distance: f64,
    pub lane_clear_height: f64,
    pub path_clear_height: f64,
    pub water_drop: f64,
}
#[derive(Deserialize, Clone, Debug)]
pub struct RoadRule {
    pub kind: String,
    pub half_width: f64,
    pub material: String,
    pub pavement: bool,
}
#[derive(Deserialize, Clone, Debug)]
pub struct AreaRule {
    pub key: String,
    pub values: Vec<String>,
    pub material: String,
    pub priority: u8,
    #[serde(default)]
    pub trees: bool,
    #[serde(default)]
    pub parking: bool,
    #[serde(default)]
    pub water: bool,
}
#[derive(Deserialize, Clone, Debug)]
pub struct WindowRule {
    pub width: f64,
    pub height: f64,
    pub sill: f64,
    pub bay: f64,
    pub corner_margin: f64,
    pub clear_out: f64,
}
#[derive(Deserialize, Clone, Debug)]
pub struct DoorRule {
    pub width: f64,
    pub height: f64,
    pub clear_depth: f64,
    pub clear_side: f64,
    pub max_step: f64,
}
#[derive(Deserialize, Clone, Debug)]
pub struct BuildingRules {
    pub level_height: f64,
    pub wall_thickness: f64,
    pub min_area: f64,
    pub default_levels: BTreeMap<String, u32>,
    pub no_windows: Vec<String>,
    pub skip: Vec<String>,
    pub neighbour_radius: f64,
    pub window: WindowRule,
    pub door: DoorRule,
    pub roof_pitch: f64,
    pub max_roof_height: f64,
    pub flat_roof_min_levels: u32,
    pub flat_roof_min_area: f64,
    pub foundation_depth: f64,
}
#[derive(Deserialize, Clone, Debug)]
pub struct Palette {
    pub name: String,
    pub lat: [f64; 2],
    pub lon: [f64; 2],
    pub default_roof: String,
    pub walls: Vec<[u8; 3]>,
    pub roofs: Vec<[u8; 3]>,
}
#[derive(Deserialize, Clone, Debug)]
pub struct BarrierRule {
    pub height: f64,
    pub width: f64,
    pub material: String,
}
#[derive(Deserialize, Clone, Debug)]
pub struct BoxRule {
    pub min: [f64; 3],
    pub max: [f64; 3],
    pub material: String,
}
#[derive(Deserialize, Clone, Debug)]
pub struct PropRule {
    #[serde(default)]
    pub node: Option<[String; 2]>,
    pub boxes: Vec<BoxRule>,
    pub foot: [f64; 2],
    pub upright: bool,
    #[serde(default)]
    pub on_lane: bool,
    #[serde(default)]
    pub bodies: Vec<String>,
    #[serde(default)]
    pub solid_radius: Option<f64>,
}
#[derive(Deserialize, Clone, Debug)]
pub struct ParkingRule {
    pub row_spacing: f64,
    pub car_spacing: f64,
    pub occupancy: f64,
    pub edge_margin: f64,
}
#[derive(Deserialize, Clone, Debug)]
pub struct Rules {
    pub ground: GroundRules,
    pub materials: BTreeMap<String, [u8; 3]>,
    pub roads: BTreeMap<String, RoadRule>,
    pub surfaces: BTreeMap<String, String>,
    pub areas: Vec<AreaRule>,
    pub buildings: BuildingRules,
    pub palettes: Vec<Palette>,
    pub colours: BTreeMap<String, [u8; 3]>,
    pub barriers: BTreeMap<String, BarrierRule>,
    pub props: BTreeMap<String, PropRule>,
    pub forest_tree_spacing: f64,
    pub parking: ParkingRule,
}

pub const DEFAULT_RULES: &str = include_str!("../../data/rules.json");

impl Rules {
    pub fn default_rules() -> Rules {
        serde_json::from_str(DEFAULT_RULES).expect("built-in rules are valid")
    }
    pub fn from_file(path: &str) -> Result<Rules, String> {
        let s = std::fs::read_to_string(path).map_err(|e| format!("{path}: {e}"))?;
        serde_json::from_str(&s).map_err(|e| format!("{path}: {e}"))
    }
}

/// A rectangle of cells, half open: i0 <= i < i1, j0 <= j < j1.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Bounds {
    pub i0: i32,
    pub j0: i32,
    pub i1: i32,
    pub j1: i32,
}
impl Bounds {
    pub fn w(&self) -> usize {
        (self.i1 - self.i0) as usize
    }
    pub fn h(&self) -> usize {
        (self.j1 - self.j0) as usize
    }
    pub fn has(&self, i: i32, j: i32) -> bool {
        i >= self.i0 && i < self.i1 && j >= self.j0 && j < self.j1
    }
    pub fn grow(&self, n: i32) -> Bounds {
        Bounds { i0: self.i0 - n, j0: self.j0 - n, i1: self.i1 + n, j1: self.j1 + n }
    }
    pub fn tuple(&self) -> (i32, i32, i32, i32) {
        (self.i0, self.j0, self.i1, self.j1)
    }
    pub fn min(&self) -> P2 {
        P2::new(self.i0 as f64 * CELL, self.j0 as f64 * CELL)
    }
    pub fn max(&self) -> P2 {
        P2::new(self.i1 as f64 * CELL, self.j1 as f64 * CELL)
    }
}

#[derive(Clone)]
pub struct Grid<T: Clone> {
    pub b: Bounds,
    pub data: Vec<T>,
}
impl<T: Clone> Grid<T> {
    pub fn new(b: Bounds, v: T) -> Self {
        Grid { b, data: vec![v; b.w() * b.h()] }
    }
    #[inline]
    pub fn idx(&self, i: i32, j: i32) -> usize {
        (j - self.b.j0) as usize * self.b.w() + (i - self.b.i0) as usize
    }
    #[inline]
    pub fn get(&self, i: i32, j: i32) -> Option<&T> {
        if self.b.has(i, j) {
            Some(&self.data[self.idx(i, j)])
        } else {
            None
        }
    }
    #[inline]
    pub fn at(&self, i: i32, j: i32) -> &T {
        &self.data[self.idx(i, j)]
    }
    #[inline]
    pub fn set(&mut self, i: i32, j: i32, v: T) {
        if self.b.has(i, j) {
            let k = self.idx(i, j);
            self.data[k] = v;
        }
    }
}

pub fn cell_of(p: P2) -> (i32, i32) {
    ((p.x / CELL).floor() as i32, (p.z / CELL).floor() as i32)
}
pub fn cell_centre(i: i32, j: i32) -> P2 {
    P2::new((i as f64 + 0.5) * CELL, (j as f64 + 0.5) * CELL)
}

/// Every cell whose centre lies in the rectangle c + dir*u + out*v, u0 <= u <= u1, v0 <= v <= v1.
/// Exact for any rotation (stepping along a rotated rectangle would skip cells).
pub fn rect_cells(c: P2, dir: P2, out: P2, u: (f64, f64), v: (f64, f64)) -> Vec<(i32, i32)> {
    let corners = [(u.0, v.0), (u.0, v.1), (u.1, v.0), (u.1, v.1)].map(|(a, b)| c.add(dir.mul(a)).add(out.mul(b)));
    let (mut x0, mut z0, mut x1, mut z1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
    for p in corners {
        x0 = x0.min(p.x);
        z0 = z0.min(p.z);
        x1 = x1.max(p.x);
        z1 = z1.max(p.z);
    }
    let mut cells = Vec::new();
    for j in (z0 / CELL).floor() as i32..=(z1 / CELL).floor() as i32 {
        for i in (x0 / CELL).floor() as i32..=(x1 / CELL).floor() as i32 {
            let p = cell_centre(i, j).sub(c);
            let (a, b) = (p.dot(dir), p.dot(out));
            if a >= u.0 - 1e-9 && a <= u.1 + 1e-9 && b >= v.0 - 1e-9 && b <= v.1 + 1e-9 {
                cells.push((i, j));
            }
        }
    }
    cells
}

// Zones of a ground cell.
pub const Z_NONE: u8 = 0;
pub const Z_ROAD: u8 = 1; // travel lane of a road
pub const Z_PATH: u8 = 2; // travel lane of a path or pedestrian street
pub const Z_PAVEMENT: u8 = 3;
pub const Z_WATER: u8 = 4;
pub const Z_BUILDING: u8 = 5;
/// ground lowered in one step beside a building (foot of a retaining wall)
pub const Z_CUT: u8 = 6;

// Owners of a ground cell (who may put voxels in this column).
pub const O_NONE: u32 = 0;
pub const O_BARRIER: u32 = 1;
pub const O_BUILDING0: u32 = 16; // building index + O_BUILDING0

#[derive(Clone, Debug)]
pub struct Opening {
    pub is_door: bool,
    pub edge: usize,
    /// metres along the edge
    pub t0: f64,
    pub t1: f64,
    /// voxel rows, half open
    pub r0: i32,
    pub r1: i32,
    pub level: u32,
    /// centre point on the outer wall face, and outward direction
    pub centre: P2,
    pub out: P2,
}

#[derive(Clone, Debug)]
pub struct Edge {
    pub a: P2,
    pub b: P2,
    pub len: f64,
    /// unit normal pointing out of the building
    pub out: P2,
}

#[derive(Clone, Debug)]
pub struct BuildingPlan {
    pub id: i64,
    pub rings: Vec<Vec<P2>>,
    pub edges: Vec<Edge>,
    pub cells: Bounds,
    pub levels: u32,
    pub level_rows: i32,
    pub floor_row: i32,
    pub base_row: i32,
    pub eave_row: i32,
    pub top_row: i32,
    pub roof: String,
    pub ridge_dir: P2,
    pub ridge_mid: f64,
    pub ridge_half: f64,
    pub roof_rows: i32,
    pub wall_mat: u8,
    pub roof_mat: u8,
    pub openings: Vec<Opening>,
    pub has_door: bool,
    pub kind: String,
    pub area: f64,
    pub centroid: P2,
    /// true when this place builds the building (its centre lies in the core area)
    pub owned: bool,
    /// ground cells kept clear in front of the door
    pub door_zone: Vec<(i32, i32)>,
    /// top row of a passage carved for a travel lane, 0 = none
    pub pass_top: i32,
    pub windows: bool,
}

#[derive(Clone, Debug)]
pub struct PropInstance {
    pub model: String,
    pub id: i64,
    pub pos: P2,
    pub y: f64,
    /// rotation around the up axis, radians
    pub yaw: f64,
    /// ground slope under the prop (dy/dx, dy/dz); upright props ignore it
    pub slope: (f64, f64),
    pub body: u8,
    pub scale: f64,
    /// true for real barriers that stand on a travel lane (bollards, gates)
    pub on_lane: bool,
}

#[derive(Clone, Debug)]
pub struct TravelLine {
    pub way_id: i64,
    pub kind: String,
    pub class: String,
    /// x, y (ground height), z
    pub pts: Vec<[f64; 3]>,
    pub half_width: Vec<f64>,
}

#[derive(Clone, Debug)]
pub struct MarkedBarrier {
    pub id: i64,
    pub kind: String,
    pub pos: P2,
}

#[derive(Clone, Debug, Default)]
pub struct Stats {
    pub rejected_writes: u64,
    pub skipped: BTreeMap<String, u64>,
    pub dropped_openings: BTreeMap<String, u64>,
}
impl Stats {
    pub fn skip(&mut self, why: &str) {
        *self.skipped.entry(why.to_string()).or_default() += 1;
    }
    pub fn drop_opening(&mut self, why: &str) {
        *self.dropped_openings.entry(why.to_string()).or_default() += 1;
    }
}

pub struct World {
    pub name: String,
    pub proj: Proj,
    /// the whole area the grids cover (core plus a margin)
    pub bounds: Bounds,
    /// the area this place is responsible for
    pub core: Bounds,
    pub rules: Rules,
    pub base_height: f64,
    /// 1 = real heights; above 1 the hills are exaggerated (stress test)
    pub exaggeration: f64,
    /// base terrain from elevation data
    pub terrain: Grid<f32>,
    /// THE ground height of every cell (metres above sea level)
    pub ground: Grid<f32>,
    pub mat: Grid<u8>,
    pub zone: Grid<u8>,
    pub owner: Grid<u32>,
    /// top row (exclusive) of the reserved clear space above a cell, 0 = none.
    pub reserved_top: Grid<i32>,
    /// who may write inside the reserved space of a cell (O_NONE = nobody)
    pub reserved_for: Grid<u32>,
    /// pitched roofs: height of the roof surface above each cell (NaN = none), 10 cm columns at true height
    pub roof_h: Grid<f32>,
    pub roof_mat: Grid<u8>,
    /// direction a wall faces, for every wall column: 0 = none, 1..=126 = compass angle in steps
    pub wall_dir: Grid<u8>,
    pub palette: Vec<[u8; 3]>,
    pub mat_ids: HashMap<String, u8>,
    pub chunks: HashMap<(i32, i32, i32), Box<[u8]>>,
    pub buildings: Vec<BuildingPlan>,
    pub props: Vec<PropInstance>,
    pub travel: Vec<TravelLine>,
    pub barriers: Vec<MarkedBarrier>,
    pub stats: Stats,
}

impl World {
    pub fn mat(&self, name: &str) -> u8 {
        *self.mat_ids.get(name).unwrap_or(&1)
    }
    pub fn colour_id(&mut self, c: [u8; 3]) -> u8 {
        if let Some(k) = self.palette.iter().position(|p| *p == c) {
            return k as u8;
        }
        if self.palette.len() >= 254 {
            // palette full: nearest existing colour
            let mut best = (1usize, i32::MAX);
            for (k, p) in self.palette.iter().enumerate().skip(1) {
                let d: i32 = (0..3).map(|n| (p[n] as i32 - c[n] as i32).pow(2)).sum();
                if d < best.1 {
                    best = (k, d);
                }
            }
            return best.0 as u8;
        }
        self.palette.push(c);
        (self.palette.len() - 1) as u8
    }

    /// The one ground height function. Bilinear between cell centres, so it is
    /// continuous; outside the built area it returns the nearest edge value.
    pub fn ground_height(&self, p: P2) -> f64 {
        let fx = p.x / CELL - 0.5;
        let fz = p.z / CELL - 0.5;
        let (i, j) = (fx.floor() as i32, fz.floor() as i32);
        let (tx, tz) = (fx - i as f64, fz - j as f64);
        let g = |i: i32, j: i32| {
            let i = i.clamp(self.bounds.i0, self.bounds.i1 - 1);
            let j = j.clamp(self.bounds.j0, self.bounds.j1 - 1);
            *self.ground.at(i, j) as f64
        };
        let a = g(i, j) * (1.0 - tx) + g(i + 1, j) * tx;
        let b = g(i, j + 1) * (1.0 - tx) + g(i + 1, j + 1) * tx;
        a * (1.0 - tz) + b * tz
    }
    pub fn ground_cell(&self, i: i32, j: i32) -> f64 {
        let i = i.clamp(self.bounds.i0, self.bounds.i1 - 1);
        let j = j.clamp(self.bounds.j0, self.bounds.j1 - 1);
        *self.ground.at(i, j) as f64
    }

    #[inline]
    pub fn voxel(&self, i: i32, r: i32, j: i32) -> u8 {
        match self.chunks.get(&(i.div_euclid(CH), r.div_euclid(CH), j.div_euclid(CH))) {
            Some(c) => c[((r.rem_euclid(CH) * CH + j.rem_euclid(CH)) * CH + i.rem_euclid(CH)) as usize],
            None => 0,
        }
    }

    /// True when (i, r, j) lies inside the body of a building (behind its walls and under its roof).
    /// Nothing is drawn there, but it hides the faces that could only be seen from inside.
    pub fn inside_building(&self, i: i32, r: i32, j: i32) -> bool {
        let Some(o) = self.owner.get(i, j) else { return false };
        if *o < O_BUILDING0 {
            return false;
        }
        let bp = &self.buildings[(*o - O_BUILDING0) as usize];
        if !bp.owned {
            return false;
        }
        let z = *self.zone.at(i, j);
        let base = if z == Z_ROAD || z == Z_PATH { bp.pass_top - 2 } else { bp.base_row };
        let rh = *self.roof_h.at(i, j);
        let top = if rh.is_nan() { bp.eave_row + 2 } else { (rh as f64 / ROW).floor() as i32 };
        r >= base && r < top
    }
    /// Top row (exclusive) of the walkable-against body of a building column.
    pub fn collision_top(&self, i: i32, j: i32) -> Option<i32> {
        let o = self.owner.get(i, j)?;
        if *o < O_BUILDING0 {
            return None;
        }
        let bp = &self.buildings[(*o - O_BUILDING0) as usize];
        Some(if bp.roof == "flat" { bp.eave_row + 5 } else { bp.eave_row })
    }

    /// The only way to write a voxel. A write is refused when the column
    /// belongs to someone else or the voxel lies in reserved clear space.
    pub fn put(&mut self, i: i32, r: i32, j: i32, m: u8, who: u32) -> bool {
        if !self.bounds.has(i, j) {
            return false;
        }
        if *self.owner.at(i, j) != who {
            self.stats.rejected_writes += 1;
            return false;
        }
        let top = *self.reserved_top.at(i, j);
        if top != 0 && r < top && *self.reserved_for.at(i, j) != who {
            let floor = (*self.ground.at(i, j) as f64 / ROW).floor() as i32;
            if r >= floor {
                self.stats.rejected_writes += 1;
                return false;
            }
        }
        let c = self
            .chunks
            .entry((i.div_euclid(CH), r.div_euclid(CH), j.div_euclid(CH)))
            .or_insert_with(|| vec![0u8; (CH * CH * CH) as usize].into_boxed_slice());
        c[((r.rem_euclid(CH) * CH + j.rem_euclid(CH)) * CH + i.rem_euclid(CH)) as usize] = m;
        true
    }

    /// Reserve clear space above a cell up to `top_row`. Higher reservations win.
    pub fn reserve(&mut self, i: i32, j: i32, top_row: i32, who: u32) {
        if self.bounds.has(i, j) && top_row > *self.reserved_top.at(i, j) {
            self.reserved_top.set(i, j, top_row);
            self.reserved_for.set(i, j, who);
        }
    }

    /// A hash of everything that defines the built world. Same input must give the same hash.
    pub fn fingerprint(&self) -> u64 {
        let mut h: u64 = 0xcbf29ce484222325;
        let mut eat = |b: &[u8]| {
            for x in b {
                h ^= *x as u64;
                h = h.wrapping_mul(0x100000001b3);
            }
        };
        for v in &self.ground.data {
            eat(&v.to_le_bytes());
        }
        eat(&self.mat.data);
        eat(&self.zone.data);
        for v in &self.roof_h.data {
            eat(&v.to_bits().to_le_bytes());
        }
        let mut keys: Vec<_> = self.chunks.keys().copied().collect();
        keys.sort();
        for k in keys {
            eat(&k.0.to_le_bytes());
            eat(&k.1.to_le_bytes());
            eat(&k.2.to_le_bytes());
            eat(&self.chunks[&k]);
        }
        for p in &self.props {
            eat(p.model.as_bytes());
            eat(&p.pos.x.to_le_bytes());
            eat(&p.pos.z.to_le_bytes());
            eat(&p.y.to_le_bytes());
            eat(&p.yaw.to_le_bytes());
        }
        h
    }
}
