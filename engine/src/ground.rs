//! Ground stage. Elevation data only gives the broad shape of the land;
//! roads and paths define the detailed ground: smooth along their length,
//! level across their width, pavements a fixed kerb height above the road.
//! Every value of a cell depends only on features near that cell, never on
//! where the built area happens to end.

use crate::dem::Dem;
use crate::geo::{edt, fill_rings, seg_project, smoothstep, CELL, P2};
use crate::osm::{MapData, Tags};
use crate::world::*;
use std::collections::HashMap;

/// Rounds of height smoothing along the road network (each reaches about one metre further).
pub const SMOOTH_ROUNDS: usize = 150;
/// Largest extra width of a bank between a road and the land, metres.
pub const MAX_BANK: f64 = 6.0;
/// How much further than the nearest piece another piece may be and still share a cell, metres.
pub const SHARE: f64 = 0.3;

pub fn is_building(tags: &Tags, rules: &Rules) -> bool {
    match tags.get("building") {
        Some(v) => !rules.buildings.skip.iter().any(|s| s == v) && !tags.is("location", "underground"),
        None => false,
    }
}

#[derive(Clone, Debug)]
pub struct Piece {
    pub a: P2,
    pub b: P2,
    pub ha: f64,
    pub hb: f64,
    pub la: f64,
    pub lb: f64,
    pub pa: f64,
    pub pb: f64,
    pub ka: f64,
    pub kb: f64,
    pub road: bool,
    pub mat: u8,
    pub way: i64,
    /// pieces with the same group form one stretch without sharp bends
    pub group: u32,
}

pub struct GroundOut {
    /// distance in metres from each cell to the nearest building footprint (capped), on the extended area
    pub clearance: Grid<f32>,
    pub pieces: Vec<Piece>,
}

fn road_tags_ok(t: &Tags) -> bool {
    if let Some(v) = t.get("tunnel") {
        if v != "no" && v != "building_passage" {
            return false;
        }
    }
    if t.is("indoor", "yes") || t.is("area", "yes") {
        return false;
    }
    if let Some(l) = t.number("layer") {
        if l < 0.0 && !t.has("bridge") {
            return false;
        }
    }
    true
}

pub fn build_ground(w: &mut World, map: &MapData, dem: &Dem) -> GroundOut {
    let g = w.rules.ground.clone();
    let b = w.bounds;
    let ext = b.grow(400);

    // 1. Building footprints and the distance to them.
    let mut bmask = vec![false; ext.w() * ext.h()];
    for a in &map.areas {
        if is_building(&a.tags, &w.rules) {
            fill_rings(&a.rings, Some(ext.tuple()), |i, j| {
                bmask[(j - ext.j0) as usize * ext.w() + (i - ext.i0) as usize] = true;
            });
        }
    }
    let d = edt(&bmask, ext.w(), ext.h());
    drop(bmask);
    let clearance = Grid { b: ext, data: d.into_iter().map(|x| (x as f64 * CELL).min(99.0) as f32).collect() };
    let clear_at = |p: P2| -> f64 {
        let (i, j) = cell_of(p);
        clearance.get(i, j).map(|v| *v as f64).unwrap_or(99.0)
    };

    // 2. Base terrain.
    for j in b.j0..b.j1 {
        for i in b.i0..b.i1 {
            let h = dem.height(&w.proj, cell_centre(i, j)) as f32;
            w.terrain.set(i, j, h);
        }
    }

    // 3. Roads and paths: geometry and layout first (no heights yet).
    let (bmin, bmax) = (b.min(), b.max());
    struct Way {
        id: i64,
        index: u32,
        class: String,
        kind: String,
        road: bool,
        pts: Vec<P2>,
        vert: Vec<usize>,
        bridge: bool,
        lane: Vec<f64>,
        pave: Vec<f64>,
        kerb: Vec<f64>,
        mat: u8,
    }
    let mut ways: Vec<Way> = Vec::new();
    let mut vert_p: Vec<P2> = Vec::new();
    let mut vert_road: Vec<bool> = Vec::new();
    let mut vert_of_node: HashMap<i64, usize> = HashMap::new();
    for l in &map.lines {
        let Some(class) = l.tags.get("highway") else { continue };
        let Some(rule) = w.rules.roads.get(class).cloned() else {
            w.stats.skip("road type without a rule");
            continue;
        };
        let broken = l.pts.iter().any(|p| !p.x.is_finite() || !p.z.is_finite()) || l.pts.windows(2).any(|s| s[0].dist(s[1]) > 2000.0);
        if broken || !road_tags_ok(&l.tags) || l.pts.len() < 2 || l.pts.len() != l.node_ids.len() {
            w.stats.skip("road underground or unusable");
            continue;
        }
        let far = SMOOTH_ROUNDS as f64 + 10.0; // heights can travel one metre per round along the network
        if !l.pts.iter().any(|p| p.x > bmin.x - far && p.x < bmax.x + far && p.z > bmin.z - far && p.z < bmax.z + far) {
            continue;
        }
        let road = rule.kind == "road";
        let (mut pts, mut vert): (Vec<P2>, Vec<usize>) = (Vec::new(), Vec::new());
        let mut add = |p: P2, node: Option<i64>, pts: &mut Vec<P2>, vert: &mut Vec<usize>| {
            let v = match node.and_then(|n| vert_of_node.get(&n).copied()).filter(|v| vert_p[*v].dist(p) < 0.5) {
                Some(v) => v,
                None => {
                    vert_p.push(p);
                    vert_road.push(false);
                    if let Some(n) = node {
                        vert_of_node.insert(n, vert_p.len() - 1);
                    }
                    vert_p.len() - 1
                }
            };
            if road {
                vert_road[v] = true;
            }
            pts.push(p);
            vert.push(v);
        };
        for k in 0..l.pts.len() - 1 {
            let (a, c) = (l.pts[k], l.pts[k + 1]);
            let n = (a.dist(c) / 1.0).ceil().max(1.0) as usize;
            for s in 0..n {
                add(a.lerp(c, s as f64 / n as f64), if s == 0 { Some(l.node_ids[k]) } else { None }, &mut pts, &mut vert);
            }
        }
        add(*l.pts.last().unwrap(), Some(*l.node_ids.last().unwrap()), &mut pts, &mut vert);
        let n = pts.len();
        let hw0 = if let Some(wd) = l.tags.metres("width") {
            (wd / 2.0).clamp(0.4, 12.0)
        } else if let (true, Some(n)) = (road, l.tags.number("lanes")) {
            (n.clamp(1.0, 8.0) * 1.6).max(1.75)
        } else {
            rule.half_width
        };
        let sidewalk = l.tags.get("sidewalk").or(l.tags.get("sidewalk:both")).unwrap_or("");
        let wants_pave = road && sidewalk != "no" && sidewalk != "none" && (rule.pavement || matches!(sidewalk, "both" | "left" | "right" | "yes"));
        let surf = l.tags.get("surface").and_then(|s| w.rules.surfaces.get(s)).cloned().unwrap_or(rule.material.clone());
        let min_hw = if road { 0.75 } else { 0.45 };
        let (mut lane, mut pave): (Vec<f64>, Vec<f64>) = (Vec::with_capacity(n), Vec::with_capacity(n));
        for p in &pts {
            let c = clear_at(*p);
            let lh = (c - g.wall_gap).clamp(min_hw, hw0.max(min_hw));
            lane.push(lh);
            pave.push(if wants_pave && c <= g.urban_distance { (c - g.wall_gap - lh).clamp(0.0, g.pavement_width) } else { 0.0 });
        }
        // the layout changes gently along the road: narrowest nearby value, then averaged
        let smooth = |v: &Vec<f64>| -> Vec<f64> {
            let r = 3i64;
            let mn: Vec<f64> = (0..n as i64)
                .map(|k| (k - r..=k + r).filter(|x| *x >= 0 && *x < n as i64).map(|x| v[x as usize]).fold(f64::MAX, f64::min))
                .collect();
            (0..n as i64)
                .map(|k| {
                    let it: Vec<f64> = (k - r..=k + r).filter(|x| *x >= 0 && *x < n as i64).map(|x| mn[x as usize]).collect();
                    it.iter().sum::<f64>() / it.len() as f64
                })
                .collect()
        };
        let lane = smooth(&lane);
        let mut pave = smooth(&pave);
        // A kerb is one fixed height. A stretch either has a pavement (wide enough, for at least
        // five metres) or it has none.
        let mut has: Vec<bool> = pave.iter().map(|p| *p >= g.min_pavement_width).collect();
        let mut k = 0;
        while k < n {
            let mut e = k;
            while e < n && has[e] == has[k] {
                e += 1;
            }
            if has[k] && e - k < 5 {
                for x in k..e {
                    has[x] = false;
                }
            }
            k = e;
        }
        let mut kerb = vec![0.0; n];
        for k in 0..n {
            if has[k] {
                kerb[k] = g.kerb_height;
            } else {
                pave[k] = 0.0;
            }
        }
        let index = ways.len() as u32;
        ways.push(Way { id: l.id, index, class: class.to_string(), kind: rule.kind.clone(), road, pts, vert, bridge: l.tags.get("bridge").map(|v| v != "no").unwrap_or(false), lane, pave, kerb, mat: w.mat(&surf) });
    }

    // Heights along the network. Every round each point moves halfway to the mean of its
    // neighbours, so roads are smooth along their length and agree at every junction.
    let relax = |h: &mut Vec<f64>, ways: &Vec<Way>, roads: bool, fixed: &Vec<bool>| {
        let mut nbr: Vec<Vec<usize>> = vec![Vec::new(); h.len()];
        for wy in ways.iter().filter(|x| x.road == roads) {
            for k in 0..wy.vert.len() - 1 {
                let (a, c) = (wy.vert[k], wy.vert[k + 1]);
                if a != c {
                    nbr[a].push(c);
                    nbr[c].push(a);
                }
            }
        }
        // At a junction the two arms that run most nearly straight through set the height;
        // side roads join it. Otherwise every side road would put a kink in the main road.
        for v in 0..h.len() {
            if nbr[v].len() > 2 {
                nbr[v].sort();
                nbr[v].dedup();
                let (mut best, mut pair) = (f64::MAX, (nbr[v][0], nbr[v][1]));
                for a in 0..nbr[v].len() {
                    for c in a + 1..nbr[v].len() {
                        let da = vert_p[nbr[v][a]].sub(vert_p[v]).norm();
                        let dc = vert_p[nbr[v][c]].sub(vert_p[v]).norm();
                        if da.dot(dc) < best - 1e-9 {
                            best = da.dot(dc);
                            pair = (nbr[v][a], nbr[v][c]);
                        }
                    }
                }
                nbr[v] = vec![pair.0, pair.1];
            }
        }
        for _ in 0..SMOOTH_ROUNDS {
            let old = h.clone();
            for v in 0..h.len() {
                if !nbr[v].is_empty() && !fixed[v] {
                    let m: f64 = nbr[v].iter().map(|n| old[*n]).sum::<f64>() / nbr[v].len() as f64;
                    h[v] = 0.5 * old[v] + 0.5 * m;
                }
            }
        }
    };
    let pieces_of = |wy: &Way, vert_h: &Vec<f64>| -> Vec<Piece> {
        let n = wy.pts.len();
        let mut h: Vec<f64> = wy.vert.iter().map(|v| vert_h[*v]).collect();
        if wy.bridge {
            let (h0, h1) = (h[0], h[n - 1]);
            for k in 0..n {
                h[k] = h0 + (h1 - h0) * k as f64 / (n - 1).max(1) as f64;
            }
        }
        // a new stretch starts whenever the way has turned more than 60 degrees
        let mut out: Vec<Piece> = Vec::with_capacity(n - 1);
        let (mut sub, mut start_dir) = (0u32, wy.pts[1].sub(wy.pts[0]).norm());
        for k in 0..n - 1 {
            let dir = wy.pts[k + 1].sub(wy.pts[k]).norm();
            if dir.dot(start_dir) < 0.5 {
                sub += 1;
                start_dir = dir;
            }
            out.push(Piece { a: wy.pts[k], b: wy.pts[k + 1], ha: h[k], hb: h[k + 1], la: wy.lane[k], lb: wy.lane[k + 1], pa: wy.pave[k], pb: wy.pave[k + 1], ka: wy.kerb[k], kb: wy.kerb[k + 1], road: wy.road, mat: wy.mat, way: wy.id, group: wy.index * 4096 + sub });
        }
        out
    };
    let mut vert_h: Vec<f64> = vert_p.iter().map(|p| dem.height(&w.proj, *p)).collect();
    relax(&mut vert_h, &ways, true, &vec![false; vert_p.len()]);
    let road_pieces: Vec<Piece> = ways.iter().filter(|x| x.road).flat_map(|x| pieces_of(x, &vert_h)).collect();
    // path pieces without heights yet: enough to mark their lanes
    let mut path_pieces: Vec<Piece> = ways.iter().filter(|x| !x.road).flat_map(|x| pieces_of(x, &vert_h)).collect();

    // 4. Which cells are travel lanes and pavements.
    let big = 1e9f32;
    let mut lane = Grid::new(b, 0u8);
    let mut lane_best = Grid::new(b, big);
    let mut lane_mat = Grid::new(b, 0u8);
    let mut q = Grid::new(b, big); // distance beyond the outer edge of the nearest road corridor
    let mut e = Grid::new(b, big); // distance beyond the nearest road lane edge
    let mut paved = Grid::new(b, false);
    let mut edge_best = Grid::new(b, big);
    let mut edge_mat = Grid::new(b, 0u8);
    let cells_near = |p: &Piece, r: f64| -> (i32, i32, i32, i32) {
        let i0 = (((p.a.x.min(p.b.x) - r) / CELL).floor() as i32).max(b.i0);
        let i1 = (((p.a.x.max(p.b.x) + r) / CELL).ceil() as i32).min(b.i1);
        let j0 = (((p.a.z.min(p.b.z) - r) / CELL).floor() as i32).max(b.j0);
        let j1 = (((p.a.z.max(p.b.z) + r) / CELL).ceil() as i32).min(b.j1);
        (i0, j0, i1, j1)
    };
    for p in road_pieces.iter().chain(path_pieces.iter()) {
        let r = p.la.max(p.lb) + p.pa.max(p.pb) + 0.7;
        let (i0, j0, i1, j1) = cells_near(p, r);
        for j in j0..j1 {
            for i in i0..i1 {
                let (t, d, _) = seg_project(cell_centre(i, j), p.a, p.b);
                let lh = p.la + (p.lb - p.la) * t;
                let pw = p.pa + (p.pb - p.pa) * t;
                let k = lane.idx(i, j);
                if d < lh {
                    let nd = (d / lh) as f32;
                    let cur = lane.data[k];
                    let rank = if p.road { 1 } else { 2 };
                    if cur == 0 || rank < cur || (rank == cur && nd < lane_best.data[k]) {
                        lane.data[k] = rank;
                        lane_best.data[k] = nd;
                        lane_mat.data[k] = p.mat;
                    }
                }
                // the surface runs on past the lane edge up to the walls beside it
                if d < lh + g.wall_gap + 0.25 && (d - lh) as f32 <= edge_best.data[k] {
                    edge_best.data[k] = (d - lh) as f32;
                    edge_mat.data[k] = p.mat;
                }
                if p.road {
                    // a pavement runs right up to the wall beside it (no leftover strip in between)
                    let to_wall = if pw > 0.0 && (*clearance.at(i, j) as f64) < g.wall_gap + 0.35 { g.wall_gap + 0.25 } else { 0.0 };
                    e.data[k] = e.data[k].min((d - lh) as f32);
                    q.data[k] = q.data[k].min((d - lh - pw - to_wall) as f32);
                    if pw > 0.0 && d < lh + pw + to_wall {
                        paved.data[k] = true;
                    }
                }
            }
        }
    }
    // 5. Ground height in two layers. First the roads on the land, then the paths on top of that.
    // A piece's surface is its centre height (level across); several pieces are mixed by distance,
    // and a bank joins the surface to the layer below. The reach of a piece does not depend on heights.
    let layer = |pieces: &Vec<Piece>, below: &Vec<f32>, roads: bool, w: &mut World| -> Vec<f32> {
        let blend0 = if roads { g.verge_blend } else { g.path_blend };
        let reach = blend0 + MAX_BANK;
        // distance from each cell to the nearest piece of this layer
        let mut dmin = vec![big; below.len()];
        for p in pieces {
            let (i0, j0, i1, j1) = cells_near(p, p.la.max(p.lb) + p.pa.max(p.pb) + reach);
            for j in j0..j1 {
                for i in i0..i1 {
                    let (_, d, _) = seg_project(cell_centre(i, j), p.a, p.b);
                    let k = w.ground.idx(i, j);
                    dmin[k] = dmin[k].min(d as f32);
                }
            }
        }
        // Pieces about as near as the nearest one share the cell; anything further away has no say.
        // On a straight road that is the piece under the cell (level across); inside a bend and at
        // junctions the neighbours are mixed in smoothly, so there is never a jump.
        let mut sw = vec![0f64; below.len()];
        let mut sws = vec![0f64; below.len()];
        let mut sws2 = vec![0f64; below.len()];
        let mut amin = vec![big; below.len()];
        for p in pieces {
            let (i0, j0, i1, j1) = cells_near(p, p.la.max(p.lb) + p.pa.max(p.pb) + reach);
            for j in j0..j1 {
                for i in i0..i1 {
                    let (t, d, _) = seg_project(cell_centre(i, j), p.a, p.b);
                    let hw = p.la + (p.lb - p.la) * t + p.pa + (p.pb - p.pa) * t;
                    let k = w.ground.idx(i, j);
                    let over = (d - dmin[k] as f64) / SHARE;
                    if d >= hw + reach || over > 12.0 {
                        continue;
                    }
                    let mut s = p.ha + (p.hb - p.ha) * t;
                    if roads && lane.data[k] != 1 {
                        // the kerb face follows the true edge of the lane: the row of cells the edge
                        // passes through rises in proportion to how far past the edge each one lies
                        s += (p.ka + (p.kb - p.ka) * t) * (e.data[k] as f64 / CELL + 0.5).clamp(0.3, 1.0);
                    }
                    let wt = (1.0 - d / (hw + reach)).powi(2) * (-over).exp();
                    sw[k] += wt;
                    sws[k] += wt * s;
                    sws2[k] += wt * s * s;
                    // the further the surface is above or below the layer underneath, the wider the bank
                    let blend = blend0 + (s - below[k] as f64).abs().min(MAX_BANK);
                    amin[k] = amin[k].min(((d - hw) / blend) as f32);
                }
            }
        }
        (0..below.len())
            .map(|k| {
                let t = below[k] as f64;
                if sw[k] > 0.0 {
                    let r = sws[k] / sw[k];
                    // Two surfaces more than half a metre apart are being joined here (two arms of a
                    // hairpin, a road above a road): the short steep join is a retaining wall.
                    let spread = (sws2[k] / sw[k] - r * r).max(0.0).sqrt();
                    if spread > 0.25 && w.zone.data[k] == Z_NONE && lane.data[k] == 0 && !paved.data[k] {
                        w.zone.data[k] = Z_CUT;
                    }
                    let mut u = 1.0 - smoothstep(amin[k] as f64);
                    if !roads {
                        // Inside a road corridor only the road counts, so roads stay level across.
                        // A path that meets the road at nearly its height joins it over a metre; a path
                        // that passes at another height keeps its own level and a retaining step
                        // separates the two.
                        let qq = q.data[k] as f64;
                        if (r - t).abs() > 0.25 {
                            u *= if qq > 0.0 { 1.0 } else { 0.0 };
                            // the cells at the foot (or top) of that step are marked as a retaining wall
                            if qq > 0.0 && u > 0.0 && w.zone.data[k] == Z_NONE {
                                let (i, j) = (b.i0 + (k % b.w()) as i32, b.j0 + (k / b.w()) as i32);
                                if [(1, 0), (-1, 0), (0, 1), (0, -1)].iter().any(|(di, dj)| q.get(i + di, j + dj).map(|x| *x <= 0.0).unwrap_or(false)) {
                                    w.zone.data[k] = Z_CUT;
                                }
                            }
                        } else {
                            u *= smoothstep(qq);
                        }
                    }
                    (t + (r - t) * u) as f32
                } else {
                    t as f32
                }
            })
            .collect()
    };
    let below = w.terrain.data.clone();
    let g1 = layer(&road_pieces, &below, true, w);
    w.ground.data = g1;
    // paths start from the ground the roads made, then get their own smooth profile
    // Inside a road's width a path has no height of its own: its profile starts at the road's edge.
    let mut fixed = vert_road.clone();
    for v in 0..vert_p.len() {
        if !vert_road[v] {
            vert_h[v] = w.ground_height(vert_p[v]);
            let (i, j) = cell_of(vert_p[v]);
            if q.get(i, j).map(|x| *x <= 0.0).unwrap_or(false) {
                fixed[v] = true;
            }
        }
    }
    relax(&mut vert_h, &ways, false, &fixed);
    path_pieces = ways.iter().filter(|x| !x.road).flat_map(|x| pieces_of(x, &vert_h)).collect();
    let below = w.ground.data.clone();
    let g2 = layer(&path_pieces, &below, false, w);
    w.ground.data = g2;
    let mut travel: Vec<TravelLine> = ways
        .iter()
        .map(|wy| TravelLine { way_id: wy.id, kind: wy.kind.clone(), class: wy.class.clone(), pts: wy.pts.iter().map(|p| [p.x, 0.0, p.z]).collect(), half_width: wy.lane.clone() })
        .collect();
    let mut pieces = road_pieces;
    pieces.extend(path_pieces);
    drop(ways);

    // 6. Materials from map areas, by rule priority; then roads on top.
    let mut prio = Grid::new(b, 0u8);
    let area_rules = w.rules.areas.clone();
    for a in &map.areas {
        for rule in &area_rules {
            let Some(v) = a.tags.get(&rule.key) else { continue };
            if !rule.values.iter().any(|x| x == v) {
                continue;
            }
            if rule.key == "highway" && !a.tags.is("area", "yes") {
                continue;
            }
            let m = w.mat(&rule.material);
            let mut level = f64::MAX;
            if rule.water {
                for r in &a.rings {
                    for p in r {
                        level = level.min(dem.height(&w.proj, *p));
                    }
                }
                level -= g.water_drop;
            }
            let (prio_ref, mat_ref, zone_ref, ground_ref, lane_ref) = (&mut prio, &mut w.mat, &mut w.zone, &mut w.ground, &lane);
            fill_rings(&a.rings, Some(b.tuple()), |i, j| {
                let k = prio_ref.idx(i, j);
                if rule.priority >= prio_ref.data[k] {
                    prio_ref.data[k] = rule.priority;
                    mat_ref.data[k] = m;
                    if rule.water && lane_ref.data[k] == 0 {
                        zone_ref.data[k] = Z_WATER;
                        ground_ref.data[k] = level as f32;
                    } else if zone_ref.data[k] == Z_WATER {
                        zone_ref.data[k] = Z_NONE;
                    }
                }
            });
        }
    }
    let (m_grass, m_yard, m_pave) = (w.mat("grass"), w.mat("yard"), w.mat("pavement"));
    let lane_rows = (g.lane_clear_height / ROW).round() as i32;
    let path_rows = (g.path_clear_height / ROW).round() as i32;
    for j in b.j0..b.j1 {
        for i in b.i0..b.i1 {
            let k = lane.idx(i, j);
            if prio.data[k] == 0 {
                w.mat.data[k] = if (*clearance.at(i, j) as f64) <= g.urban_distance { m_yard } else { m_grass };
            }
            match lane.data[k] {
                1 | 2 => {
                    let road = lane.data[k] == 1;
                    w.zone.data[k] = if road { Z_ROAD } else { Z_PATH };
                    w.mat.data[k] = if !road && paved.data[k] { m_pave } else { lane_mat.data[k] };
                    let top = (w.ground.data[k] as f64 / ROW).floor() as i32 + if road { lane_rows } else { path_rows };
                    w.reserved_top.data[k] = top;
                    w.reserved_for.data[k] = O_NONE;
                }
                _ => {
                    if paved.data[k] && w.zone.data[k] != Z_WATER {
                        w.zone.data[k] = Z_PAVEMENT;
                        w.mat.data[k] = m_pave;
                    } else if edge_mat.data[k] != 0 && w.zone.data[k] != Z_WATER && (*clearance.at(i, j) as f64) < g.wall_gap + 0.35 {
                        w.mat.data[k] = edge_mat.data[k];
                    }
                }
            }
        }
    }

    // 7. The clear line along every road and path, with ground heights.
    for t in travel.iter_mut() {
        for p in t.pts.iter_mut() {
            p[1] = w.ground_height(P2::new(p[0], p[2]));
        }
    }
    w.travel = travel;
    GroundOut { clearance, pieces }
}
