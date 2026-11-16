//! Buildings. Plan first, write once:
//!  1. every building gets its size, floor level and door before anything is written,
//!  2. the ground in front of every door is levelled and reserved,
//!  3. each wall gets a complete, validated plan of its openings,
//!  4. only then are the voxels written, each column by its one owner.

use crate::geo::{edt, fill_rings, hash2, point_in_rings, rand01, ring_area, ring_centroid, seg_project, CELL, P2};
use crate::ground::{is_building, GroundOut};
use crate::osm::{Area, MapData, Tags};
use crate::world::*;

fn parse_colour(w: &World, v: &str) -> Option<[u8; 3]> {
    let v = v.trim().to_lowercase();
    if let Some(h) = v.strip_prefix('#') {
        if h.len() == 6 {
            let n = u32::from_str_radix(h, 16).ok()?;
            return Some([(n >> 16) as u8, (n >> 8) as u8, n as u8]);
        }
        if h.len() == 3 {
            let n = u32::from_str_radix(h, 16).ok()?;
            return Some([((n >> 8) & 15) as u8 * 17, ((n >> 4) & 15) as u8 * 17, (n & 15) as u8 * 17]);
        }
        return None;
    }
    w.rules.colours.get(&v).copied()
}

fn levels_from_tags(t: &Tags, level_height: f64) -> Option<u32> {
    if let Some(n) = t.number("building:levels") {
        if n >= 1.0 && n <= 80.0 {
            return Some(n.round() as u32);
        }
    }
    if let Some(h) = t.metres("height") {
        return Some(((h / level_height).round() as u32).clamp(1, 80));
    }
    None
}

/// Phase 1: sizes, floor levels and doors for every building. Nothing is written.
pub fn plan_buildings(w: &mut World, map: &MapData, gout: &GroundOut, lane_dist: &Grid<f32>) {
    let br = w.rules.buildings.clone();
    let b = w.bounds;
    let (core_min, core_max) = (w.core.min(), w.core.max());

    // All buildings that touch the built area, in id order. Lower id wins a shared cell.
    let mut list: Vec<&Area> = map.areas.iter().filter(|a| is_building(&a.tags, &w.rules)).collect();
    list.sort_by_key(|a| a.id);
    let tagged: Vec<(P2, u32)> = list
        .iter()
        .filter_map(|a| levels_from_tags(&a.tags, br.level_height).map(|l| (ring_centroid(&a.rings[0]), l)))
        .collect();

    let (lat, lon) = (w.proj.lat0, w.proj.lon0);
    let pal = w
        .rules
        .palettes
        .iter()
        .find(|p| lat >= p.lat[0] && lat <= p.lat[1] && lon >= p.lon[0] && lon <= p.lon[1])
        .cloned()
        .unwrap_or_else(|| w.rules.palettes.last().cloned().expect("at least one palette"));

    for a in list {
        let area: f64 = a.rings.iter().map(|r| ring_area(r).abs()).fold(0.0, f64::max);
        if area < br.min_area {
            w.stats.skip("building smaller than the minimum area");
            continue;
        }
        let outer = a.rings.iter().max_by(|x, y| ring_area(x).abs().partial_cmp(&ring_area(y).abs()).unwrap()).unwrap();
        let centroid = ring_centroid(outer);
        let (mut x0, mut z0, mut x1, mut z1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in a.rings.iter().flatten() {
            x0 = x0.min(p.x);
            z0 = z0.min(p.z);
            x1 = x1.max(p.x);
            z1 = z1.max(p.z);
        }
        let cells = Bounds { i0: (x0 / CELL).floor() as i32 - 1, j0: (z0 / CELL).floor() as i32 - 1, i1: (x1 / CELL).ceil() as i32 + 1, j1: (z1 / CELL).ceil() as i32 + 1 };
        if cells.i1 <= b.i0 || cells.i0 >= b.i1 || cells.j1 <= b.j0 || cells.j0 >= b.j1 {
            continue;
        }
        let mut owned = centroid.x >= core_min.x && centroid.x < core_max.x && centroid.z >= core_min.z && centroid.z < core_max.z;
        if owned && (cells.i0 < b.i0 || cells.j0 < b.j0 || cells.i1 > b.i1 || cells.j1 > b.j1) {
            w.stats.skip("building larger than the build margin");
            owned = false;
        }
        let kind = a.tags.get("building").unwrap_or("yes").to_string();
        let levels = levels_from_tags(&a.tags, br.level_height).unwrap_or_else(|| {
            // no height in the data: use what the tagged neighbours have, else the default for the type
            let mut near: Vec<u32> = tagged.iter().filter(|(c, _)| c.dist(centroid) <= br.neighbour_radius).map(|(_, l)| *l).collect();
            let small = br.default_levels.get(&kind).copied().unwrap_or(2);
            if kind == "yes" && near.len() >= 3 {
                near.sort();
                near[near.len() / 2]
            } else if kind == "yes" {
                small + (hash2(a.id as u64, 7) % 2) as u32
            } else {
                small
            }
        });
        let levels = if area < 30.0 { levels.min(1) } else { levels };
        let mut edges: Vec<Edge> = Vec::new();
        for r in &a.rings {
            for k in 0..r.len() {
                let (p, q) = (r[k], r[(k + 1) % r.len()]);
                let len = p.dist(q);
                if len < 1e-6 {
                    continue;
                }
                let d = q.sub(p).norm();
                let mut out = P2::new(d.z, -d.x);
                if point_in_rings(p.lerp(q, 0.5).add(out.mul(0.05)), &a.rings) {
                    out = out.mul(-1.0);
                }
                edges.push(Edge { a: p, b: q, len, out });
            }
        }
        if edges.len() < 3 {
            w.stats.skip("building outline broken");
            continue;
        }
        let shape_tag = a.tags.get("roof:shape").unwrap_or("");
        let roof = match shape_tag {
            "flat" | "skillion" => "flat",
            "gabled" | "gambrel" | "saltbox" | "round" => "gabled",
            "hipped" | "pyramidal" | "dome" | "cone" | "onion" | "half-hipped" | "mansard" => "hipped",
            _ => {
                if levels >= br.flat_roof_min_levels || area >= br.flat_roof_min_area {
                    "flat"
                } else {
                    pal.default_roof.as_str()
                }
            }
        }
        .to_string();
        let long = edges.iter().max_by(|x, y| x.len.partial_cmp(&y.len).unwrap()).unwrap();
        let ridge_dir = long.b.sub(long.a).norm();
        let across = P2::new(-ridge_dir.z, ridge_dir.x);
        let (mut amin, mut amax) = (f64::MAX, f64::MIN);
        for p in outer {
            let v = p.dot(across);
            amin = amin.min(v);
            amax = amax.max(v);
        }
        let ridge_half = ((amax - amin) / 2.0).max(0.2);
        let roof_rows = if roof == "flat" { 0 } else { ((ridge_half * br.roof_pitch).min(br.max_roof_height) / ROW).round() as i32 };
        let wall_c = a.tags.get("building:colour").and_then(|v| parse_colour(w, v)).unwrap_or_else(|| pal.walls[(hash2(a.id as u64, 1) % pal.walls.len() as u64) as usize]);
        let roof_c = a.tags.get("roof:colour").and_then(|v| parse_colour(w, v)).unwrap_or_else(|| pal.roofs[(hash2(a.id as u64, 2) % pal.roofs.len() as u64) as usize]);
        let (wall_mat, roof_mat) = (w.colour_id(wall_c), w.colour_id(roof_c));
        let windows = !br.no_windows.iter().any(|k| *k == kind) && area >= 12.0;
        let level_rows = (br.level_height / ROW).round() as i32;
        w.buildings.push(BuildingPlan {
            id: a.id, rings: a.rings.clone(), edges, cells, levels, level_rows, floor_row: 0, base_row: 0, eave_row: 0, top_row: 0, roof,
            ridge_dir, ridge_mid: (amin + amax) / 2.0, ridge_half, roof_rows, wall_mat, roof_mat, openings: Vec::new(), has_door: false,
            kind, area, centroid, owned, door_zone: Vec::new(), pass_top: 0, windows,
        });
    }

    // Column ownership: one owner per ground cell, decided before any voxel exists.
    for (n, bp) in w.buildings.iter().enumerate() {
        let code = O_BUILDING0 + n as u32;
        let (owner, zone) = (&mut w.owner, &mut w.zone);
        fill_rings(&bp.rings, Some(b.tuple()), |i, j| {
            let k = owner.idx(i, j);
            if owner.data[k] == O_NONE {
                owner.data[k] = code;
                if zone.data[k] != Z_ROAD && zone.data[k] != Z_PATH {
                    zone.data[k] = Z_BUILDING;
                }
            }
        });
    }
    let _ = gout;

    // Doors: for every owned building pick the spot on its walls closest to a road or path
    // where a door fits and the space in front is free.
    let dr = br.door.clone();
    let half = dr.width / 2.0;
    let entrances: Vec<P2> = map.points.iter().filter(|p| p.tags.get("entrance").map(|v| v != "no").unwrap_or(false)).map(|p| p.p).collect();
    struct DoorPick {
        b: usize,
        edge: usize,
        t: f64,
        h: f64,
    }
    let mut picks: Vec<DoorPick> = Vec::new();
    let mut plannable = vec![false; w.buildings.len()];
    let safe = b.grow(-100);
    for (n, bp) in w.buildings.iter().enumerate() {
        // Doors are planned for every building that lies safely inside the built area, also the ones a
        // neighbouring place will write, so the ground in front of a door is the same from both sides.
        if bp.cells.i0 < safe.i0 || bp.cells.j0 < safe.j0 || bp.cells.i1 > safe.i1 || bp.cells.j1 > safe.j1 {
            continue;
        }
        let zone_free = |c: P2, dir: P2, out: P2| -> bool {
            rect_cells(c, dir, out, (-(half + dr.clear_side), half + dr.clear_side), (0.15, dr.clear_depth)).iter().all(|(i, j)| w.owner.get(*i, *j) == Some(&O_NONE))
        };
        let mut best: Option<(f64, usize, f64)> = None;
        for (ei, e) in bp.edges.iter().enumerate() {
            if e.len < dr.width + 2.0 * br.window.corner_margin {
                continue;
            }
            let dir = e.b.sub(e.a).norm();
            let mut cand: Vec<f64> = Vec::new();
            for en in &entrances {
                let (t, d, _) = seg_project(*en, e.a, e.b);
                if d < 0.4 {
                    cand.push((t * e.len).clamp(br.window.corner_margin + half, e.len - br.window.corner_margin - half));
                }
            }
            let mapped = !cand.is_empty();
            if !mapped {
                let mut t = br.window.corner_margin + half;
                while t <= e.len - br.window.corner_margin - half + 1e-9 {
                    cand.push(t);
                    t += 0.5;
                }
            }
            for t in cand {
                let c = e.a.add(dir.mul(t));
                if !zone_free(c, dir, e.out) {
                    continue;
                }
                let (i, j) = cell_of(c.add(e.out.mul(0.6)));
                let ld = lane_dist.get(i, j).map(|v| *v as f64).unwrap_or(99.0);
                // a mapped entrance always beats a guessed one
                let score = ld + 0.02 * (t - e.len / 2.0).abs() - if mapped { 1000.0 } else { 0.0 };
                if best.map(|(s, _, _)| score < s - 1e-9).unwrap_or(true) {
                    best = Some((score, ei, t));
                }
            }
        }
        if let Some((_, edge, t)) = best {
            let e = &bp.edges[edge];
            let c = e.a.add(e.b.sub(e.a).norm().mul(t));
            // A door that opens straight onto a road or path takes its level from it;
            // otherwise from the ground at the door.
            let dir = e.b.sub(e.a).norm();
            let mut h = f64::MIN;
            for (i, j) in rect_cells(c, dir, e.out, (-(half + dr.clear_side), half + dr.clear_side), (0.15, dr.clear_depth)) {
                if matches!(w.zone.get(i, j), Some(&Z_ROAD) | Some(&Z_PATH) | Some(&Z_PAVEMENT)) {
                    h = h.max(*w.ground.at(i, j) as f64);
                }
            }
            if h == f64::MIN {
                h = w.ground_height(c.add(e.out.mul(0.3)));
            }
            picks.push(DoorPick { b: n, edge, t, h });
        }
        plannable[n] = true;
    }

    // Level the ground in front of each door (never on a travel lane) and work out the floor level.
    let mut apron_w = Grid::new(b, 0f32);
    let mut apron_h = Grid::new(b, 0f32);
    let mut apron_d = Grid::new(b, 0f32);
    let ring = 0.5;
    for pk in &picks {
        let e = w.buildings[pk.b].edges[pk.edge].clone();
        let dir = e.b.sub(e.a).norm();
        let c = e.a.add(dir.mul(pk.t));
        let h = pk.h;
        let (su, sv) = (half + dr.clear_side, dr.clear_depth);
        let r = su.max(sv) + ring + 0.2;
        let (ci, cj) = cell_of(c);
        let n = (r / CELL).ceil() as i32;
        for j in cj - n..=cj + n {
            for i in ci - n..=ci + n {
                if !b.has(i, j) {
                    continue;
                }
                let p = cell_centre(i, j).sub(c);
                let (u, v) = (p.dot(dir), p.dot(e.out));
                if v < 0.0 {
                    continue;
                }
                let du = (u.abs() - su).max(0.0);
                let dv = (v - sv).max(0.0);
                let d = (du * du + dv * dv).sqrt();
                if d >= ring {
                    continue;
                }
                let k = apron_w.idx(i, j);
                let z = w.zone.data[k];
                if z == Z_ROAD || z == Z_PATH || z == Z_BUILDING || z == Z_WATER {
                    continue;
                }
                // several doors close together: their landings are mixed, the nearest one dominating
                let wt = crate::geo::smoothstep(1.0 - d / ring) as f32;
                let mix = wt.powi(4) / (0.05 + p.len() as f32);
                apron_d.data[k] += mix;
                apron_h.data[k] += mix * h as f32;
                apron_w.data[k] = apron_w.data[k].max(wt);
            }
        }
    }
    // Site cut: the ground next to a building is never more than half a metre above its floor
    // at the wall, rising away from it. Roads, pavements and water are never touched, and the
    // cut ground climbs back up to them at a fixed slope, so there are no cliffs.
    {
        let mut f0: Vec<Option<f64>> = vec![None; w.buildings.len()];
        for pk in &picks {
            f0[pk.b] = Some((pk.h / ROW - 1e-4).ceil() * ROW);
        }
        let mut bound = Grid::new(b, f32::MAX);
        let mut cap = Grid::new(b, f32::MAX);
        let reach = 80i32;
        for n in 0..w.buildings.len() {
            if !plannable[n] {
                continue;
            }
            let code = O_BUILDING0 + n as u32;
            let c = w.buildings[n].cells.grow(reach);
            let c = Bounds { i0: c.i0.max(b.i0), j0: c.j0.max(b.j0), i1: c.i1.min(b.i1), j1: c.j1.min(b.j1) };
            let mut own = vec![false; c.w() * c.h()];
            let (mut gmax, mut any) = (f64::MIN, false);
            for j in c.j0..c.j1 {
                for i in c.i0..c.i1 {
                    if *w.owner.at(i, j) == code {
                        own[(j - c.j0) as usize * c.w() + (i - c.i0) as usize] = true;
                        gmax = gmax.max(*w.ground.at(i, j) as f64);
                        any = true;
                    }
                }
            }
            if !any {
                continue;
            }
            let floor = f0[n].unwrap_or((gmax / ROW - 1e-4).ceil() * ROW);
            let d = edt(&own, c.w(), c.h());
            for j in c.j0..c.j1 {
                for i in c.i0..c.i1 {
                    let dd = d[(j - c.j0) as usize * c.w() + (i - c.i0) as usize] as f64 * CELL;
                    if dd > 0.0 && dd < reach as f64 * CELL {
                        let k = bound.idx(i, j);
                        bound.data[k] = bound.data[k].min((floor + 0.5 + dd) as f32);
                        let eave = floor + w.buildings[n].levels as f64 * w.rules.buildings.level_height;
                        cap.data[k] = cap.data[k].min((eave - 1.0 + dd) as f32);
                    }
                }
            }
        }
        // h = cut ground; protected cells keep their height; then make sure the cut never drops
        // faster than BANK below its surroundings (two sweeps, like a distance transform).
        const BANK: f32 = 1.5;
        let (bw, bh) = (b.w() as i32, b.h() as i32);
        // door landings are fixed too: the cut climbs up to them like it does to a road
        let landing: Vec<bool> = (0..w.zone.data.len()).map(|k| apron_w.data[k] > 0.999 && apron_d.data[k] > 0.0).collect();
        let fixed: Vec<bool> = (0..w.zone.data.len()).map(|k| landing[k] || matches!(w.zone.data[k], Z_ROAD | Z_PATH | Z_PAVEMENT | Z_WATER)).collect();
        let mut h: Vec<f32> = (0..w.ground.data.len())
            .map(|k| {
                if landing[k] {
                    apron_h.data[k] / apron_d.data[k]
                } else if fixed[k] {
                    w.ground.data[k]
                } else if w.zone.data[k] == Z_BUILDING {
                    f32::MIN
                } else {
                    w.ground.data[k].min(bound.data[k])
                }
            })
            .collect();
        let (s1, s2) = (BANK * CELL as f32, BANK * CELL as f32 * std::f32::consts::SQRT_2);
        let relax = |h: &mut Vec<f32>, x: i32, y: i32, nb: &[(i32, i32, f32)]| {
            let k = (y * bw + x) as usize;
            if fixed[k] {
                return;
            }
            let mut v = h[k];
            for (dx, dy, c) in nb {
                let (nx, ny) = (x + dx, y + dy);
                if nx >= 0 && ny >= 0 && nx < bw && ny < bh {
                    v = v.max(h[(ny * bw + nx) as usize] - c);
                }
            }
            h[k] = v;
        };
        for y in 0..bh {
            for x in 0..bw {
                relax(&mut h, x, y, &[(-1, 0, s1), (0, -1, s1), (-1, -1, s2), (1, -1, s2)]);
            }
        }
        for y in (0..bh).rev() {
            for x in (0..bw).rev() {
                relax(&mut h, x, y, &[(1, 0, s1), (0, 1, s1), (1, 1, s2), (-1, 1, s2)]);
            }
        }
        for k in 0..h.len() {
            if !fixed[k] && w.zone.data[k] != Z_BUILDING {
                let mut v = w.ground.data[k].min(h[k]);
                // never higher than a building beside it: where a road stands far above a building,
                // the ground drops in one step at the road's edge (drawn as a retaining wall)
                if v > cap.data[k] {
                    v = cap.data[k];
                    w.zone.data[k] = Z_CUT;
                }
                w.ground.data[k] = v;
            }
        }
    }

    for k in 0..w.ground.data.len() {
        let wt = apron_w.data[k];
        if wt > 0.0 && apron_d.data[k] > 0.0 {
            let diff = apron_h.data[k] / apron_d.data[k] - w.ground.data[k];
            // a landing that stands well above or below the ground beside it gets a retaining edge
            if wt < 0.999 && diff.abs() > 0.3 && w.zone.data[k] == Z_NONE {
                w.zone.data[k] = Z_CUT;
            }
            w.ground.data[k] += diff * wt;
        }
    }
    // a landing that still ends up a step above or below the road or path beside it: retaining edge
    for j in b.j0 + 1..b.j1 - 1 {
        for i in b.i0 + 1..b.i1 - 1 {
            let k = apron_w.idx(i, j);
            if apron_w.data[k] > 0.0 && w.zone.data[k] == Z_NONE {
                let g = w.ground.data[k];
                if [(1, 0), (-1, 0), (0, 1), (0, -1)].iter().any(|(di, dj)| matches!(*w.zone.at(i + di, j + dj), Z_ROAD | Z_PATH | Z_PAVEMENT) && (*w.ground.at(i + di, j + dj) - g).abs() > 0.25) {
                    w.zone.data[k] = Z_CUT;
                }
            }
        }
    }
    let door_rows = (dr.height / ROW).round() as i32;
    for pk in &picks {
        let e = w.buildings[pk.b].edges[pk.edge].clone();
        let dir = e.b.sub(e.a).norm();
        let c = e.a.add(dir.mul(pk.t));
        let mut zone: Vec<(i32, i32)> = Vec::new();
        let mut gmax = f64::MIN;
        for (i, j) in rect_cells(c, dir, e.out, (-(half + dr.clear_side), half + dr.clear_side), (0.15, dr.clear_depth)) {
            if b.has(i, j) && *w.owner.at(i, j) == O_NONE {
                zone.push((i, j));
                if cell_centre(i, j).sub(c).dot(e.out) < 0.5 {
                    gmax = gmax.max(*w.ground.at(i, j) as f64);
                }
            }
        }
        if zone.is_empty() {
            continue;
        }
        let floor_row = (gmax / ROW - 1e-4).ceil() as i32;
        for (i, j) in &zone {
            w.reserve(*i, *j, floor_row + door_rows + 2, O_NONE);
        }
        let bp = &mut w.buildings[pk.b];
        bp.floor_row = floor_row;
        bp.has_door = true;
        bp.door_zone = zone;
        bp.openings.push(Opening { is_door: true, edge: pk.edge, t0: pk.t - half, t1: pk.t + half, r0: floor_row, r1: floor_row + door_rows, level: 0, centre: c, out: e.out });
    }

    // Heights. A building without a door sits on the highest ground under it, so it is never buried.
    let found_rows = (br.foundation_depth / ROW).round() as i32;
    for n in 0..w.buildings.len() {
        if !w.buildings[n].owned {
            continue;
        }
        let code = O_BUILDING0 + n as u32;
        let (mut gmin, mut gmax) = (f64::MAX, f64::MIN);
        let c = w.buildings[n].cells;
        for j in c.j0.max(b.j0)..c.j1.min(b.j1) {
            for i in c.i0.max(b.i0)..c.i1.min(b.i1) {
                if *w.owner.at(i, j) == code {
                    let g = *w.ground.at(i, j) as f64;
                    gmin = gmin.min(g);
                    gmax = gmax.max(g);
                    for (di, dj) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                        gmin = gmin.min(w.ground_cell(i + di, j + dj));
                    }
                }
            }
        }
        if gmin > gmax {
            w.buildings[n].owned = false;
            w.stats.skip("building has no cells of its own");
            continue;
        }
        let bp = &mut w.buildings[n];
        if !bp.has_door {
            bp.floor_row = (gmax / ROW - 1e-4).ceil() as i32;
        }
        bp.base_row = (gmin / ROW).floor() as i32 - found_rows;
        bp.eave_row = bp.floor_row + bp.levels as i32 * bp.level_rows;
        bp.top_row = bp.eave_row + bp.roof_rows.max(5);
    }
}

/// Phase 2: per building, plan and validate the windows against the finished
/// wall raster, then write every voxel of the building once.
pub fn write_buildings(w: &mut World) {
    let br = w.rules.buildings.clone();
    let b = w.bounds;
    let (m_plinth, m_glass, m_frame, m_inner, m_door) = (w.mat("plinth"), w.mat("glass"), w.mat("frame"), w.mat("interior"), w.mat("door"));
    let th_cells = (br.wall_thickness / CELL).round().max(2.0) as f32; // 3
    let win = br.window.clone();
    let win_rows = (win.height / ROW).round() as i32;
    let sill_rows = (win.sill / ROW).round() as i32;

    for n in 0..w.buildings.len() {
        if !w.buildings[n].owned {
            continue;
        }
        let code = O_BUILDING0 + n as u32;
        let mut bp = w.buildings[n].clone();
        let c = bp.cells;
        let (cw, ch) = (c.w(), c.h());
        let li = |i: i32, j: i32| (j - c.j0) as usize * cw + (i - c.i0) as usize;

        // Raster of this building: its own cells, travel lanes through it, wall layers, nearest wall edge.
        let mut outside = vec![true; cw * ch];
        let mut lane = vec![false; cw * ch];
        let mut any_lane = false;
        let mut pass_top = 0;
        for j in c.j0..c.j1 {
            for i in c.i0..c.i1 {
                if b.has(i, j) && *w.owner.at(i, j) == code {
                    outside[li(i, j)] = false;
                    let z = *w.zone.at(i, j);
                    if z == Z_ROAD || z == Z_PATH {
                        lane[li(i, j)] = true;
                        any_lane = true;
                        pass_top = pass_top.max(*w.reserved_top.at(i, j));
                    }
                }
            }
        }
        let d_full = edt(&outside, cw, ch);
        let d_low = if any_lane {
            let m: Vec<bool> = (0..cw * ch).map(|k| outside[k] || lane[k]).collect();
            edt(&m, cw, ch)
        } else {
            Vec::new()
        };
        if any_lane {
            pass_top += 2;
            bp.pass_top = pass_top;
        }
        let mut near_edge = vec![u16::MAX; cw * ch];
        let mut near_t = vec![0f32; cw * ch];
        let mut near_d = vec![f32::MAX; cw * ch];
        let band = br.wall_thickness + 0.15;
        for (ei, e) in bp.edges.iter().enumerate() {
            let i0 = (((e.a.x.min(e.b.x) - band) / CELL).floor() as i32).max(c.i0);
            let i1 = (((e.a.x.max(e.b.x) + band) / CELL).ceil() as i32).min(c.i1);
            let j0 = (((e.a.z.min(e.b.z) - band) / CELL).floor() as i32).max(c.j0);
            let j1 = (((e.a.z.max(e.b.z) + band) / CELL).ceil() as i32).min(c.j1);
            for j in j0..j1 {
                for i in i0..i1 {
                    let k = li(i, j);
                    if outside[k] {
                        continue;
                    }
                    let (t, d, _) = seg_project(cell_centre(i, j), e.a, e.b);
                    if d < band && (d as f32) < near_d[k] {
                        near_d[k] = d as f32;
                        near_edge[k] = ei as u16;
                        near_t[k] = (t * e.len) as f32;
                    }
                }
            }
        }
        // Depth layers of a wall, measured from its outer face: 0 = recess (first 10 cm),
        // 1 = glass or door leaf (15 cm, thick enough that no line of sight slips past it
        // on a diagonal wall), 2 = dark backing.
        let layer = |d: f32| -> u8 {
            if d < 0.1 {
                0
            } else if d < 0.25 {
                1
            } else {
                2
            }
        };

        // Window plan: regular bays per wall and floor, each one validated before it is accepted.
        if bp.windows {
            let mut planned: Vec<Opening> = Vec::new();
            for (ei, e) in bp.edges.iter().enumerate() {
                let usable = e.len - 2.0 * win.corner_margin;
                let nb = (usable / win.bay).floor() as i32;
                if nb < 1 {
                    continue;
                }
                let dir = e.b.sub(e.a).norm();
                let bay = usable / nb as f64;
                for lvl in 0..bp.levels {
                    let r0 = bp.floor_row + lvl as i32 * bp.level_rows + sill_rows;
                    let r1 = r0 + win_rows;
                    if r1 > bp.eave_row - 3 {
                        continue;
                    }
                    for k in 0..nb {
                        let tc = win.corner_margin + bay * (k as f64 + 0.5);
                        let (t0, t1) = (tc - win.width / 2.0, tc + win.width / 2.0);
                        planned.push(Opening { is_door: false, edge: ei, t0, t1, r0, r1, level: lvl, centre: e.a.add(dir.mul(tc)), out: e.out });
                    }
                }
            }
            let doors: Vec<Opening> = bp.openings.iter().filter(|o| o.is_door).cloned().collect();
            for o in planned {
                let e = &bp.edges[o.edge];
                let dir = e.b.sub(e.a).norm();
                // 1. not on top of a door
                if doors.iter().any(|d| d.edge == o.edge && o.t0 < d.t1 + 0.3 && o.t1 > d.t0 - 0.3 && o.r0 < d.r1 + 3) {
                    w.stats.drop_opening("window would overlap the door");
                    continue;
                }
                // 2. nothing built in front, and fully above the ground along its whole width
                let (mut blocked, mut gmax) = (false, f64::MIN);
                let tc = (o.t0 + o.t1) / 2.0;
                for (i, j) in rect_cells(e.a.add(dir.mul(tc)), dir, e.out, (o.t0 - tc, o.t1 - tc), (0.15, win.clear_out)) {
                    match w.owner.get(i, j) {
                        Some(ow) if *ow >= O_BUILDING0 => blocked = true,
                        None => blocked = true,
                        _ => {}
                    }
                    if cell_centre(i, j).sub(e.a).dot(e.out) < 0.6 {
                        gmax = gmax.max(w.ground_cell(i, j));
                    }
                }
                if blocked {
                    w.stats.drop_opening("window faces another building");
                    continue;
                }
                if (o.r0 as f64) * ROW < gmax + 0.3 {
                    w.stats.drop_opening("window would be below the ground");
                    continue;
                }
                // 3. the wall there really is this building's own, complete wall
                let need = ((o.t1 - o.t0) / CELL * 0.8) as usize;
                let (mut n0, mut n1, mut bad) = (0usize, 0usize, false);
                let reach = ((o.t1 - o.t0) / 2.0 + br.wall_thickness + 0.3) / CELL;
                let (ci, cj) = cell_of(o.centre);
                let rr = reach.ceil() as i32;
                for j in (cj - rr).max(c.j0)..(cj + rr + 1).min(c.j1) {
                    for i in (ci - rr).max(c.i0)..(ci + rr + 1).min(c.i1) {
                        let k = li(i, j);
                        if outside[k] || near_edge[k] as usize != o.edge || d_full[k] >= th_cells + 0.5 {
                            continue;
                        }
                        let t = near_t[k] as f64;
                        if t < o.t0 || t >= o.t1 {
                            continue;
                        }
                        if any_lane && o.r0 < pass_top && (d_low[k] - d_full[k]).abs() > 0.01 {
                            bad = true;
                        }
                        match layer(near_d[k]) {
                            0 => n0 += 1,
                            1 => n1 += 1,
                            _ => {}
                        }
                    }
                }
                if bad || n0 < need || n1 < need {
                    w.stats.drop_opening("wall too irregular or shared for a window");
                    continue;
                }
                bp.openings.push(o);
            }
        }
        // A door also needs a complete wall of its own; if not, it is removed from the plan (never half made).
        let mut keep: Vec<Opening> = Vec::new();
        for o in bp.openings.drain(..) {
            if !o.is_door {
                keep.push(o);
                continue;
            }
            let need = ((o.t1 - o.t0) / CELL * 0.8) as usize;
            let (mut n0, mut n1) = (0usize, 0usize);
            for k in 0..cw * ch {
                if outside[k] || near_edge[k] as usize != o.edge || d_full[k] >= th_cells + 0.5 || (any_lane && lane[k]) {
                    continue;
                }
                let t = near_t[k] as f64;
                if t >= o.t0 && t < o.t1 {
                    match layer(near_d[k]) {
                        0 => n0 += 1,
                        1 => n1 += 1,
                        _ => {}
                    }
                }
            }
            if n0 >= need && n1 >= need {
                keep.push(o);
            } else {
                w.stats.drop_opening("door wall too irregular");
                bp.has_door = false;
            }
        }
        bp.openings = keep;

        // Per edge list of openings for fast lookup while writing.
        let mut by_edge: Vec<Vec<usize>> = vec![Vec::new(); bp.edges.len()];
        for (k, o) in bp.openings.iter().enumerate() {
            by_edge[o.edge].push(k);
        }
        let across = P2::new(-bp.ridge_dir.z, bp.ridge_dir.x);
        let pitch_rows_per_cell = br.roof_pitch * CELL / ROW;

        for j in c.j0..c.j1 {
            for i in c.i0..c.i1 {
                let k = li(i, j);
                if outside[k] {
                    continue;
                }
                let d = d_full[k];
                let flat = bp.roof == "flat";
                let rise = match bp.roof.as_str() {
                    "flat" => 0.0,
                    "gabled" => (1.0 - (cell_centre(i, j).dot(across) - bp.ridge_mid).abs() / bp.ridge_half).clamp(0.0, 1.0) * bp.roof_rows as f64,
                    _ => ((d as f64 - 0.5).max(0.0) * pitch_rows_per_cell).min(bp.roof_rows as f64),
                };
                let roof_top = if flat { bp.eave_row + 2 } else { bp.eave_row + rise.floor() as i32 };
                if !flat {
                    w.roof_h.set(i, j, ((bp.eave_row as f64 + rise) * ROW) as f32);
                    w.roof_mat.set(i, j, bp.roof_mat);
                }
                let is_lane = any_lane && lane[k];
                let low_wall = any_lane && !is_lane && d_low[k] < th_cells + 0.5;
                let is_wall = d < th_cells + 0.5;
                let wall_top = if flat { bp.eave_row + 5 } else { roof_top };
                let start = if is_lane { pass_top - 2 } else { bp.base_row };
                if is_lane && start >= wall_top {
                    continue;
                }
                let lay = layer(near_d[k]);
                if is_wall && near_edge[k] != u16::MAX {
                    // remember which way this wall faces, so it can be lit as one flat wall
                    let o = bp.edges[near_edge[k] as usize].out;
                    let ang = o.z.atan2(o.x).rem_euclid(std::f64::consts::TAU);
                    w.wall_dir.set(i, j, 1 + ((ang / std::f64::consts::TAU * 126.0).round() as u8 % 126));
                }
                let ops: &[usize] = if is_wall && near_edge[k] != u16::MAX { &by_edge[near_edge[k] as usize] } else { &[] };
                let t = near_t[k] as f64;
                for r in start..wall_top.max(roof_top) {
                    let solid_wall = (is_wall && r < wall_top) || (low_wall && r < pass_top) || (is_lane && r < pass_top);
                    let m: u8 = if solid_wall {
                        let mut m = if r < bp.floor_row { m_plinth } else { bp.wall_mat };
                        for &oi in ops {
                            let o = &bp.openings[oi];
                            if r >= o.r0 && r < o.r1 && t >= o.t0 && t < o.t1 {
                                m = match lay {
                                    0 => M_SOLID_AIR,
                                    1 => {
                                        if o.is_door {
                                            m_door
                                        } else if r == o.r0 || r == o.r1 - 1 || t < o.t0 + CELL || t >= o.t1 - CELL {
                                            m_frame
                                        } else {
                                            m_glass
                                        }
                                    }
                                    _ => m_inner,
                                };
                                break;
                            }
                        }
                        m
                    } else if flat && r >= roof_top - 2 && r < roof_top {
                        bp.roof_mat
                    } else {
                        0
                    };
                    if m != 0 {
                        w.put(i, r, j, m, code);
                    }
                }
            }
        }
        bp.top_row = bp.eave_row + bp.roof_rows.max(5);
        w.buildings[n] = bp;
    }
}

pub fn _unused(_: f64) -> f64 {
    rand01(0, 0)
}
