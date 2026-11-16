//! Barriers (walls, fences, hedges) and props (trees, benches, lamps, cars).
//! Nothing here may stand in a travel lane or in front of a door; a spot is
//! checked in the plan and the prop is moved or dropped, never half placed.
//! Real barriers that the map puts on a path (bollards, gates) are kept and marked.

use crate::geo::{fill_rings, hash2, point_in_rings, rand01, ring_centroid, seg_project, CELL, P2};
use crate::ground::GroundOut;
use crate::osm::MapData;
use crate::world::*;

pub fn build_barriers(w: &mut World, map: &MapData, clearance: &Grid<f32>) {
    let core = w.core;
    for l in &map.lines {
        let Some(kind) = l.tags.get("barrier") else { continue };
        let Some(rule) = w.rules.barriers.get(kind).cloned() else {
            w.stats.skip("barrier type without a rule");
            continue;
        };
        let m = w.mat(&rule.material);
        let h = l.tags.metres("height").unwrap_or(rule.height).clamp(0.3, 8.0);
        let hw = (rule.width / 2.0).max(CELL * 0.5);
        for k in 0..l.pts.len().saturating_sub(1) {
            let (a, c) = (l.pts[k], l.pts[k + 1]);
            let i0 = ((a.x.min(c.x) - hw) / CELL).floor() as i32;
            let i1 = ((a.x.max(c.x) + hw) / CELL).ceil() as i32;
            let j0 = ((a.z.min(c.z) - hw) / CELL).floor() as i32;
            let j1 = ((a.z.max(c.z) + hw) / CELL).ceil() as i32;
            for j in j0.max(core.j0)..j1.min(core.j1) {
                for i in i0.max(core.i0)..i1.min(core.i1) {
                    let (_, d, _) = seg_project(cell_centre(i, j), a, c);
                    if d > hw.max(CELL * 0.71) {
                        continue;
                    }
                    // plan check: free column, not a lane, not in front of a door, not water
                    let z = *w.zone.at(i, j);
                    let own = *w.owner.at(i, j);
                    if (own != O_NONE && own != O_BARRIER) || *w.reserved_top.at(i, j) != 0 || z == Z_WATER {
                        continue;
                    }
                    // a barrier running along a building is the building's own wall, and it must
                    // never stand in front of a window
                    if clearance.get(i, j).map(|c| *c < 0.6).unwrap_or(false) {
                        continue;
                    }
                    w.owner.set(i, j, O_BARRIER);
                    let g = *w.ground.at(i, j) as f64;
                    let (r0, r1) = ((g / ROW).floor() as i32 - 3, ((g + h) / ROW).round() as i32);
                    for r in r0..r1 {
                        w.put(i, r, j, m, O_BARRIER);
                    }
                }
            }
        }
    }
}

fn foot_corners(pos: P2, yaw: f64, foot: [f64; 2], scale: f64) -> [P2; 5] {
    let (s, c) = yaw.sin_cos();
    let ax = P2::new(c, s);
    let az = P2::new(-s, c);
    let (fx, fz) = (foot[0] * scale, foot[1] * scale);
    [
        pos,
        pos.add(ax.mul(fx)).add(az.mul(fz)),
        pos.add(ax.mul(fx)).add(az.mul(-fz)),
        pos.add(ax.mul(-fx)).add(az.mul(fz)),
        pos.add(ax.mul(-fx)).add(az.mul(-fz)),
    ]
}

fn foot_cells(pos: P2, yaw: f64, foot: [f64; 2], scale: f64, grow: f64) -> Vec<(i32, i32)> {
    let (s, c) = yaw.sin_cos();
    let (fx, fz) = (foot[0] * scale + grow, foot[1] * scale + grow);
    let mut cells = rect_cells(pos, P2::new(c, s), P2::new(-s, c), (-fx, fx), (-fz, fz));
    cells.push(cell_of(pos));
    cells
}

/// True when every cell under the prop's footprint is free, fairly even ground.
fn spot_free(w: &World, pos: P2, yaw: f64, foot: [f64; 2], scale: f64, allow_lane: bool, taken: &Grid<bool>) -> bool {
    let (mut lo, mut hi) = (f32::MAX, f32::MIN);
    for (i, j) in foot_cells(pos, yaw, foot, scale, 0.0) {
        if !w.core.has(i, j) {
            return false;
        }
        let z = *w.zone.at(i, j);
        // never on water, on a retaining step, or where something already stands
        if *w.owner.at(i, j) != O_NONE || z == Z_WATER || z == Z_CUT || *taken.at(i, j) {
            return false;
        }
        if !allow_lane && (*w.reserved_top.at(i, j) != 0 || z == Z_ROAD || z == Z_PATH) {
            return false;
        }
        lo = lo.min(*w.ground.at(i, j));
        hi = hi.max(*w.ground.at(i, j));
    }
    hi - lo <= 0.4 * w.exaggeration as f32
}

fn mark_taken(pos: P2, yaw: f64, foot: [f64; 2], scale: f64, taken: &mut Grid<bool>) {
    for (i, j) in foot_cells(pos, yaw, foot, scale, 0.1) {
        taken.set(i, j, true);
    }
}

fn place(w: &mut World, model: &str, id: i64, pos: P2, yaw: f64, scale: f64, body: u8, on_lane: bool, taken: &mut Grid<bool>) {
    let rule = w.rules.props[model].clone();
    let cs = foot_corners(pos, yaw, rule.foot, scale);
    let hs: Vec<f64> = cs.iter().map(|p| w.ground_height(*p)).collect();
    let (y, slope) = if rule.upright {
        (hs.iter().cloned().fold(f64::MAX, f64::min), (0.0, 0.0))
    } else {
        // least-squares plane through the four corners: the prop lies on the slope
        let e = 0.5;
        let sx = (w.ground_height(P2::new(pos.x + e, pos.z)) - w.ground_height(P2::new(pos.x - e, pos.z))) / (2.0 * e);
        let sz = (w.ground_height(P2::new(pos.x, pos.z + e)) - w.ground_height(P2::new(pos.x, pos.z - e))) / (2.0 * e);
        let mut y = hs[0];
        // lift so no corner is below the ground
        for (k, c) in cs.iter().enumerate().skip(1) {
            let plane = hs[0] + sx * (c.x - pos.x) + sz * (c.z - pos.z);
            y = y.max(hs[0] + (hs[k] - plane));
        }
        (y, (sx, sz))
    };
    mark_taken(pos, yaw, rule.foot, scale, taken);
    w.props.push(PropInstance { model: model.to_string(), id, pos, y, yaw, slope, body, scale, on_lane });
}

pub fn build_props(w: &mut World, map: &MapData, gout: &GroundOut, lane_dist: &Grid<f32>) {
    let core = w.core;
    let mut taken = Grid::new(w.bounds, false);
    let models: Vec<(String, PropRule)> = w.rules.props.iter().map(|(k, v)| (k.clone(), v.clone())).collect();
    let lane_clear = w.rules.ground.lane_clear_height;
    let lane_d = |p: P2| -> f64 {
        let (i, j) = cell_of(p);
        lane_dist.get(i, j).map(|v| *v as f64).unwrap_or(99.0)
    };
    let clear_d = |p: P2| -> f64 {
        let (i, j) = cell_of(p);
        gout.clearance.get(i, j).map(|v| *v as f64).unwrap_or(99.0)
    };
    // direction pointing away from the nearest lane (so benches face the path)
    let away_from_lane = |p: P2| -> Option<P2> {
        let e = 0.3;
        let gx = lane_d(P2::new(p.x + e, p.z)) - lane_d(P2::new(p.x - e, p.z));
        let gz = lane_d(P2::new(p.x, p.z + e)) - lane_d(P2::new(p.x, p.z - e));
        let g = P2::new(gx, gz);
        if g.len() < 1e-6 {
            None
        } else {
            Some(g.norm())
        }
    };

    // 1. Props the map gives as points.
    for pt in &map.points {
        if !pt.p.x.is_finite() || !pt.p.z.is_finite() {
            w.stats.skip("point with broken coordinates");
            continue;
        }
        let (ci, cj) = cell_of(pt.p);
        if !core.has(ci, cj) {
            continue;
        }
        // real barriers on a lane are kept and reported to the game
        if let Some(kind) = pt.tags.get("barrier") {
            let z = *w.zone.at(ci, cj);
            if z == Z_ROAD || z == Z_PATH {
                w.barriers.push(MarkedBarrier { id: pt.id, kind: kind.to_string(), pos: pt.p });
            }
        }
        for (name, rule) in &models {
            let Some(node) = &rule.node else { continue };
            if !pt.tags.is(&node[0], &node[1]) {
                continue;
            }
            let mut scale = 1.0;
            let mut yaw = rand01(pt.id as u64, 11) * std::f64::consts::TAU;
            if name == "tree" {
                scale = 0.8 + 0.45 * rand01(pt.id as u64, 12);
            }
            if name == "bench" {
                if let Some(a) = away_from_lane(pt.p) {
                    // model faces -z (back rest at +z): turn the back away from the lane
                    yaw = a.x.atan2(a.z) * -1.0;
                }
            }
            // try the mapped spot first, then rings around it; a fixed order keeps it repeatable
            let mut done = false;
            'search: for ring in 0..=10 {
                let steps = if ring == 0 { 1 } else { 8 };
                for s in 0..steps {
                    let ang = s as f64 * std::f64::consts::TAU / steps as f64;
                    let p = P2::new(pt.p.x + ang.cos() * ring as f64 * 0.25, pt.p.z + ang.sin() * ring as f64 * 0.25);
                    let mut sc = scale;
                    if name == "tree" {
                        // crown must not reach into a building, and must be above the lane clear height
                        let c = clear_d(p);
                        if c < 1.9 * sc {
                            sc = c / 1.9;
                        }
                        if sc < 0.5 {
                            continue;
                        }
                        if lane_d(p) < 1.9 * sc && 2.2 * sc < lane_clear + 0.1 {
                            if c >= 1.9 * 1.25 {
                                sc = 1.25;
                            } else {
                                continue;
                            }
                        }
                    }
                    if spot_free(w, p, yaw, rule.foot, sc, rule.on_lane, &taken) {
                        // marked when any part of it stands on a travel lane
                        let on_lane = rule.on_lane && {
                            let r = (rule.foot[0].max(rule.foot[1]) / CELL).ceil() as i32 + 1;
                            let (ci, cj) = cell_of(p);
                            (cj - r..=cj + r).any(|j| (ci - r..=ci + r).any(|i| matches!(w.zone.get(i, j), Some(&Z_ROAD) | Some(&Z_PATH))))
                        };
                        place(w, name, pt.id, p, yaw, sc, 0, on_lane, &mut taken);
                        done = true;
                        break 'search;
                    }
                    if rule.on_lane {
                        break 'search; // a real barrier is placed where it is, or not at all
                    }
                }
            }
            if !done {
                w.stats.skip(&format!("{name}: no free spot"));
            }
        }
    }

    // 2. Trees in woods: one per grid square of a world-fixed grid, shifted by a hash of the square.
    let sp = w.rules.forest_tree_spacing;
    let area_rules = w.rules.areas.clone();
    for a in &map.areas {
        let woods = area_rules.iter().any(|r| r.trees && a.tags.get(&r.key).map(|v| r.values.iter().any(|x| x == v)).unwrap_or(false));
        if !woods {
            continue;
        }
        let (mut x0, mut z0, mut x1, mut z1) = (f64::MAX, f64::MAX, f64::MIN, f64::MIN);
        for p in a.rings.iter().flatten() {
            x0 = x0.min(p.x);
            z0 = z0.min(p.z);
            x1 = x1.max(p.x);
            z1 = z1.max(p.z);
        }
        let (cmin, cmax) = (core.min(), core.max());
        let (gx0, gx1) = ((x0.max(cmin.x) / sp).floor() as i64, (x1.min(cmax.x) / sp).ceil() as i64);
        let (gz0, gz1) = ((z0.max(cmin.z) / sp).floor() as i64, (z1.min(cmax.z) / sp).ceil() as i64);
        for gz in gz0..gz1 {
            for gx in gx0..gx1 {
                let key = hash2(gx as u64, gz as u64);
                let p = P2::new((gx as f64 + 0.15 + 0.7 * rand01(key, 1)) * sp, (gz as f64 + 0.15 + 0.7 * rand01(key, 2)) * sp);
                if !point_in_rings(p, &a.rings) {
                    continue;
                }
                let mut sc = 0.9 + 0.5 * rand01(key, 3);
                let c = clear_d(p);
                if c < 1.9 * sc {
                    sc = c / 1.9;
                }
                if sc < 0.5 || (lane_d(p) < 1.9 * sc && 2.2 * sc < lane_clear + 0.1) {
                    continue;
                }
                let rule = &w.rules.props["tree"];
                let yaw = rand01(key, 4) * std::f64::consts::TAU;
                if spot_free(w, p, yaw, rule.foot, sc, false, &taken) {
                    place(w, "tree", (key >> 1) as i64, p, yaw, sc, 0, false, &mut taken);
                }
            }
        }
    }

    // 3. Parked cars: only inside mapped parking areas, never on the aisles (those are travel lanes).
    let pk = w.rules.parking.clone();
    let car = w.rules.props["car"].clone();
    let bodies: Vec<u8> = car.bodies.iter().map(|b| w.mat(b)).collect();
    for a in &map.areas {
        let Some(am) = a.tags.get("amenity") else { continue };
        if am != "parking" && am != "parking_space" {
            continue;
        }
        if a.tags.get("parking").map(|v| v == "underground" || v == "multi-storey").unwrap_or(false) {
            continue;
        }
        let ring = &a.rings[0];
        let cen = ring_centroid(ring);
        let (mut best, mut dir) = (0.0, P2::new(1.0, 0.0));
        for k in 0..ring.len() {
            let (p, q) = (ring[k], ring[(k + 1) % ring.len()]);
            if p.dist(q) > best {
                best = p.dist(q);
                dir = q.sub(p).norm();
            }
        }
        let nrm = P2::new(-dir.z, dir.x);
        // cars stand across the long side
        let yaw = (-nrm.x).atan2(nrm.z);
        let yaw = if am == "parking_space" { (-dir.x).atan2(dir.z) } else { yaw };
        let n = if am == "parking_space" { 0 } else { 40 };
        for r in -n..=n {
            for c in -n..=n {
                let p = cen.add(dir.mul(c as f64 * pk.car_spacing)).add(nrm.mul(r as f64 * pk.row_spacing));
                let key = hash2(a.id as u64, hash2(r as u64, c as u64));
                if rand01(key, 5) > pk.occupancy {
                    continue;
                }
                let foot = [car.foot[0] + pk.edge_margin, car.foot[1] + pk.edge_margin];
                if !foot_corners(p, yaw, foot, 1.0).iter().all(|q| point_in_rings(*q, &a.rings)) {
                    continue;
                }
                if spot_free(w, p, yaw, car.foot, 1.0, false, &taken) {
                    let body = bodies[(key % bodies.len().max(1) as u64) as usize];
                    place(w, "car", (key >> 1) as i64, p, yaw, 1.0, body, false, &mut taken);
                }
            }
        }
    }
    let _ = fill_rings::<fn(i32, i32)>;
}
