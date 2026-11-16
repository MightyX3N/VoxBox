//! Automatic checks on a built world. Every check looks at the finished
//! world (voxels, ground, props), not at what the builder meant to do.

use crate::geo::{CELL, P2};
use crate::world::*;

#[derive(Clone, Debug)]
pub struct Check {
    pub name: &'static str,
    pub pass: bool,
    pub tested: usize,
    pub failed: usize,
    pub detail: String,
}

fn check(name: &'static str, tested: usize, failed: usize, detail: String) -> Check {
    Check { name, pass: failed == 0, tested, failed, detail }
}

fn solid(m: u8) -> bool {
    m != 0 && m != M_SOLID_AIR
}

fn prop_cells(w: &World, p: &PropInstance) -> Vec<(i32, i32)> {
    let rule = &w.rules.props[&p.model];
    let (s, c) = p.yaw.sin_cos();
    let (fx, fz) = (rule.foot[0] * p.scale, rule.foot[1] * p.scale);
    let mut v = rect_cells(p.pos, P2::new(c, s), P2::new(-s, c), (-fx, fx), (-fz, fz));
    v.push(cell_of(p.pos));
    v
}

pub fn run(w: &World) -> Vec<Check> {
    let mut out = Vec::new();
    let core = w.core;
    let b = w.bounds;
    let (m_glass, m_frame, m_door) = (w.mat("glass"), w.mat("frame"), w.mat("door"));

    out.push(check("no write ever hit another owner's column or reserved space", 1, w.stats.rejected_writes as usize, format!("{} refused writes", w.stats.rejected_writes)));

    // Props by cell, for the door and lane checks.
    let mut prop_at = Grid::new(b, 0u32);
    for (k, p) in w.props.iter().enumerate() {
        for (i, j) in prop_cells(w, p) {
            prop_at.set(i, j, k as u32 + 1);
        }
    }

    // Windows and doors, from the finished voxels.
    let (mut nwin, mut bad_win, mut ndoor, mut bad_door, mut no_door) = (0, 0, 0, 0, 0);
    let (mut first_win, mut first_door) = (String::new(), String::new());
    let dr = &w.rules.buildings.door;
    for bp in w.buildings.iter().filter(|b| b.owned) {
        if !bp.has_door {
            no_door += 1;
        }
        for o in &bp.openings {
            let e = &bp.edges[o.edge];
            let dir = e.b.sub(e.a).norm();
            // Look at the opening from outside: along every line of sight into the wall, the first
            // thing hit must be the glass (or the door leaf), never wall, and nothing may stand in front.
            let (mut leaf, mut leaf_n, mut front, mut gmax) = (0, 0, 0, f64::MIN);
            let mut t = o.t0 + 0.05;
            while t < o.t1 {
                let on = e.a.add(dir.mul(t));
                for r in o.r0..o.r1 {
                    leaf_n += 1;
                    for depth in [0.02, 0.07, 0.12, 0.17, 0.22, 0.27, 0.32] {
                        let (i, j) = cell_of(on.sub(o.out.mul(depth)));
                        let m = w.voxel(i, r, j);
                        if solid(m) {
                            if (o.is_door && m == m_door) || (!o.is_door && (m == m_glass || m == m_frame)) {
                                leaf += 1;
                            }
                            break;
                        }
                    }
                    for v in [0.15, 0.25, 0.35, 0.45, 0.6, 0.8] {
                        if !o.is_door && v > 0.5 {
                            continue;
                        }
                        let (fi, fj) = cell_of(on.add(o.out.mul(v)));
                        if solid(w.voxel(fi, r, fj)) || w.inside_building(fi, r, fj) {
                            front += 1;
                        }
                    }
                }
                for v in [0.15, 0.25, 0.35] {
                    let (fi, fj) = cell_of(on.add(o.out.mul(v)));
                    gmax = gmax.max(w.ground_cell(fi, fj));
                }
                t += CELL;
            }
            // count the leaf voxels themselves in the middle row: the whole width must be there
            let (ci, cj) = cell_of(o.centre);
            let rr = ((o.t1 - o.t0) / CELL) as i32 + 4;
            let mid = (o.r0 + o.r1) / 2;
            let mut width = 0;
            for j in cj - rr..=cj + rr {
                for i in ci - rr..=ci + rr {
                    let m = w.voxel(i, mid, j);
                    if (o.is_door && m == m_door) || (!o.is_door && (m == m_glass || m == m_frame)) {
                        let (tt, d, _) = crate::geo::seg_project(cell_centre(i, j), e.a, e.b);
                        if d < 0.45 && tt * e.len >= o.t0 - 0.05 && tt * e.len < o.t1 + 0.05 {
                            width += 1;
                        }
                    }
                }
            }
            let need = ((o.t1 - o.t0) / CELL * 0.8) as i32;
            let mut ok = leaf_n > 0 && leaf as f64 >= 0.75 * leaf_n as f64 && width >= need && front == 0;
            let mut why = format!("leaf seen in {leaf}/{leaf_n} lines of sight, {width} leaf voxels across (need {need}), solid in front {front}");
            if o.is_door {
                ndoor += 1;
                let floor = o.r0 as f64 * ROW;
                let (mut buried, mut blocked, mut gmin_front) = (0, 0, f64::MAX);
                for (i, j) in &bp.door_zone {
                    let g = w.ground_cell(*i, *j);
                    // right at the door the ground is never above the threshold; further out it may
                    // rise by at most one step
                    let near = cell_centre(*i, *j).sub(o.centre).dot(o.out) < 0.5;
                    if g > floor + if near { 0.005 } else { dr.max_step * w.exaggeration } {
                        buried += 1;
                    }
                    if *prop_at.at(*i, *j) != 0 {
                        blocked += 1;
                    }
                    let fr = (g / ROW).floor() as i32 + 1;
                    for r in fr.max(o.r0)..o.r1 {
                        if solid(w.voxel(*i, r, *j)) {
                            blocked += 1;
                        }
                    }
                    if cell_centre(*i, *j).sub(o.centre).dot(o.out) < 0.45 {
                        gmin_front = gmin_front.min(floor - g);
                    }
                }
                if buried > 0 || blocked > 0 || gmin_front > dr.max_step * w.exaggeration + 0.005 {
                    ok = false;
                    why = format!("{why}, ground above threshold in {buried} cells, {blocked} blocked, step up {gmin_front:.2} m");
                }
                if !ok {
                    bad_door += 1;
                    if first_door.is_empty() {
                        first_door = format!("building {} at ({:.1}, {:.1}): {why}", bp.id, o.centre.x, o.centre.z);
                    }
                }
            } else {
                nwin += 1;
                if (o.r0 as f64) * ROW < gmax + 0.25 {
                    ok = false;
                    why = format!("{why}, sill only {:.2} m above ground", o.r0 as f64 * ROW - gmax);
                }
                if !ok {
                    bad_win += 1;
                    if first_win.is_empty() {
                        first_win = format!("building {} at ({:.1}, {:.1}) level {}: {why}", bp.id, o.centre.x, o.centre.z, o.level);
                    }
                }
            }
        }
    }
    out.push(check("every planned window exists as glass, with a free recess, free space in front, above the ground", nwin, bad_win, first_win));
    out.push(check("every door has its leaf, a free opening, clear space in front, ground at or below the threshold", ndoor, bad_door, first_door));

    // Walls reach below the ground everywhere along the outline.
    let (mut nwall, mut gap) = (0, 0);
    let mut first = String::new();
    for (n, bp) in w.buildings.iter().enumerate().filter(|(_, b)| b.owned) {
        let code = O_BUILDING0 + n as u32;
        let c = bp.cells;
        for j in c.j0.max(b.j0 + 1)..c.j1.min(b.j1 - 1) {
            for i in c.i0.max(b.i0 + 1)..c.i1.min(b.i1 - 1) {
                if *w.owner.at(i, j) != code || matches!(*w.zone.at(i, j), Z_ROAD | Z_PATH) {
                    continue;
                }
                for (di, dj) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    let (ni, nj) = (i + di, j + dj);
                    if *w.owner.at(ni, nj) >= O_BUILDING0 {
                        continue;
                    }
                    nwall += 1;
                    let gr = (*w.ground.at(ni, nj) as f64 / ROW).floor() as i32;
                    // solid from one row below the outside ground up to one row above it
                    if !(gr - 1..=gr + 1).all(|r| w.voxel(i, r, j) != 0) {
                        gap += 1;
                        if first.is_empty() {
                            first = format!("building {} at cell ({i}, {j}): no wall at ground row {gr}", bp.id);
                        }
                    }
                }
            }
        }
    }
    out.push(check("walls continue below the ground along every outline (no gaps, no floating buildings)", nwall, gap, first));

    // Props sit on the ground.
    let (mut bad, mut first) = (0, String::new());
    for p in &w.props {
        let rule = &w.rules.props[&p.model];
        let (s, c) = p.yaw.sin_cos();
        let (ax, az) = (P2::new(c, s), P2::new(-s, c));
        let (fx, fz) = (rule.foot[0] * p.scale, rule.foot[1] * p.scale);
        let (mut lo, mut hi) = (f64::MAX, f64::MIN);
        for (u, v) in [(0.0, 0.0), (fx, fz), (fx, -fz), (-fx, fz), (-fx, -fz)] {
            let q = p.pos.add(ax.mul(u)).add(az.mul(v));
            let base = p.y + p.slope.0 * (q.x - p.pos.x) + p.slope.1 * (q.z - p.pos.z);
            let g = base - w.ground_height(q);
            lo = lo.min(g);
            hi = hi.max(g);
        }
        // lowest point touches the ground (within 2 cm), no point floats or sinks more than 30 cm
        // upright things (trees, posts) stand in the ground at their lowest corner; lying things rest on it
        let ok = if rule.upright { hi <= 0.02 && lo >= -1.2 * w.exaggeration } else { lo >= -0.02 && lo <= 0.02 && hi <= 0.3 * w.exaggeration };
        if !ok {
            bad += 1;
            if first.is_empty() {
                first = format!("{} at ({:.1}, {:.1}): gap to ground {lo:.2}..{hi:.2} m", p.model, p.pos.x, p.pos.z);
            }
        }
    }
    out.push(check("no prop floats above or is buried in the ground", w.props.len(), bad, first));

    // Travel lanes are clear.
    let (mut nlane, mut bad, mut first) = (0, 0, String::new());
    for j in core.j0..core.j1 {
        for i in core.i0..core.i1 {
            if !matches!(*w.zone.at(i, j), Z_ROAD | Z_PATH) {
                continue;
            }
            nlane += 1;
            let g = (*w.ground.at(i, j) as f64 / ROW).floor() as i32;
            let top = *w.reserved_top.at(i, j);
            let mut hit = (g + 1..top).any(|r| solid(w.voxel(i, r, j)));
            let pk = *prop_at.at(i, j);
            if pk != 0 && !w.props[pk as usize - 1].on_lane {
                hit = true;
            }
            if hit {
                bad += 1;
                if first.is_empty() {
                    first = format!("cell ({i}, {j}){}", if pk != 0 { format!(": {} stands on it", w.props[pk as usize - 1].model) } else { String::new() });
                }
            }
        }
    }
    out.push(check("nothing solid stands in any travel lane of any road or path", nlane, bad, first));

    // Roads: level across, kerb a fixed height, smooth along. Sampled away from junctions,
    // where two roads at different heights are blended into each other on purpose.
    let kerb = w.rules.ground.kerb_height;
    let all_pts: Vec<(i64, P2)> = w.travel.iter().flat_map(|t| t.pts.iter().map(move |p| (t.way_id, P2::new(p[0], p[2])))).collect();
    let (cmin, cmax) = (core.min(), core.max());
    let (mut ncross, mut bad_cross, mut nkerb, mut bad_kerb, mut nsm, mut bad_sm) = (0, 0, 0, 0, 0, 0);
    let (mut f_cross, mut f_kerb, mut f_sm) = (String::new(), String::new(), String::new());
    for t in &w.travel {
        let n = t.pts.len();
        for k in (2..n.saturating_sub(2)).step_by(2) {
            let p = P2::new(t.pts[k][0], t.pts[k][2]);
            if p.x < cmin.x + 8.0 || p.x > cmax.x - 8.0 || p.z < cmin.z + 8.0 || p.z > cmax.z - 8.0 {
                continue;
            }
            let (a, c) = (P2::new(t.pts[k - 2][0], t.pts[k - 2][2]), P2::new(t.pts[k + 2][0], t.pts[k + 2][2]));
            // smooth along: no sudden change of grade
            let (ga, gc) = ((t.pts[k][1] - t.pts[k - 2][1]) / a.dist(p).max(0.01), (t.pts[k + 2][1] - t.pts[k][1]) / c.dist(p).max(0.01));
            // a path that steps down a kerb to cross a road is not judged there
            let (pi, pj) = cell_of(p);
            let at_road = t.kind != "road" && (-30..=30).step_by(3).any(|dj| (-30..=30).step_by(3).any(|di| matches!(w.zone.get(pi + di, pj + dj), Some(&Z_ROAD) | Some(&Z_PAVEMENT))));
            // at a junction a way may turn from one street into another; the through line is what is smooth
            let junction = all_pts.iter().any(|(id, q)| *id != t.way_id && q.dist(p) < 3.0);
            if at_road || junction {
                continue;
            }
            nsm += 1;
            if (ga - gc).abs() > 0.12 * w.exaggeration {
                bad_sm += 1;
                if f_sm.is_empty() {
                    f_sm = format!("{} {} at ({:.1}, {:.1}): grade {:.2} -> {:.2}", t.class, t.way_id, p.x, p.z, ga, gc);
                }
            }
            if t.kind != "road" {
                continue;
            }
            let straight = c.sub(p).norm().dot(p.sub(a).norm()) > 0.995;
            let alone = !all_pts.iter().any(|(id, q)| *id != t.way_id && q.dist(p) < 12.0);
            if !alone || !straight {
                continue;
            }
            let side = P2::new(-(c.z - a.z), c.x - a.x).norm();
            let hw = t.half_width[k];
            let (gl, gr) = (w.ground_height(p.add(side.mul(hw * 0.8))), w.ground_height(p.sub(side.mul(hw * 0.8))));
            ncross += 1;
            // 2 cm, plus an allowance on steep roads: there a few degrees of doubt about which way
            // is 'across' already shows up as height
            let tol = 0.02 + 0.08 * ga.abs().max(gc.abs()) * hw;
            if (gl - gr).abs() > tol || (gl - t.pts[k][1]).abs() > tol {
                bad_cross += 1;
                if f_cross.is_empty() {
                    f_cross = format!("way {} at ({:.1}, {:.1}): left {:.3}, centre {:.3}, right {:.3}", t.way_id, p.x, p.z, gl, t.pts[k][1], gr);
                }
            }
            for sgn in [1.0, -1.0] {
                let q1 = p.add(side.mul(sgn * (hw + 0.35)));
                let q2 = p.add(side.mul(sgn * (hw + 0.65)));
                let (c1, c2) = (cell_of(q1), cell_of(q2));
                // not next to a path: kerbs are dropped where paths cross
                let near_path = (-30..=30).step_by(3).any(|dj| (-30..=30).step_by(3).any(|di| w.zone.get(c1.0 + di, c1.1 + dj) == Some(&Z_PATH)));
                // not within three metres of where a pavement ends: there the road with and the road without a kerb run into each other
                let along = c.sub(a).norm();
                let ends = [-3.0, -2.0, -1.0, 1.0, 2.0, 3.0].iter().any(|d| {
                    let cc = cell_of(q1.add(along.mul(*d)));
                    w.zone.get(cc.0, cc.1) != Some(&Z_PAVEMENT)
                });
                if !near_path && !ends && w.zone.get(c1.0, c1.1) == Some(&Z_PAVEMENT) && w.zone.get(c2.0, c2.1) == Some(&Z_PAVEMENT) {
                    nkerb += 1;
                    let step = w.ground_cell(c1.0, c1.1) - w.ground_height(p.add(side.mul(sgn * (hw - 0.35))));
                    if (step - kerb).abs() > 0.02 {
                        bad_kerb += 1;
                        if f_kerb.is_empty() {
                            f_kerb = format!("way {} at ({:.1}, {:.1}): kerb {:.3} m", t.way_id, p.x, p.z, step);
                        }
                    }
                }
            }
        }
    }
    // No travel line has a step in it, anywhere, junctions and road crossings included.
    let (mut nstep, mut bad_step, mut f_step) = (0, 0, String::new());
    for t in &w.travel {
        for k in 1..t.pts.len() {
            let (a, c) = (P2::new(t.pts[k - 1][0], t.pts[k - 1][2]), P2::new(t.pts[k][0], t.pts[k][2]));
            let inside = |p: P2| p.x > cmin.x + 1.0 && p.x < cmax.x - 1.0 && p.z > cmin.z + 1.0 && p.z < cmax.z - 1.0;
            if !inside(a) || !inside(c) || a.dist(c) < 0.3 {
                continue;
            }
            nstep += 1;
            let rise = (t.pts[k][1] - t.pts[k - 1][1]).abs() / a.dist(c);
            if rise > 1.0 * w.exaggeration {
                bad_step += 1;
                if f_step.is_empty() {
                    f_step = format!("{} {} at ({:.1}, {:.1}): {:.2} m up per metre", t.class, t.way_id, c.x, c.z, rise);
                }
            }
        }
    }
    out.push(check("no road or path has a step along it (junctions and crossings included)", nstep, bad_step, f_step));
    out.push(check("roads are level across their width", ncross, bad_cross, f_cross));
    out.push(check("pavements are exactly one kerb height above the road", nkerb, bad_kerb, f_kerb));
    out.push(check("roads and paths are smooth along their length", nsm, bad_sm, f_sm));

    // Ground has no cliffs except kerbs and water edges.
    // "corridor" = a road with its pavements (a path running on a pavement is part of it)
    let m_pave = w.mat("pavement");
    let corridor = |i: i32, j: i32| matches!(*w.zone.at(i, j), Z_ROAD | Z_PAVEMENT) || (*w.zone.at(i, j) == Z_PATH && *w.mat.at(i, j) == m_pave);
    let (mut n, mut bad, mut first) = (0usize, 0, String::new());
    for j in core.j0..core.j1 - 1 {
        for i in core.i0..core.i1 - 1 {
            let (g, z) = (*w.ground.at(i, j), *w.zone.at(i, j));
            for (ni, nj) in [(i + 1, j), (i, j + 1)] {
                let (g2, z2) = (*w.ground.at(ni, nj), *w.zone.at(ni, nj));
                n += 1;
                let lim = if z == Z_WATER || z2 == Z_WATER || z == Z_BUILDING || z2 == Z_BUILDING || z == Z_CUT || z2 == Z_CUT {
                    f32::MAX
                } else if (corridor(i, j) && z2 == Z_PATH && !corridor(ni, nj)) || (corridor(ni, nj) && z == Z_PATH && !corridor(i, j)) {
                    // a path running beside a road at another height: a retaining step between the two
                    // (where a path really joins a road, the travel-line check below catches any step)
                    f32::MAX
                } else if (z == Z_ROAD) != (z2 == Z_ROAD) {
                    kerb as f32 + 0.3 * w.exaggeration as f32
                } else {
                    0.3 * w.exaggeration as f32
                };
                if (g - g2).abs() > lim {
                    bad += 1;
                    if first.is_empty() {
                        first = format!("cells ({i}, {j}) and ({ni}, {nj}): {:.2} m step", (g - g2).abs());
                    }
                }
            }
        }
    }
    out.push(check("the ground has no sudden steps other than kerbs, water edges and marked retaining walls", n, bad, first));
    let _ = no_door;
    out
}

/// Compares two builds of overlapping areas. Everything in the shared area must be identical:
/// the result may not depend on where a built area ends.
pub fn compare(a: &World, b: &World) -> Check {
    let (i0, j0) = (a.core.i0.max(b.core.i0), a.core.j0.max(b.core.j0));
    let (i1, j1) = (a.core.i1.min(b.core.i1), a.core.j1.min(b.core.j1));
    let (mut n, mut bad, mut first) = (0usize, 0usize, String::new());
    let mut note = |bad: &mut usize, first: &mut String, s: String| {
        *bad += 1;
        if first.is_empty() {
            *first = s;
        }
    };
    let base_shift = ((a.base_height - b.base_height) / ROW).round() as i32;
    let _ = base_shift;
    for j in j0..j1 {
        for i in i0..i1 {
            n += 1;
            if a.ground.at(i, j).to_bits() != b.ground.at(i, j).to_bits() || a.zone.at(i, j) != b.zone.at(i, j) || a.palette[*a.mat.at(i, j) as usize] != b.palette[*b.mat.at(i, j) as usize] {
                note(&mut bad, &mut first, format!("ground differs at cell ({i}, {j}): {} vs {}", a.ground.at(i, j), b.ground.at(i, j)));
            }
        }
    }
    for (na, pa) in a.buildings.iter().enumerate().filter(|(_, p)| p.owned) {
        let Some((nb, pb)) = b.buildings.iter().enumerate().find(|(_, p)| p.id == pa.id && p.owned) else { continue };
        let (ca, cb) = (O_BUILDING0 + na as u32, O_BUILDING0 + nb as u32);
        let c = pa.cells;
        for j in c.j0..c.j1 {
            for i in c.i0..c.i1 {
                let (oa, ob) = (a.owner.get(i, j) == Some(&ca), b.owner.get(i, j) == Some(&cb));
                if oa != ob {
                    note(&mut bad, &mut first, format!("building {} owns different cells at ({i}, {j})", pa.id));
                    continue;
                }
                if !oa {
                    continue;
                }
                for r in pa.base_row.min(pb.base_row) - 2..pa.top_row.max(pb.top_row) + 8 {
                    n += 1;
                    let (va, vb) = (a.voxel(i, r, j), b.voxel(i, r, j));
                    if va != vb && !(va != 0 && vb != 0 && a.palette.get(va as usize) == b.palette.get(vb as usize)) && !(va == M_SOLID_AIR && vb == M_SOLID_AIR) {
                        note(&mut bad, &mut first, format!("building {} differs at ({i}, {r}, {j})", pa.id));
                    }
                }
            }
        }
    }
    let inner = |p: &PropInstance| p.pos.x > i0 as f64 * CELL + 4.0 && p.pos.x < i1 as f64 * CELL - 4.0 && p.pos.z > j0 as f64 * CELL + 4.0 && p.pos.z < j1 as f64 * CELL - 4.0;
    let key = |p: &PropInstance| (p.model.clone(), p.id, p.pos.x.to_bits(), p.pos.z.to_bits(), p.y.to_bits());
    let mut sa: Vec<_> = a.props.iter().filter(|p| inner(p)).map(key).collect();
    let mut sb: Vec<_> = b.props.iter().filter(|p| inner(p)).map(key).collect();
    sa.sort();
    sb.sort();
    n += sa.len();
    if sa != sb {
        note(&mut bad, &mut first, format!("props differ: {} vs {}", sa.len(), sb.len()));
    }
    check("two overlapping builds are identical in the shared area (no seams)", n, bad, first)
}

/// Returns a copy of the map with about a third of its features damaged in a repeatable way.
/// Used to prove that bad data never stops a build or produces a half-made feature.
pub fn damage(map: &crate::osm::MapData, seed: u64) -> crate::osm::MapData {
    use crate::geo::hash2;
    let mut m = map.clone();
    for (n, a) in m.areas.iter_mut().enumerate() {
        match hash2(n as u64, seed) % 12 {
            0 => a.tags.0.retain(|(k, _)| k == "building"), // only the bare type left
            1 => {
                // outline cut down to a sliver
                for r in a.rings.iter_mut() {
                    r.truncate(3);
                }
            }
            2 => {
                // outline that doubles back on itself
                for r in a.rings.iter_mut() {
                    let mut back = r.clone();
                    back.reverse();
                    r.extend(back);
                }
            }
            3 => a.tags.0.push(("building:levels".into(), "lots".into())),
            _ => {}
        }
    }
    for (n, l) in m.lines.iter_mut().enumerate() {
        match hash2(n as u64, seed + 1) % 12 {
            0 => {
                l.pts.truncate(1);
                l.node_ids.truncate(1);
            }
            1 => l.tags.0.push(("width".into(), "wide".into())),
            2 => {
                // a point thrown ten kilometres away
                if let Some(p) = l.pts.last_mut() {
                    p.x += 10_000.0;
                }
            }
            3 => l.tags.0.retain(|(k, _)| k == "highway" || k == "barrier"),
            _ => {}
        }
    }
    for (n, p) in m.points.iter_mut().enumerate() {
        if hash2(n as u64, seed + 2) % 10 == 0 {
            p.p.x = f64::NAN;
        }
    }
    m
}
