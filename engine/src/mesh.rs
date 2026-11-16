//! Turns the world into triangles: the ground (merged flat or sloped patches
//! of 10 cm columns) and the voxels (greedy merged faces), in tiles.

use crate::geo::CELL;
use crate::world::*;
use std::collections::BTreeMap;

pub const TILE: i32 = 128;

#[derive(Default, Clone)]
pub struct MeshBuf {
    pub pos: Vec<f32>,
    pub col: Vec<u8>,
    pub idx: Vec<u32>,
}
impl MeshBuf {
    fn quad(&mut self, p: [[f32; 3]; 4], n: [f32; 3], c: [u8; 3]) {
        self.quad_a(p, n, c, 255)
    }
    /// alpha 255 = use this colour; 0 = ground top, colour comes from the ground colour map;
    /// 128 = ground side face, colour comes from the higher cell beside it
    fn quad_a(&mut self, p: [[f32; 3]; 4], _n: [f32; 3], c: [u8; 3], a: u8) {
        let b = (self.pos.len() / 3) as u32;
        for v in p {
            self.pos.extend_from_slice(&v);
            self.col.extend_from_slice(&[c[0], c[1], c[2], a]);
        }
        self.idx.extend_from_slice(&[b, b + 1, b + 2, b, b + 2, b + 3]);
    }
    pub fn tris(&self) -> usize {
        self.idx.len() / 3
    }
}

struct Leaf {
    a: f32,
    bi: f32,
    bj: f32,
    i0: i32,
    j0: i32,
    s: i32,
    mat: u8,
}

/// Steps in a surface no smaller than this are joined into one sloped patch (metres).
pub const MERGE_TOLERANCE: f32 = 0.012;
/// Height differences between neighbouring patches smaller than this get no side face (metres).
pub const CRACK: f32 = 0.0001;
/// Side faces lower than this are slivers between two patches of the same surface, not real steps (metres).
pub const SLIVER: f32 = 0.03;

/// Mesh of a surface made of 10 cm columns at true height: the ground, or the pitched roofs.
/// A square of columns is merged into one patch when all of them lie on one plane (within
/// MERGE_TOLERANCE) and share a material; otherwise each column is its own flat top. Side
/// faces close every step between patches, so the surface has no holes.
pub fn surface_mesh(w: &World, roofs: bool) -> BTreeMap<(i32, i32), MeshBuf> {
    let area = if roofs { w.bounds } else { w.core };
    let tol = MERGE_TOLERANCE;
    let mut leaves: Vec<Leaf> = Vec::new();
    let mut leaf_of = Grid::new(area, u32::MAX);
    // The ground runs one cell in under every wall, so the two always overlap and no gap can open
    // between them; only cells deeper inside a building are left out.
    let bld = |i: i32, j: i32| w.zone.get(i, j).map(|z| *z == Z_BUILDING).unwrap_or(false);
    let skip = |i: i32, j: i32| if roofs { w.roof_h.at(i, j).is_nan() } else { bld(i, j) && bld(i - 1, j) && bld(i + 1, j) && bld(i, j - 1) && bld(i, j + 1) };
    let h = |i: i32, j: i32| if roofs { *w.roof_h.at(i, j) } else { *w.ground.at(i, j) };
    // the ground is one surface whatever it is made of: its colours come from a colour map
    let m = |i: i32, j: i32| if roofs { *w.roof_mat.at(i, j) } else { 0 };
    #[allow(clippy::too_many_arguments)]
    fn split(i0: i32, j0: i32, s: i32, area: &Bounds, tol: f32, leaves: &mut Vec<Leaf>, leaf_of: &mut Grid<u32>, skip: &dyn Fn(i32, i32) -> bool, h: &dyn Fn(i32, i32) -> f32, m: &dyn Fn(i32, i32) -> u8) {
        if i0 + s > area.i1 || j0 + s > area.j1 || i0 < area.i0 || j0 < area.j0 {
            if s > 1 {
                let hs = s / 2;
                for (di, dj) in [(0, 0), (hs, 0), (0, hs), (hs, hs)] {
                    split(i0 + di, j0 + dj, hs, area, tol, leaves, leaf_of, skip, h, m);
                }
            }
            return;
        }
        let mut ok = true;
        let (mut a, mut bi, mut bj) = (0f32, 0f32, 0f32);
        if s == 1 {
            if skip(i0, j0) {
                return;
            }
            a = h(i0, j0);
            // A single column still follows the slope it stands on: the smaller of the slopes to its
            // two neighbours (zero at a kerb or wall, where one neighbour is a step away).
            let minmod = |p: f32, q: f32| if p * q <= 0.0 { 0.0 } else if p.abs() < q.abs() { p } else { q };
            let nb = |i: i32, j: i32| if area.has(i, j) && !skip(i, j) { Some(h(i, j)) } else { None };
            if let (Some(l), Some(r)) = (nb(i0 - 1, j0), nb(i0 + 1, j0)) {
                bi = minmod(a - l, r - a);
            }
            if let (Some(u), Some(d)) = (nb(i0, j0 - 1), nb(i0, j0 + 1)) {
                bj = minmod(a - u, d - a);
            }
        } else {
            let mut any = false;
            let mat = m(i0, j0);
            if skip(i0, j0) || skip(i0 + s - 1, j0) || skip(i0, j0 + s - 1) {
                ok = false;
            } else {
                a = h(i0, j0);
                bi = (h(i0 + s - 1, j0) - a) / (s - 1) as f32;
                bj = (h(i0, j0 + s - 1) - a) / (s - 1) as f32;
            }
            'c: for j in j0..j0 + s {
                for i in i0..i0 + s {
                    let sk = skip(i, j);
                    any |= !sk;
                    if ok && (sk || m(i, j) != mat || (h(i, j) - (a + bi * (i - i0) as f32 + bj * (j - j0) as f32)).abs() > tol) {
                        ok = false;
                    }
                    if !ok && any {
                        break 'c;
                    }
                }
            }
            if !any {
                return;
            }
        }
        if ok {
            let id = leaves.len() as u32;
            leaves.push(Leaf { a, bi, bj, i0, j0, s, mat: m(i0, j0) });
            for j in j0..j0 + s {
                for i in i0..i0 + s {
                    leaf_of.set(i, j, id);
                }
            }
        } else {
            let hs = s / 2;
            for (di, dj) in [(0, 0), (hs, 0), (0, hs), (hs, hs)] {
                split(i0 + di, j0 + dj, hs, area, tol, leaves, leaf_of, skip, h, m);
            }
        }
    }
    let s0 = 32;
    let mut j = area.j0.div_euclid(s0) * s0;
    while j < area.j1 {
        let mut i = area.i0.div_euclid(s0) * s0;
        while i < area.i1 {
            split(i, j, s0, &area, tol, &mut leaves, &mut leaf_of, &skip, &h, &m);
            i += s0;
        }
        j += s0;
    }

    let base = w.base_height as f32;
    let c = CELL as f32;
    let stone = w.palette[w.mat("stone_wall") as usize];
    let mut out: BTreeMap<(i32, i32), MeshBuf> = BTreeMap::new();
    // height of a leaf's surface at a cell-corner position (in cell index space, corner = index - 0.5)
    let at = |l: &Leaf, ci: f32, cj: f32| l.a + l.bi * (ci - l.i0 as f32) + l.bj * (cj - l.j0 as f32) - base;
    for l in &leaves {
        let buf = out.entry((l.i0.div_euclid(TILE), l.j0.div_euclid(TILE))).or_default();
        let (x0, x1) = (l.i0 as f32 * c, (l.i0 + l.s) as f32 * c);
        let (z0, z1) = (l.j0 as f32 * c, (l.j0 + l.s) as f32 * c);
        let (f0, f1) = (-0.5f32, l.s as f32 - 0.5);
        let (i0, j0) = (l.i0 as f32, l.j0 as f32);
        let col = w.palette[l.mat as usize];
        buf.quad_a(
            [
                [x0, at(l, i0 + f0, j0 + f0), z0],
                [x0, at(l, i0 + f0, j0 + f1), z1],
                [x1, at(l, i0 + f1, j0 + f1), z1],
                [x1, at(l, i0 + f1, j0 + f0), z0],
            ],
            [0.0, 1.0, 0.0],
            col,
            if roofs { 255 } else { 0 },
        );
    }
    if std::env::var("VOXSTAT").is_ok() {
        let mut hist = std::collections::BTreeMap::new();
        for l in &leaves {
            *hist.entry(l.s).or_insert(0usize) += 1;
        }
        let tops: usize = out.values().map(|b| b.idx.len() / 6).sum();
        eprintln!("surface roofs={roofs}: leaves by size {hist:?}, top quads {tops}");
    }
    // Side faces between neighbouring patches, merged along straight runs.
    // dir 0: edges between (i-1, j) and (i, j) running along j. dir 1: between (i, j-1) and (i, j) along i.
    for dir in 0..2 {
        let (n_outer, n_inner) = if dir == 0 { (area.i0 + 1..area.i1, area.j0..area.j1) } else { (area.j0 + 1..area.j1, area.i0..area.i1) };
        for o in n_outer {
            let mut run: Option<(u32, u32, i32)> = None;
            let inner_end = n_inner.end;
            for k in n_inner.clone().chain(std::iter::once(inner_end)) {
                let pair = if k < inner_end {
                    let (ia, ja, ib, jb) = if dir == 0 { (o - 1, k, o, k) } else { (k, o - 1, k, o) };
                    let (la, lb) = (*leaf_of.at(ia, ja), *leaf_of.at(ib, jb));
                    // the ground has no edge of its own; a roof ends where the next cell has none
                    if la != lb && (roofs || (la != u32::MAX && lb != u32::MAX)) {
                        Some((la, lb))
                    } else {
                        None
                    }
                } else {
                    None
                };
                let same = match (run, pair) {
                    (Some((a, b, _)), Some((pa, pb))) => a == pa && b == pb,
                    _ => false,
                };
                if same {
                    continue;
                }
                if let Some((la, lb, k0)) = run.take() {
                    let e = o as f32 - 0.5; // shared edge position in index space
                    let (s0, s1) = (k0 as f32 - 0.5, k as f32 - 0.5);
                    let hgt = |id: u32, other: u32, s: f32| -> (f32, u8) {
                        if id != u32::MAX {
                            let l = &leaves[id as usize];
                            (if dir == 0 { at(l, e, s) } else { at(l, s, e) }, l.mat)
                        } else {
                            // roof edge: the side face goes down to the top of the wall voxels below it
                            let l = &leaves[other as usize];
                            let y = if dir == 0 { at(l, e, s) } else { at(l, s, e) };
                            ((((y + base) as f64 / ROW).floor() * ROW) as f32 - base - 0.1, l.mat)
                        }
                    };
                    let ((ya0, ma), (ya1, _), (yb0, mb), (yb1, _)) = (hgt(la, lb, s0), hgt(la, lb, s1), hgt(lb, la, s0), hgt(lb, la, s1));
                    if (ya0 - yb0).abs() > CRACK || (ya1 - yb1).abs() > CRACK {
                        let a_higher = ya0 + ya1 > yb0 + yb1;
                        let big = !roofs && ((ya0 - yb0).abs() > 0.3 || (ya1 - yb1).abs() > 0.3);
                        // a tall step in the ground is a retaining wall
                        let col = if big { stone } else { w.palette[if a_higher { ma } else { mb } as usize] };
                        // 255 = own colour; 128 = a real step in the ground (kerb, bank); 64 = a sliver that only
                        // closes the surface and must look exactly like the ground around it
                        let sliver = (ya0 - yb0).abs() < SLIVER && (ya1 - yb1).abs() < SLIVER;
                        let alpha = if big { 255 } else if roofs { if sliver { 200 } else { 255 } } else if sliver { 64 } else { 128 };
                        let ec = o as f32 * c;
                        let (c0, c1) = (k0 as f32 * c, k as f32 * c);
                        let sgn = if a_higher { 1.0 } else { -1.0 };
                        // counter-clockwise seen from the lower side
                        let (p, n) = if dir == 0 {
                            ([[ec, ya0, c0], [ec, ya1, c1], [ec, yb1, c1], [ec, yb0, c0]], [sgn, 0.0, 0.0])
                        } else {
                            ([[c0, ya0, ec], [c0, yb0, ec], [c1, yb1, ec], [c1, ya1, ec]], [0.0, 0.0, sgn])
                        };
                        let p = if a_higher { p } else { [p[0], p[3], p[2], p[1]] };
                        let (ti, tj) = if dir == 0 { (o.div_euclid(TILE), k0.div_euclid(TILE)) } else { (k0.div_euclid(TILE), o.div_euclid(TILE)) };
                        out.entry((ti, tj)).or_default().quad_a(p, n, col, alpha);
                    }
                }
                if let Some((a, b)) = pair {
                    run = Some((a, b, k));
                }
            }
        }
    }
    if std::env::var("VOXSTAT").is_ok() {
        let all: usize = out.values().map(|b| b.idx.len() / 6).sum();
        eprintln!("surface roofs={roofs}: quads with side faces {all}");
    }
    out
}

/// Voxel mesh, greedy merged. `collision` = true gives a plain solid mesh
/// (materials ignored, recesses filled) for walking against.
pub fn voxel_mesh(w: &World, collision: bool) -> BTreeMap<(i32, i32), MeshBuf> {
    let mut out: BTreeMap<(i32, i32), MeshBuf> = BTreeMap::new();
    let base_row = (w.base_height / ROW) as f32;
    let n = CH as usize;
    let mut keys: Vec<_> = w.chunks.keys().copied().collect();
    keys.sort();
    let solid = |m: u8| -> u8 {
        if collision {
            (m != 0) as u8
        } else if m == M_SOLID_AIR {
            0
        } else {
            m
        }
    };
    let empty: Vec<u8> = vec![0u8; n * n * n];
    let s = ROW as f32;
    for key in keys {
        let ch = &w.chunks[&key];
        let nb = |dx: i32, dy: i32, dz: i32| -> &[u8] { w.chunks.get(&(key.0 + dx, key.1 + dy, key.2 + dz)).map(|c| &c[..]).unwrap_or(&empty[..]) };
        let nbs = [nb(-1, 0, 0), nb(1, 0, 0), nb(0, -1, 0), nb(0, 1, 0), nb(0, 0, -1), nb(0, 0, 1)];
        let (bx, by, bz) = (key.0 * CH, key.1 * CH, key.2 * CH);
        // world position of a voxel index that may be one step outside this chunk
        let hidden = |x: i32, y: i32, z: i32| -> bool { w.inside_building(bx + x, by + y, bz + z) };
        let get = |c: &[u8], x: usize, y: usize, z: usize| -> u8 {
            let m = solid(c[(y * n + z) * n + x]);
            m
        };
        let cap = |x: usize, y: usize, z: usize| -> bool {
            // collision only: building columns stop at the eave (nobody walks on roofs)
            collision && w.collision_top(bx + x as i32, bz + z as i32).map(|t| by + y as i32 >= t).unwrap_or(false)
        };
        let buf = out.entry(((key.0 * CH).div_euclid(TILE), (key.2 * CH).div_euclid(TILE))).or_default();
        let org = [(key.0 * CH) as f32, (key.1 * CH) as f32 - base_row, (key.2 * CH) as f32];
        // six directions: axis d, sign
        for face in 0..6 {
            let d = face / 2;
            let pos = face % 2 == 1;
            let (u, v) = ((d + 1) % 3, (d + 2) % 3);
            let mut mask = vec![0u16; n * n];
            for sl in 0..n {
                let mut any = false;
                for b in 0..n {
                    for a in 0..n {
                        let mut p = [0usize; 3];
                        p[d] = sl;
                        p[u] = a;
                        p[v] = b;
                        let m = get(ch, p[0], p[1], p[2]);
                        let mut vis = 0u16;
                        let m = m as u16 | if collision { 0 } else { (*w.wall_dir.at((bx + p[0] as i32).clamp(w.bounds.i0, w.bounds.i1 - 1), (bz + p[2] as i32).clamp(w.bounds.j0, w.bounds.j1 - 1)) as u16) << 8 };
                        if m & 255 != 0 && !cap(p[0], p[1], p[2]) {
                            let edge = if pos { sl == n - 1 } else { sl == 0 };
                            let other = if edge {
                                let mut q = p;
                                q[d] = if pos { 0 } else { n - 1 };
                                get(nbs[d * 2 + pos as usize], q[0], q[1], q[2])
                            } else {
                                let mut q = p;
                                q[d] = if pos { sl + 1 } else { sl - 1 };
                                get(ch, q[0], q[1], q[2])
                            };
                            let mut q = [p[0] as i32, p[1] as i32, p[2] as i32];
                            q[d] += if pos { 1 } else { -1 };
                            // a window or door recess is open air you can see into, even though it lies
                            // within the building's body
                            let recess = !collision && w.voxel(bx + q[0], by + q[1], bz + q[2]) == M_SOLID_AIR;
                            if other == 0 && (recess || !hidden(q[0], q[1], q[2])) {
                                vis = m;
                                any = true;
                            } else if collision && other != 0 && pos && d == 1 && cap(p[0], p[1] + 1, p[2]) && p[1] + 1 < n {
                                // top of a capped column
                                vis = m;
                                any = true;
                            }
                        }
                        mask[b * n + a] = vis;
                    }
                }
                if !any {
                    continue;
                }
                for b in 0..n {
                    let mut a = 0;
                    while a < n {
                        let m = mask[b * n + a];
                        if m == 0 {
                            a += 1;
                            continue;
                        }
                        let mut wd = 1;
                        while a + wd < n && mask[b * n + a + wd] == m {
                            wd += 1;
                        }
                        let mut ht = 1;
                        'grow: while b + ht < n {
                            for k in 0..wd {
                                if mask[(b + ht) * n + a + k] != m {
                                    break 'grow;
                                }
                            }
                            ht += 1;
                        }
                        for bb in b..b + ht {
                            for aa in a..a + wd {
                                mask[bb * n + aa] = 0;
                            }
                        }
                        let dpos = (sl + pos as usize) as f32;
                        let corner = |ua: usize, vb: usize| -> [f32; 3] {
                            let mut p = [0f32; 3];
                            p[d] = dpos;
                            p[u] = ua as f32;
                            p[v] = vb as f32;
                            [(p[0] + org[0]) * s, (p[1] + org[1]) * s, (p[2] + org[2]) * s]
                        };
                        let (c00, c10, c11, c01) = (corner(a, b), corner(a + wd, b), corner(a + wd, b + ht), corner(a, b + ht));
                        let mut nrm = [0f32; 3];
                        nrm[d] = if pos { 1.0 } else { -1.0 };
                        let col = if collision { [200, 200, 200] } else { w.palette[(m & 255) as usize] };
                        // side faces of a wall carry the direction the wall faces (alpha 129..254)
                        let alpha = if d != 1 && (m >> 8) != 0 { 128 + (m >> 8) as u8 } else { 255 };
                        // counter-clockwise seen from outside
                        if pos {
                            buf.quad_a([c00, c10, c11, c01], nrm, col, alpha);
                        } else {
                            buf.quad_a([c00, c01, c11, c10], nrm, col, alpha);
                        }
                        a += wd;
                    }
                }
            }
        }
    }
    out
}

/// Solid cores of the buildings, a little inside the walls and under the roofs. They are never
/// seen; they only cast shadows. A building drawn as a thin shell lets sunlight that runs almost
/// along a wall slip past the foot of that wall; a solid core behind the wall stops it.
pub fn shadow_cores(w: &World) -> MeshBuf {
    let b = w.bounds;
    let (bw, bh) = (b.w(), b.h());
    let inset = 2i32;
    // core columns: every cell within `inset` is building too
    let mask: Vec<bool> = w.zone.data.iter().enumerate().map(|(k, z)| *z == Z_BUILDING && w.owner.data[k] >= O_BUILDING0 && w.buildings[(w.owner.data[k] - O_BUILDING0) as usize].owned).collect();
    let mut rowmin = vec![false; bw * bh];
    for j in 0..bh {
        for i in 0..bw {
            rowmin[j * bw + i] = (i as i32 - inset..=i as i32 + inset).all(|x| x >= 0 && (x as usize) < bw && mask[j * bw + x as usize]);
        }
    }
    let mut core = vec![false; bw * bh];
    for j in 0..bh {
        for i in 0..bw {
            core[j * bw + i] = (j as i32 - inset..=j as i32 + inset).all(|y| y >= 0 && (y as usize) < bh && rowmin[y as usize * bw + i]);
        }
    }
    let range = |i: usize, j: usize| -> (i32, i32) {
        if !core[j * bw + i] {
            return (0, 0);
        }
        let bp = &w.buildings[(w.owner.data[j * bw + i] - O_BUILDING0) as usize];
        (bp.base_row, bp.eave_row.max(bp.base_row))
    };
    let base_row = (w.base_height / ROW) as f32;
    let (c, r) = (CELL as f32, ROW as f32);
    let x = |i: usize| (b.i0 + i as i32) as f32 * c;
    let z = |j: usize| (b.j0 + j as i32) as f32 * c;
    let y = |row: i32| (row as f32 - base_row) * r;
    let mut out = MeshBuf::default();
    let grey = [128, 128, 128];
    // one closed box per run of core cells in a row (faces inside the core do no harm: it is never seen)
    for j in 0..bh {
        let mut i = 0;
        while i < bw {
            let (b0, t0) = range(i, j);
            if t0 <= b0 {
                i += 1;
                continue;
            }
            let mut e = i + 1;
            while e < bw && range(e, j) == (b0, t0) {
                e += 1;
            }
            let (x0, x1, z0, z1, y0, y1) = (x(i), x(e), z(j), z(j + 1), y(b0), y(t0));
            let p = [[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0], [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]];
            for q in [[3, 7, 6, 2], [0, 1, 5, 4], [0, 3, 2, 1], [4, 5, 6, 7], [0, 4, 7, 3], [1, 2, 6, 5]] {
                out.quad([p[q[0]], p[q[1]], p[q[2]], p[q[3]]], [0.0, 1.0, 0.0], grey);
            }
            i = e;
        }
    }
    out
}
