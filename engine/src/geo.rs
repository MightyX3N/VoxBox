//! Small geometry helpers. Everything is in metres in the local frame:
//! x = east, z = south, y = up. One cell is CELL metres.

pub const CELL: f64 = 0.1;

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct P2 {
    pub x: f64,
    pub z: f64,
}

impl P2 {
    pub fn new(x: f64, z: f64) -> Self {
        P2 { x, z }
    }
    pub fn sub(self, o: P2) -> P2 {
        P2::new(self.x - o.x, self.z - o.z)
    }
    pub fn add(self, o: P2) -> P2 {
        P2::new(self.x + o.x, self.z + o.z)
    }
    pub fn mul(self, s: f64) -> P2 {
        P2::new(self.x * s, self.z * s)
    }
    pub fn dot(self, o: P2) -> f64 {
        self.x * o.x + self.z * o.z
    }
    pub fn cross(self, o: P2) -> f64 {
        self.x * o.z - self.z * o.x
    }
    pub fn len(self) -> f64 {
        (self.x * self.x + self.z * self.z).sqrt()
    }
    pub fn norm(self) -> P2 {
        let l = self.len();
        if l < 1e-12 {
            P2::new(1.0, 0.0)
        } else {
            self.mul(1.0 / l)
        }
    }
    pub fn dist(self, o: P2) -> f64 {
        self.sub(o).len()
    }
    pub fn lerp(self, o: P2, t: f64) -> P2 {
        P2::new(self.x + (o.x - self.x) * t, self.z + (o.z - self.z) * t)
    }
}

/// Local projection around a fixed origin. The origin is part of the world
/// definition, so the same origin always gives the same grid.
#[derive(Clone, Copy, Debug)]
pub struct Proj {
    pub lat0: f64,
    pub lon0: f64,
    mx: f64,
    mz: f64,
}

impl Proj {
    pub fn new(lat0: f64, lon0: f64) -> Self {
        let r = 6_378_137.0_f64;
        let mz = r.to_radians();
        let mx = mz * lat0.to_radians().cos();
        Proj { lat0, lon0, mx, mz }
    }
    pub fn to_local(&self, lat: f64, lon: f64) -> P2 {
        P2::new((lon - self.lon0) * self.mx, -(lat - self.lat0) * self.mz)
    }
    pub fn to_latlon(&self, p: P2) -> (f64, f64) {
        (self.lat0 - p.z / self.mz, self.lon0 + p.x / self.mx)
    }
}

/// Closest point on segment a-b: returns (t in 0..1, distance, signed side).
/// side > 0 means the point is to the right of a->b (in x-east, z-south frame).
pub fn seg_project(p: P2, a: P2, b: P2) -> (f64, f64, f64) {
    let ab = b.sub(a);
    let l2 = ab.dot(ab);
    let t = if l2 < 1e-12 { 0.0 } else { (p.sub(a).dot(ab) / l2).clamp(0.0, 1.0) };
    let q = a.lerp(b, t);
    let d = p.dist(q);
    let side = ab.cross(p.sub(a));
    (t, d, side)
}

pub fn ring_area(r: &[P2]) -> f64 {
    let mut a = 0.0;
    for i in 0..r.len() {
        let j = (i + 1) % r.len();
        a += r[i].cross(r[j]);
    }
    a * 0.5
}

pub fn ring_centroid(r: &[P2]) -> P2 {
    let mut c = P2::new(0.0, 0.0);
    let mut a = 0.0;
    for i in 0..r.len() {
        let j = (i + 1) % r.len();
        let k = r[i].cross(r[j]);
        a += k;
        c.x += (r[i].x + r[j].x) * k;
        c.z += (r[i].z + r[j].z) * k;
    }
    if a.abs() < 1e-9 {
        let n = r.len().max(1) as f64;
        let mut s = P2::new(0.0, 0.0);
        for p in r {
            s = s.add(*p);
        }
        return s.mul(1.0 / n);
    }
    c.mul(1.0 / (3.0 * a))
}

/// Even-odd point in polygon over several rings (outer and inner).
pub fn point_in_rings(p: P2, rings: &[Vec<P2>]) -> bool {
    let mut inside = false;
    for r in rings {
        let n = r.len();
        if n < 3 {
            continue;
        }
        let mut j = n - 1;
        for i in 0..n {
            let (a, b) = (r[i], r[j]);
            if (a.z > p.z) != (b.z > p.z) {
                let x = a.x + (p.z - a.z) / (b.z - a.z) * (b.x - a.x);
                if p.x < x {
                    inside = !inside;
                }
            }
            j = i;
        }
    }
    inside
}

/// Scanline fill: calls f(i, j) for every cell whose centre is inside the rings.
/// Cell (i, j) has its centre at ((i + 0.5) * CELL, (j + 0.5) * CELL).
pub fn fill_rings<F: FnMut(i32, i32)>(rings: &[Vec<P2>], clip: Option<(i32, i32, i32, i32)>, mut f: F) {
    let mut zmin = f64::MAX;
    let mut zmax = f64::MIN;
    for r in rings {
        for p in r {
            zmin = zmin.min(p.z);
            zmax = zmax.max(p.z);
        }
    }
    if zmin > zmax {
        return;
    }
    let mut j0 = (zmin / CELL - 0.5).ceil() as i32;
    let mut j1 = (zmax / CELL - 0.5).floor() as i32;
    if let Some((_, cj0, _, cj1)) = clip {
        j0 = j0.max(cj0);
        j1 = j1.min(cj1 - 1);
    }
    let mut xs: Vec<f64> = Vec::new();
    for j in j0..=j1 {
        let z = (j as f64 + 0.5) * CELL;
        xs.clear();
        for r in rings {
            let n = r.len();
            if n < 3 {
                continue;
            }
            let mut k = n - 1;
            for i in 0..n {
                let (a, b) = (r[i], r[k]);
                if (a.z > z) != (b.z > z) {
                    xs.push(a.x + (z - a.z) / (b.z - a.z) * (b.x - a.x));
                }
                k = i;
            }
        }
        xs.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut k = 0;
        while k + 1 < xs.len() {
            let mut i0 = (xs[k] / CELL - 0.5).ceil() as i32;
            let mut i1 = (xs[k + 1] / CELL - 0.5).floor() as i32;
            if let Some((ci0, _, ci1, _)) = clip {
                i0 = i0.max(ci0);
                i1 = i1.min(ci1 - 1);
            }
            for i in i0..=i1 {
                f(i, j);
            }
            k += 2;
        }
    }
}

/// Deterministic hash helpers (no global random state anywhere in the engine).
pub fn hash64(mut x: u64) -> u64 {
    x = x.wrapping_add(0x9E3779B97F4A7C15);
    x = (x ^ (x >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
    x = (x ^ (x >> 27)).wrapping_mul(0x94D049BB133111EB);
    x ^ (x >> 31)
}
pub fn hash2(a: u64, b: u64) -> u64 {
    hash64(hash64(a) ^ b.wrapping_mul(0xD6E8FEB86659FD93))
}
/// 0..1 from an id and a salt.
pub fn rand01(id: u64, salt: u64) -> f64 {
    (hash2(id, salt) >> 11) as f64 / (1u64 << 53) as f64
}

pub fn smoothstep(t: f64) -> f64 {
    let t = t.clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// Exact squared Euclidean distance transform (Felzenszwalb) on a w*h grid.
/// Input: true = feature cell. Output: distance in cells to the nearest feature cell.
pub fn edt(mask: &[bool], w: usize, h: usize) -> Vec<f32> {
    let inf = 1e20_f64;
    let mut g: Vec<f64> = mask.iter().map(|&m| if m { 0.0 } else { inf }).collect();
    let n = w.max(h);
    let mut f = vec![0.0f64; n];
    let mut d = vec![0.0f64; n];
    let mut v = vec![0usize; n];
    let mut zz = vec![0.0f64; n + 1];
    let mut pass = |f: &[f64], d: &mut [f64], len: usize, v: &mut [usize], zz: &mut [f64]| {
        let mut k = 0usize;
        v[0] = 0;
        zz[0] = -inf;
        zz[1] = inf;
        for q in 1..len {
            loop {
                let p = v[k];
                let s = ((f[q] + (q * q) as f64) - (f[p] + (p * p) as f64)) / (2.0 * q as f64 - 2.0 * p as f64);
                if s <= zz[k] && k > 0 {
                    k -= 1;
                } else {
                    k += 1;
                    v[k] = q;
                    zz[k] = s;
                    zz[k + 1] = inf;
                    break;
                }
            }
        }
        k = 0;
        for q in 0..len {
            while zz[k + 1] < q as f64 {
                k += 1;
            }
            let p = v[k];
            let dq = q as f64 - p as f64;
            d[q] = dq * dq + f[p];
        }
    };
    for x in 0..w {
        for y in 0..h {
            f[y] = g[y * w + x];
        }
        pass(&f, &mut d, h, &mut v, &mut zz);
        for y in 0..h {
            g[y * w + x] = d[y];
        }
    }
    for y in 0..h {
        f[..w].copy_from_slice(&g[y * w..y * w + w]);
        pass(&f, &mut d, w, &mut v, &mut zz);
        g[y * w..y * w + w].copy_from_slice(&d[..w]);
    }
    g.iter().map(|&x| (x.min(1e12)).sqrt() as f32).collect()
}
