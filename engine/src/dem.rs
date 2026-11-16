//! Elevation. Reads one Copernicus GLO-30 tile and gives a smooth ground
//! height for any point. The tile is a surface model (roofs and tree tops are
//! in it), so the bumps are flattened before use: the data only gives the
//! broad shape of the land.

use crate::geo::{Proj, P2};
use std::fs::File;
use tiff::decoder::{Decoder, DecodingResult, Limits};
use tiff::tags::Tag;

pub struct Dem {
    w: usize,
    h: usize,
    data: Vec<f32>,
    lon0: f64,
    lat0: f64,
    dlon: f64,
    dlat: f64,
    /// Heights are scaled around `base` by this factor (1 = real, 3 = stress test).
    pub exaggeration: f64,
    pub base: f64,
    flat: bool,
}

impl Dem {
    /// A world with no elevation data at all: flat at height 0. Used as the fallback.
    pub fn flat() -> Dem {
        Dem { w: 1, h: 1, data: vec![0.0], lon0: 0.0, lat0: 0.0, dlon: 1.0, dlat: 1.0, exaggeration: 1.0, base: 0.0, flat: true }
    }

    pub fn tile_name(lat: f64, lon: f64) -> String {
        let (la, lo) = (lat.floor() as i32, lon.floor() as i32);
        format!(
            "{}{:02}_00_{}{:03}_00.tif",
            if la >= 0 { 'N' } else { 'S' },
            la.abs(),
            if lo >= 0 { 'E' } else { 'W' },
            lo.abs()
        )
    }

    pub fn load(path: &str) -> Result<Dem, String> {
        let f = File::open(path).map_err(|e| format!("cannot open {path}: {e}"))?;
        let mut dec = Decoder::new(f).map_err(|e| format!("{path}: {e}"))?.with_limits(Limits::unlimited());
        let (w, h) = dec.dimensions().map_err(|e| format!("{path}: {e}"))?;
        let scale = dec.get_tag_f64_vec(Tag::ModelPixelScaleTag).map_err(|e| format!("{path}: no pixel scale: {e}"))?;
        let tie = dec.get_tag_f64_vec(Tag::ModelTiepointTag).map_err(|e| format!("{path}: no tie point: {e}"))?;
        if scale.len() < 2 || tie.len() < 6 {
            return Err(format!("{path}: bad georeference"));
        }
        let img = dec.read_image().map_err(|e| format!("{path}: {e}"))?;
        let mut data: Vec<f32> = match img {
            DecodingResult::F32(v) => v,
            DecodingResult::F64(v) => v.into_iter().map(|x| x as f32).collect(),
            DecodingResult::I16(v) => v.into_iter().map(|x| x as f32).collect(),
            DecodingResult::U16(v) => v.into_iter().map(|x| x as f32).collect(),
            _ => return Err(format!("{path}: unsupported sample type")),
        };
        let (w, h) = (w as usize, h as usize);
        if data.len() != w * h {
            return Err(format!("{path}: size mismatch"));
        }
        for v in data.iter_mut() {
            if !v.is_finite() || *v < -1000.0 || *v > 9000.0 {
                *v = 0.0;
            }
        }
        // Tie point is the upper-left corner of pixel (tie[0], tie[1]); we store pixel centres.
        let lon0 = tie[3] + (0.5 - tie[0]) * scale[0];
        let lat0 = tie[4] - (0.5 - tie[1]) * scale[1];
        let mut d = Dem { w, h, data, lon0, lat0, dlon: scale[0], dlat: scale[1], exaggeration: 1.0, base: 0.0, flat: false };
        d.flatten_bumps();
        Ok(d)
    }

    /// Lower envelope (3x3 minimum) followed by a 3x3 blur, twice. Roofs and
    /// tree tops stick up above the ground, so taking the local minimum
    /// removes them; the blur removes the blockiness that leaves behind.
    fn flatten_bumps(&mut self) {
        let (w, h) = (self.w, self.h);
        let get = |d: &Vec<f32>, x: i64, y: i64| d[(y.clamp(0, h as i64 - 1) as usize) * w + x.clamp(0, w as i64 - 1) as usize];
        let mut mn = vec![0f32; w * h];
        for y in 0..h as i64 {
            for x in 0..w as i64 {
                let mut m = f32::MAX;
                for dy in -1..=1 {
                    for dx in -1..=1 {
                        m = m.min(get(&self.data, x + dx, y + dy));
                    }
                }
                mn[y as usize * w + x as usize] = m;
            }
        }
        for _ in 0..2 {
            let mut bl = vec![0f32; w * h];
            for y in 0..h as i64 {
                for x in 0..w as i64 {
                    let mut s = 0f32;
                    for dy in -1..=1 {
                        for dx in -1..=1 {
                            s += get(&mn, x + dx, y + dy);
                        }
                    }
                    bl[y as usize * w + x as usize] = s / 9.0;
                }
            }
            mn = bl;
        }
        self.data = mn;
    }

    fn px(&self, x: i64, y: i64) -> f64 {
        self.data[(y.clamp(0, self.h as i64 - 1) as usize) * self.w + x.clamp(0, self.w as i64 - 1) as usize] as f64
    }

    /// Smooth (cubic B-spline) height in metres at a lat/lon. Outside the tile the edge is repeated.
    pub fn height_latlon(&self, lat: f64, lon: f64) -> f64 {
        if self.flat {
            return 0.0;
        }
        let fx = (lon - self.lon0) / self.dlon;
        let fy = (self.lat0 - lat) / self.dlat;
        let (ix, iy) = (fx.floor(), fy.floor());
        let (tx, ty) = (fx - ix, fy - iy);
        let wts = |t: f64| {
            let t2 = t * t;
            let t3 = t2 * t;
            [(1.0 - 3.0 * t + 3.0 * t2 - t3) / 6.0, (4.0 - 6.0 * t2 + 3.0 * t3) / 6.0, (1.0 + 3.0 * t + 3.0 * t2 - 3.0 * t3) / 6.0, t3 / 6.0]
        };
        let (wx, wy) = (wts(tx), wts(ty));
        let mut s = 0.0;
        for j in 0..4 {
            for i in 0..4 {
                s += wx[i] * wy[j] * self.px(ix as i64 - 1 + i as i64, iy as i64 - 1 + j as i64);
            }
        }
        self.base + (s - self.base) * self.exaggeration
    }

    pub fn height(&self, proj: &Proj, p: P2) -> f64 {
        let (lat, lon) = proj.to_latlon(p);
        self.height_latlon(lat, lon)
    }
}
