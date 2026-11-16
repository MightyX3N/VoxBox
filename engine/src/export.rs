//! Writes a built place to disk for a game to load: meshes as .glb files
//! (ground, voxels, collision) and a place.json with everything else.

use crate::geo::P2;
use crate::mesh::{surface_mesh, voxel_mesh, MeshBuf};
use crate::world::*;
use serde_json::{json, Value};
use std::collections::BTreeMap;

/// Mesh file: "VXB1", mesh count, then per mesh: name length, name, vertex count, index count,
/// positions (3 x f32), colours (2 x f32: r + 256 g, b + 256 a), indices (u32). All little endian.
/// With `soup` the mesh is written as plain triangles (9 x f32 each) for collision.
fn write_meshes(path: &str, meshes: &[(String, &MeshBuf)], soup: bool) -> Result<(usize, usize), String> {
    let mut out: Vec<u8> = Vec::new();
    out.extend_from_slice(if soup { b"VXC1" } else { b"VXB1" });
    let live: Vec<&(String, &MeshBuf)> = meshes.iter().filter(|(_, m)| !m.idx.is_empty()).collect();
    out.extend_from_slice(&(live.len() as u32).to_le_bytes());
    let mut tris = 0;
    for (name, m) in live {
        tris += m.tris();
        out.extend_from_slice(&(name.len() as u32).to_le_bytes());
        out.extend_from_slice(name.as_bytes());
        if soup {
            out.extend_from_slice(&(m.idx.len() as u32).to_le_bytes());
            for k in &m.idx {
                for c in 0..3 {
                    out.extend_from_slice(&m.pos[*k as usize * 3 + c].to_le_bytes());
                }
            }
        } else {
            out.extend_from_slice(&((m.pos.len() / 3) as u32).to_le_bytes());
            out.extend_from_slice(&(m.idx.len() as u32).to_le_bytes());
            for v in &m.pos {
                out.extend_from_slice(&v.to_le_bytes());
            }
            // colour packed into two floats per vertex (r + 256 g, b + 256 a): read by every renderer
            for c in m.col.chunks(4) {
                out.extend_from_slice(&(c[0] as f32 + 256.0 * c[1] as f32).to_le_bytes());
                out.extend_from_slice(&(c[2] as f32 + 256.0 * c[3] as f32).to_le_bytes());
            }
            for v in &m.idx {
                out.extend_from_slice(&v.to_le_bytes());
            }
        }
    }
    std::fs::write(path, &out).map_err(|e| format!("{path}: {e}"))?;
    Ok((tris, out.len()))
}

pub const CREDITS: &str = "Map data (c) OpenStreetMap contributors, available under the Open Database License (openstreetmap.org/copyright).\nTerrain: produced using Copernicus WorldDEM-30 (c) DLR e.V. 2010-2014 and (c) Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights reserved. The organisations in charge of the Copernicus programme by law or by delegation do not incur any liability for any use of the Copernicus WorldDEM-30.";

pub fn export(w: &World, dir: &str) -> Result<String, String> {
    std::fs::create_dir_all(dir).map_err(|e| format!("{dir}: {e}"))?;
    let g = surface_mesh(w, false);
    let r = surface_mesh(w, true);
    let v = voxel_mesh(w, false);
    let c = voxel_mesh(w, true);
    fn named<'a>(m: &'a BTreeMap<(i32, i32), MeshBuf>, p: &str) -> Vec<(String, &'a MeshBuf)> {
        m.iter().map(|(k, b)| (format!("{p}_{}_{}", k.0, k.1), b)).collect()
    }
    let mut surf = named(&g, "ground");
    surf.extend(named(&r, "roof"));
    let (gt, gs) = write_meshes(&format!("{dir}/surface.vxb"), &surf, false)?;
    let (vt, vs) = write_meshes(&format!("{dir}/voxels.vxb"), &named(&v, "vox"), false)?;
    let (ct, cs) = write_meshes(&format!("{dir}/collision.vxc"), &named(&c, "col"), true)?;
    let cores = crate::mesh::shadow_cores(w);
    write_meshes(&format!("{dir}/shadow.vxc"), &[("cores".to_string(), &cores)], true)?;

    // ground colour map: one pixel per 10 cm cell of the core
    // Cells under buildings take the colour of the ground beside them, so the ground keeps its own
    // colour right up to (and just under) every wall.
    let mut mat = w.mat.clone();
    let mut known: Vec<bool> = w.zone.data.iter().map(|z| *z != Z_BUILDING).collect();
    for _ in 0..4 {
        let prev = known.clone();
        for j in w.core.j0..w.core.j1 {
            for i in w.core.i0..w.core.i1 {
                let k = mat.idx(i, j);
                if prev[k] {
                    continue;
                }
                for (di, dj) in [(1, 0), (-1, 0), (0, 1), (0, -1)] {
                    if w.bounds.has(i + di, j + dj) && prev[mat.idx(i + di, j + dj)] {
                        mat.data[k] = mat.data[mat.idx(i + di, j + dj)];
                        known[k] = true;
                        break;
                    }
                }
            }
        }
    }
    let mut rgb: Vec<u8> = Vec::with_capacity(w.core.w() * w.core.h() * 3);
    for j in w.core.j0..w.core.j1 {
        for i in w.core.i0..w.core.i1 {
            rgb.extend_from_slice(&w.palette[*mat.at(i, j) as usize]);
        }
    }
    std::fs::write(format!("{dir}/ground.rgb"), &rgb).map_err(|e| e.to_string())?;

    // walking surface: ground heights every 0.2 m
    let core = w.core;
    let step = 0.2;
    let (nx, nz) = ((core.w() as f64 * 0.1 / step).round() as usize + 1, (core.h() as f64 * 0.1 / step).round() as usize + 1);
    let mut hm: Vec<u8> = Vec::with_capacity(nx * nz * 4);
    for kz in 0..nz {
        for kx in 0..nx {
            let p = P2::new(core.min().x + kx as f64 * step, core.min().z + kz as f64 * step);
            hm.extend_from_slice(&((w.ground_height(p) - w.base_height) as f32).to_le_bytes());
        }
    }
    std::fs::write(format!("{dir}/heights.bin"), &hm).map_err(|e| e.to_string())?;

    let (cmin, cmax) = (core.min(), core.max());
    let inside = |x: f64, z: f64| x >= cmin.x && x < cmax.x && z >= cmin.z && z < cmax.z;
    let mut lines = Vec::new();
    for t in &w.travel {
        let mut run: Vec<Value> = Vec::new();
        let flush = |run: &mut Vec<Value>, lines: &mut Vec<Value>| {
            if run.len() >= 2 {
                lines.push(json!({"way": t.way_id, "kind": t.kind, "class": t.class, "points": run.clone()}));
            }
            run.clear();
        };
        for (k, p) in t.pts.iter().enumerate() {
            if inside(p[0], p[2]) {
                let r = |x: f64| (x * 1000.0).round() / 1000.0;
                run.push(json!([r(p[0]), r(p[1] - w.base_height), r(p[2]), r(t.half_width[k])]));
            } else {
                flush(&mut run, &mut lines);
            }
        }
        flush(&mut run, &mut lines);
    }
    let mut models = serde_json::Map::new();
    for (name, rule) in &w.rules.props {
        let boxes: Vec<Value> = rule.boxes.iter().map(|b| json!({"min": b.min, "max": b.max, "material": b.material, "colour": w.rules.materials.get(&b.material)})).collect();
        models.insert(name.clone(), json!({"boxes": boxes, "foot": rule.foot}));
    }
    let props: Vec<Value> = w
        .props
        .iter()
        .map(|p| json!({"model": p.model, "pos": [p.pos.x, p.y - w.base_height, p.pos.z], "yaw": p.yaw, "slope": [p.slope.0, p.slope.1], "scale": p.scale, "body": w.palette[p.body as usize], "on_lane": p.on_lane}))
        .collect();
    let doors: Vec<Value> = w
        .buildings
        .iter()
        .filter(|b| b.owned)
        .flat_map(|b| b.openings.iter().filter(|o| o.is_door).map(move |o| (b, o)))
        .map(|(b, o)| json!({"building": b.id, "pos": [o.centre.x, o.r0 as f64 * ROW - w.base_height, o.centre.z], "out": [o.out.x, o.out.z]}))
        .collect();
    // spawn on the travel line point nearest to the middle of the place
    let mid = P2::new((cmin.x + cmax.x) / 2.0, (cmin.z + cmax.z) / 2.0);
    let mut spawn = [mid.x, w.ground_height(mid) - w.base_height, mid.z];
    let mut best = f64::MAX;
    for t in &w.travel {
        for p in &t.pts {
            let d = P2::new(p[0], p[2]).dist(mid);
            if d < best && inside(p[0], p[2]) {
                best = d;
                spawn = [p[0], p[1] - w.base_height, p[2]];
            }
        }
    }
    let nwin: usize = w.buildings.iter().map(|b| b.openings.iter().filter(|o| !o.is_door).count()).sum();
    let doc = json!({
        "name": w.name, "lat": w.proj.lat0, "lon": w.proj.lon0, "base_height": w.base_height,
        "min": [cmin.x, cmin.z], "max": [cmax.x, cmax.z],
        "heights": {"file": "heights.bin", "nx": nx, "nz": nz, "step": step},
        "ground_colours": {"file": "ground.rgb", "width": w.core.w(), "height": w.core.h()},
        "spawn": spawn, "travel_lines": lines,
        "barriers": w.barriers.iter().map(|b| json!({"id": b.id, "kind": b.kind, "pos": [b.pos.x, b.pos.z]})).collect::<Vec<_>>(),
        "models": models, "props": props, "doors": doors, "credits": CREDITS,
        "stats": {"buildings": w.buildings.iter().filter(|b| b.owned).count(), "windows": nwin, "doors": doors.len(), "props": w.props.len(),
                  "surface_triangles": gt, "voxel_triangles": vt, "collision_triangles": ct, "bytes": gs + vs + cs}
    });
    std::fs::write(format!("{dir}/place.json"), serde_json::to_vec(&doc).map_err(|e| e.to_string())?).map_err(|e| e.to_string())?;
    Ok(format!("ground+roofs {gt} tris, voxels {vt} tris, collision {ct} tris, {:.1} MB", (gs + vs + cs) as f64 / 1e6))
}
