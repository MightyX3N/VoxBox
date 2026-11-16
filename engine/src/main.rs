use voxbox::build::{build, load_map, PlaceSpec};
use voxbox::checks;
use voxbox::dem::Dem;
use voxbox::world::Rules;

fn usage() -> ! {
    eprintln!("voxbox build <name> <lat> <lon> <size_m> <map.osm.pbf> <dem_dir> <out_dir> [--check] [--x3] [--rules file]");
    std::process::exit(2)
}

fn main() {
    let a: Vec<String> = std::env::args().collect();
    if a.len() < 9 || a[1] != "build" {
        usage();
    }
    let flag = |f: &str| a.iter().any(|x| x == f);
    let rules = match a.iter().position(|x| x == "--rules") {
        Some(k) => Rules::from_file(&a[k + 1]).unwrap_or_else(|e| {
            eprintln!("{e}");
            std::process::exit(1)
        }),
        None => Rules::default_rules(),
    };
    let (lat, lon, size): (f64, f64, f64) = (a[3].parse().unwrap_or_else(|_| usage()), a[4].parse().unwrap_or_else(|_| usage()), a[5].parse().unwrap_or_else(|_| usage()));
    let mut spec = PlaceSpec::centred(&a[2], lat, lon, size);
    let t0 = std::time::Instant::now();
    let map = load_map(&a[6], &spec).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1)
    });
    let dem_path = format!("{}/{}", a[7], Dem::tile_name(lat, lon));
    let mut dem = Dem::load(&dem_path).unwrap_or_else(|e| {
        eprintln!("no elevation ({e}); building on flat ground");
        Dem::flat()
    });
    eprintln!("[{}] data loaded in {:.1}s: {} lines, {} areas, {} points", spec.name, t0.elapsed().as_secs_f64(), map.lines.len(), map.areas.len(), map.points.len());
    let mut all_ok = true;
    let mut report = |title: &str, cs: &[checks::Check], all_ok: &mut bool| {
        println!("  {title}");
        for c in cs {
            println!("    [{}] {} ({} tested{})", if c.pass { "PASS" } else { "FAIL" }, c.name, c.tested, if c.pass { String::new() } else { format!(", {} failed: {}", c.failed, c.detail) });
            *all_ok &= c.pass;
        }
    };
    if std::env::var("VOXX3").is_ok() {
        spec.exaggeration = 3.0;
    }
    let t = std::time::Instant::now();
    let w = build(&spec, &rules, &map, &mut dem);
    eprintln!("[{}] built in {:.1}s", spec.name, t.elapsed().as_secs_f64());
    println!("{}: {} buildings, {} windows, {} doors, {} props, {} marked barriers", spec.name,
        w.buildings.iter().filter(|b| b.owned).count(),
        w.buildings.iter().map(|b| b.openings.iter().filter(|o| !o.is_door).count()).sum::<usize>(),
        w.buildings.iter().map(|b| b.openings.iter().filter(|o| o.is_door).count()).sum::<usize>(), w.props.len(), w.barriers.len());
    println!("  skipped: {:?}", w.stats.skipped);
    println!("  openings dropped in the plan: {:?}", w.stats.dropped_openings);
    println!("  buildings without a possible door: {}", w.buildings.iter().filter(|b| b.owned && !b.has_door).count());
    if let Ok(d) = std::env::var("VOXWAY") {
        let id: i64 = d.parse().unwrap_or(0);
        for t in w.travel.iter().filter(|t| t.way_id == id) {
            for (k, p) in t.pts.iter().enumerate() {
                let (i, j) = voxbox::world::cell_of(voxbox::geo::P2::new(p[0], p[2]));
                if w.core.has(i, j) {
                    eprintln!("way pt {k} ({:.1},{:.1}) y {:.3} terrain {:.3} hw {:.2} zone {}", p[0], p[2], p[1], w.terrain.at(i, j), t.half_width[k], w.zone.at(i, j));
                }
            }
        }
    }
    if let Ok(d) = std::env::var("VOXDBG") {
        for cell in d.split(';') {
            let v: Vec<i32> = cell.split(',').filter_map(|x| x.trim().parse().ok()).collect();
            if v.len() == 2 {
                let c = voxbox::world::cell_centre(v[0], v[1]);
                for t in &w.travel {
                    for (k, p) in t.pts.iter().enumerate() {
                        if voxbox::geo::P2::new(p[0], p[2]).dist(c) < 3.0 {
                            eprintln!("  near: {} {} pt {k}/{} ({:.1},{:.1}) y {:.3} hw {:.2}", t.class, t.way_id, t.pts.len(), p[0], p[2], p[1], t.half_width[k]);
                        }
                    }
                }
                for dj in [-6, -3, 0, 3, 6] {
                    let row: Vec<String> = (-12..=12).map(|di| format!("{}{:.1}", ["", "R", "P", "V", "W", "B", "C"][*w.zone.at(v[0] + di, v[1] + dj) as usize], *w.ground.at(v[0] + di, v[1] + dj) as f64 - *w.ground.at(v[0], v[1]) as f64)).collect();
                    eprintln!("  row j{:+}: {}", dj, row.join(" "));
                }
                for (di, dj) in [(0, 0), (1, 0), (0, 1), (-1, 0), (0, -1)] {
                    let (i, j) = (v[0] + di, v[1] + dj);
                    let g = *w.ground.at(i, j) as f64;
                    let gr = (g / 0.1).floor() as i32;
                    let col: Vec<(i32, u8)> = (gr - 3..gr + 40).filter(|r| w.voxel(i, *r, j) != 0).map(|r| (r - gr, w.voxel(i, r, j))).collect();
                    let names: Vec<String> = w.mat_ids.iter().filter(|(_, id)| col.iter().any(|c| c.1 == **id)).map(|(n, id)| format!("{id}={n}")).collect();
                    let bid = if *w.owner.at(i, j) >= 16 { let b = &w.buildings[(*w.owner.at(i, j) - 16) as usize]; format!("bld {} owned {} floor {} base {} eave {} door {}", b.id, b.owned, b.floor_row - gr, b.base_row - gr, b.eave_row - gr, b.has_door) } else { String::new() };
                    eprintln!("cell ({i},{j}) {bid} ground {g:.3} terrain {:.3} zone {} owner {} mat {} reserved_top {} (rel {}) voxels(rel row, mat) {:?} {:?}", w.terrain.at(i, j), w.zone.at(i, j), w.owner.at(i, j), w.mat.at(i, j), w.reserved_top.at(i, j), *w.reserved_top.at(i, j) - gr, col, names);
                }
            }
        }
        return;
    }
    let summary = voxbox::export::export(&w, &a[8]).unwrap_or_else(|e| {
        eprintln!("{e}");
        std::process::exit(1)
    });
    println!("  exported: {summary}");
    if flag("--check") {
        report("real heights", &checks::run(&w), &mut all_ok);
        let fp = w.fingerprint();
        let w2 = build(&spec, &rules, &map, &mut dem);
        let same = w2.fingerprint() == fp;
        drop(w2);
        report("repeat", &[checks::Check { name: "building twice gives exactly the same world", pass: same, tested: 1, failed: !same as usize, detail: "fingerprints differ".into() }], &mut all_ok);
        let mut shifted = PlaceSpec::centred(&a[2], lat, lon, size);
        shifted.core.i0 += 640;
        shifted.core.i1 += 640;
        shifted.core.j0 -= 448;
        shifted.core.j1 -= 448;
        let ws = build(&shifted, &rules, &map, &mut dem);
        report("shifted area", &[checks::compare(&w, &ws)], &mut all_ok);
        drop(ws);
        drop(w);
        spec.exaggeration = 3.0;
        let w3 = build(&spec, &rules, &map, &mut dem);
        report("hills exaggerated 3x", &checks::run(&w3), &mut all_ok);
        if flag("--x3") {
            let s = voxbox::export::export(&w3, &format!("{}_x3", a[8])).unwrap_or_default();
            println!("  exported 3x: {s}");
        }
        drop(w3);
        // Broken map data: the same place with a third of its features damaged at random
        // (tags removed, outlines cut short or doubled back, points thrown far away).
        // It must build without stopping and still pass every check.
        spec.exaggeration = 1.0;
        let bad = voxbox::checks::damage(&map, 7);
        let wb = build(&spec, &rules, &bad, &mut dem);
        report("damaged map data", &checks::run(&wb), &mut all_ok);
        drop(wb);
        let mut flat = Dem::flat();
        let wf = build(&spec, &rules, &map, &mut flat);
        report("no elevation data at all", &checks::run(&wf), &mut all_ok);
        println!("  RESULT {}: {}", spec.name, if all_ok { "ALL CHECKS PASS" } else { "CHECKS FAILED" });
    }
    if !all_ok {
        std::process::exit(1);
    }
}
