//! Reads an OpenStreetMap region file (.osm.pbf) and returns the features
//! inside an area, in local metres. Missing or broken data is skipped here
//! so later stages never see half a feature.

use crate::geo::{Proj, P2};
use osmpbf::{Element, ElementReader, RelMemberType};
use std::collections::{HashMap, HashSet};

#[derive(Clone, Debug, Default)]
pub struct Tags(pub Vec<(String, String)>);

impl Tags {
    pub fn get(&self, k: &str) -> Option<&str> {
        self.0.iter().find(|(a, _)| a == k).map(|(_, v)| v.as_str())
    }
    pub fn is(&self, k: &str, v: &str) -> bool {
        self.get(k) == Some(v)
    }
    pub fn has(&self, k: &str) -> bool {
        self.get(k).is_some()
    }
    /// Parses "12", "12.5", "12 m", "12,5" as metres. Anything else is None.
    pub fn metres(&self, k: &str) -> Option<f64> {
        let v = self.get(k)?.trim().replace(',', ".");
        let v = v.trim_end_matches("metres").trim_end_matches("meters").trim_end_matches('m').trim();
        let x: f64 = v.parse().ok()?;
        if x.is_finite() && x > 0.0 && x < 2000.0 {
            Some(x)
        } else {
            None
        }
    }
    pub fn number(&self, k: &str) -> Option<f64> {
        let x: f64 = self.get(k)?.trim().parse().ok()?;
        if x.is_finite() {
            Some(x)
        } else {
            None
        }
    }
}

#[derive(Clone, Debug)]
pub struct Line {
    pub id: i64,
    pub tags: Tags,
    pub pts: Vec<P2>,
    pub node_ids: Vec<i64>,
}

#[derive(Clone, Debug)]
pub struct Area {
    pub id: i64,
    pub tags: Tags,
    pub rings: Vec<Vec<P2>>,
}

#[derive(Clone, Debug)]
pub struct Point {
    pub id: i64,
    pub tags: Tags,
    pub p: P2,
}

#[derive(Clone, Debug, Default)]
pub struct MapData {
    pub lines: Vec<Line>,
    pub areas: Vec<Area>,
    pub points: Vec<Point>,
}

const AREA_KEYS: [&str; 8] = ["building", "building:part", "landuse", "natural", "leisure", "amenity", "waterway", "area:highway"];
const LINE_KEYS: [&str; 4] = ["highway", "barrier", "waterway", "railway"];
const POINT_KEYS: [&str; 6] = ["natural", "amenity", "highway", "barrier", "entrance", "man_made"];

fn tags_of<'a, I: Iterator<Item = (&'a str, &'a str)>>(it: I) -> Tags {
    Tags(it.map(|(k, v)| (k.to_string(), v.to_string())).collect())
}

struct RawWay {
    id: i64,
    tags: Tags,
    refs: Vec<i64>,
}

/// Loads everything that touches the box (in local metres around `proj`), grown by `margin`.
pub fn load(path: &str, proj: &Proj, min: P2, max: P2, margin: f64) -> Result<MapData, String> {
    let (lat_a, lon_a) = proj.to_latlon(P2::new(min.x - margin, max.z + margin));
    let (lat_b, lon_b) = proj.to_latlon(P2::new(max.x + margin, min.z - margin));
    let (lat0, lat1) = (lat_a.min(lat_b), lat_a.max(lat_b));
    let (lon0, lon1) = (lon_a.min(lon_b), lon_a.max(lon_b));
    let open = || ElementReader::from_path(path).map_err(|e| format!("cannot open {path}: {e}"));

    // Pass 1: nodes inside the box, ways touching them, relations touching those ways.
    let mut inbox: HashSet<i64> = HashSet::new();
    let mut points_raw: Vec<(i64, Tags, f64, f64)> = Vec::new();
    let mut ways: Vec<RawWay> = Vec::new();
    let mut kept: HashSet<i64> = HashSet::new();
    let mut rels: Vec<(i64, Tags, Vec<(i64, bool)>)> = Vec::new();
    let mut wanted_ways: HashSet<i64> = HashSet::new();
    let mut node = |id: i64, lat: f64, lon: f64, tags: Tags, inbox: &mut HashSet<i64>, pr: &mut Vec<(i64, Tags, f64, f64)>| {
        if lat >= lat0 && lat <= lat1 && lon >= lon0 && lon <= lon1 {
            inbox.insert(id);
            if POINT_KEYS.iter().any(|k| tags.has(k)) {
                pr.push((id, tags, lat, lon));
            }
        }
    };
    open()?
        .for_each(|el| match el {
            Element::Node(n) => node(n.id(), n.lat(), n.lon(), tags_of(n.tags()), &mut inbox, &mut points_raw),
            Element::DenseNode(n) => node(n.id(), n.lat(), n.lon(), tags_of(n.tags()), &mut inbox, &mut points_raw),
            Element::Way(w) => {
                let refs: Vec<i64> = w.refs().collect();
                if refs.iter().any(|r| inbox.contains(r)) {
                    kept.insert(w.id());
                    ways.push(RawWay { id: w.id(), tags: tags_of(w.tags()), refs });
                }
            }
            Element::Relation(r) => {
                let tags = tags_of(r.tags());
                if !tags.is("type", "multipolygon") {
                    return;
                }
                let mem: Vec<(i64, bool)> = r
                    .members()
                    .filter(|m| m.member_type == RelMemberType::Way)
                    .map(|m| (m.member_id, m.role().map(|s| s == "inner").unwrap_or(false)))
                    .collect();
                if mem.iter().any(|(id, _)| kept.contains(id)) {
                    for (id, _) in &mem {
                        if !kept.contains(id) {
                            wanted_ways.insert(*id);
                        }
                    }
                    rels.push((r.id(), tags, mem));
                }
            }
        })
        .map_err(|e| format!("read error in {path}: {e}"))?;

    // Pass 2: member ways of kept relations that lie outside the box.
    if !wanted_ways.is_empty() {
        open()?
            .for_each(|el| {
                if let Element::Way(w) = el {
                    if wanted_ways.contains(&w.id()) {
                        ways.push(RawWay { id: w.id(), tags: tags_of(w.tags()), refs: w.refs().collect() });
                    }
                }
            })
            .map_err(|e| format!("read error in {path}: {e}"))?;
    }

    // Pass 3: coordinates of every node any kept way uses.
    let mut need: HashSet<i64> = HashSet::new();
    for w in &ways {
        need.extend(w.refs.iter().copied());
    }
    let mut coord: HashMap<i64, P2> = HashMap::with_capacity(need.len());
    open()?
        .for_each(|el| match el {
            Element::Node(n) => {
                if need.contains(&n.id()) {
                    coord.insert(n.id(), proj.to_local(n.lat(), n.lon()));
                }
            }
            Element::DenseNode(n) => {
                if need.contains(&n.id()) {
                    coord.insert(n.id(), proj.to_local(n.lat(), n.lon()));
                }
            }
            _ => {}
        })
        .map_err(|e| format!("read error in {path}: {e}"))?;

    let mut out = MapData::default();
    for (id, tags, lat, lon) in points_raw {
        out.points.push(Point { id, tags, p: proj.to_local(lat, lon) });
    }
    let way_by_id: HashMap<i64, usize> = ways.iter().enumerate().map(|(i, w)| (w.id, i)).collect();
    let pts_of = |w: &RawWay| -> Option<Vec<P2>> { w.refs.iter().map(|r| coord.get(r).copied()).collect() };

    for w in &ways {
        if !kept.contains(&w.id) {
            continue;
        }
        let Some(pts) = pts_of(w) else { continue };
        if pts.len() < 2 {
            continue;
        }
        let closed = w.refs.len() >= 4 && w.refs.first() == w.refs.last();
        let is_area_tag = AREA_KEYS.iter().any(|k| w.tags.has(k)) && !w.tags.is("area", "no");
        let is_line_tag = LINE_KEYS.iter().any(|k| w.tags.has(k));
        // A closed highway is still a road (roundabout) unless it says area=yes.
        let highway_area = w.tags.has("highway") && w.tags.is("area", "yes");
        if closed && (highway_area || (is_area_tag && !(w.tags.has("highway") || w.tags.has("barrier")))) {
            let mut ring = pts.clone();
            ring.pop();
            if ring.len() >= 3 {
                out.areas.push(Area { id: w.id, tags: w.tags.clone(), rings: vec![ring] });
            }
            if !highway_area {
                continue;
            }
        }
        if closed && w.tags.has("barrier") && is_area_tag {
            // e.g. a walled garden: keep the area, and the barrier line below.
            let mut ring = pts.clone();
            ring.pop();
            if ring.len() >= 3 {
                out.areas.push(Area { id: w.id, tags: w.tags.clone(), rings: vec![ring] });
            }
        }
        if is_line_tag && !highway_area {
            out.lines.push(Line { id: w.id, tags: w.tags.clone(), pts, node_ids: w.refs.clone() });
        }
    }

    // Multipolygons: join member ways into closed rings. If a ring cannot be
    // closed the whole relation is dropped (never half an area).
    for (rid, tags, mem) in rels {
        if !AREA_KEYS.iter().any(|k| tags.has(k)) {
            continue;
        }
        let mut segs: Vec<Vec<i64>> = Vec::new();
        let mut ok = true;
        for (id, _) in &mem {
            match way_by_id.get(id) {
                Some(&i) => segs.push(ways[i].refs.clone()),
                None => ok = false,
            }
        }
        if !ok {
            continue;
        }
        let mut rings: Vec<Vec<P2>> = Vec::new();
        while let Some(mut cur) = segs.pop() {
            let mut guard = 0;
            while cur.first() != cur.last() && guard < 10_000 {
                guard += 1;
                let tail = *cur.last().unwrap();
                let Some(k) = segs.iter().position(|s| s.first() == Some(&tail) || s.last() == Some(&tail)) else { break };
                let mut s = segs.swap_remove(k);
                if s.last() == Some(&tail) {
                    s.reverse();
                }
                cur.extend_from_slice(&s[1..]);
            }
            if cur.first() != cur.last() || cur.len() < 4 {
                ok = false;
                break;
            }
            cur.pop();
            let ring: Option<Vec<P2>> = cur.iter().map(|r| coord.get(r).copied()).collect();
            match ring {
                Some(r) => rings.push(r),
                None => {
                    ok = false;
                    break;
                }
            }
        }
        if ok && !rings.is_empty() {
            // Relation ids get a separate id space from ways.
            out.areas.push(Area { id: -rid, tags, rings });
        }
    }

    out.lines.sort_by_key(|l| l.id);
    out.areas.sort_by_key(|a| a.id);
    out.points.sort_by_key(|p| p.id);
    Ok(out)
}
