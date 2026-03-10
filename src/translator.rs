//! Translator module: converts VRP-REP XML instances to frvcpy-compatible JSON/RawInstance.
//!
//! This mirrors Python's `translator.py` and supports the same VRP-REP instance format.

use crate::core::{RawBreakpoint, RawCsDetail, RawInstance};
use quick_xml::events::{BytesStart, Event};
use quick_xml::reader::Reader;
use std::collections::HashMap;
use std::fs;

// ───────────────────── Internal XML node representation ─────────────────────

#[derive(Debug, Clone)]
struct XmlNode {
    id: usize,
    cx: f64,
    cy: f64,
    node_type: String, // "0", "1", or "2"
    cs_type: Option<String>,
}

#[derive(Debug, Clone)]
struct XmlBreakpoint {
    battery_level: f64,
    charging_time: f64,
}

#[derive(Debug, Clone)]
struct XmlChargingFunction {
    cs_type: String,
    breakpoints: Vec<XmlBreakpoint>,
}

#[derive(Debug, Clone)]
struct XmlRequest {
    node: usize,
    service_time: Option<f64>,
}

// ───────────────────── Distance helpers ─────────────────────

fn euclidean_dist(a: &XmlNode, b: &XmlNode) -> f64 {
    ((a.cx - b.cx).powi(2) + (a.cy - b.cy).powi(2)).sqrt()
}

fn manhattan_dist(a: &XmlNode, b: &XmlNode) -> f64 {
    (a.cx - b.cx).abs() + (a.cy - b.cy).abs()
}

fn dist(a: &XmlNode, b: &XmlNode, dist_type: &str) -> f64 {
    match dist_type {
        "manhattan" => manhattan_dist(a, b),
        _ => euclidean_dist(a, b),
    }
}

fn travel_time(a: &XmlNode, b: &XmlNode, speed: f64, dist_type: &str) -> f64 {
    dist(a, b, dist_type) / speed
}

fn energy_consumed(a: &XmlNode, b: &XmlNode, consump_rate: f64, dist_type: &str) -> f64 {
    dist(a, b, dist_type) * consump_rate
}

// ───────────────────── XML Parsing ─────────────────────

/// Get attribute value from a BytesStart element.
fn get_attr(e: &BytesStart, key: &str) -> Option<String> {
    for attr in e.attributes().flatten() {
        if attr.key.as_ref() == key.as_bytes() {
            return Some(String::from_utf8_lossy(&attr.value).to_string());
        }
    }
    None
}


/// Parsed result from a VRP-REP XML instance.
struct ParsedVrpRep {
    nodes: Vec<XmlNode>,
    charging_functions: Vec<XmlChargingFunction>,
    requests: Vec<XmlRequest>,
    dist_type: String,
    max_travel_time: Option<f64>,
    speed: f64,
    consump_rate: f64,
    battery_capacity: f64,
}

/// Parse the entire VRP-REP XML instance.
fn parse_vrprep_xml(xml_content: &str) -> ParsedVrpRep {
    let mut reader = Reader::from_str(xml_content);
    reader.config_mut().trim_text(true);

    let mut nodes: Vec<XmlNode> = Vec::new();
    let mut charging_functions: Vec<XmlChargingFunction> = Vec::new();
    let mut requests: Vec<XmlRequest> = Vec::new();

    let mut dist_type = String::new(); // will be set based on <euclidean/> or <manhattan/>
    let mut speed: f64 = 0.0;
    let mut consump_rate: f64 = 0.0;
    let mut battery_capacity: f64 = 0.0;
    let mut max_travel_time: Option<f64> = None;

    let mut buf = Vec::new();

    // State tracking for nested parsing
    enum State {
        Root,
        Node(usize, String, f64, f64, Option<String>), // id, type, cx, cy, cs_type
        NodeCx(usize, String, f64, f64, Option<String>),
        NodeCy(usize, String, f64, f64, Option<String>),
        NodeCsType(usize, String, f64, f64, Option<String>),
        Request(usize, Option<f64>),
        RequestServiceTime(usize, Option<f64>),
        VehicleProfile,
        SpeedFactor,
        MaxTravelTime,
        ConsumptionRate,
        BatteryCapacity,
        ChargingFunction(String, Vec<XmlBreakpoint>),
        Breakpoint(String, Vec<XmlBreakpoint>, f64, f64),
        BatteryLevel(String, Vec<XmlBreakpoint>, f64, f64),
        ChargingTime(String, Vec<XmlBreakpoint>, f64, f64),
    }

    let mut state = State::Root;

    loop {
        match reader.read_event_into(&mut buf) {
            Ok(Event::Eof) => break,
            Ok(Event::Empty(ref e)) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).to_string();
                match tag.as_str() {
                    "euclidean" => dist_type = "euclidean".to_string(),
                    "manhattan" => dist_type = "manhattan".to_string(),
                    _ => {}
                }
            }
            Ok(Event::Start(ref e)) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).to_string();

                state = match state {
                    State::Root => {
                        match tag.as_str() {
                            "node" => {
                                let id: usize = get_attr(e, "id").unwrap_or_default().parse().unwrap_or(0);
                                let node_type = get_attr(e, "type").unwrap_or_else(|| "1".to_string());
                                State::Node(id, node_type, 0.0, 0.0, None)
                            }
                            "request" => {
                                let node_id: usize = get_attr(e, "node").unwrap_or_default().parse().unwrap_or(0);
                                State::Request(node_id, None)
                            }
                            "vehicle_profile" => State::VehicleProfile,
                            _ => State::Root,
                        }
                    }
                    State::Node(id, nt, cx, cy, cs) => {
                        match tag.as_str() {
                            "cx" => State::NodeCx(id, nt, cx, cy, cs),
                            "cy" => State::NodeCy(id, nt, cx, cy, cs),
                            "cs_type" => State::NodeCsType(id, nt, cx, cy, cs),
                            _ => State::Node(id, nt, cx, cy, cs),
                        }
                    }
                    State::Request(node_id, st) => {
                        match tag.as_str() {
                            "service_time" => State::RequestServiceTime(node_id, st),
                            _ => State::Request(node_id, st),
                        }
                    }
                    State::VehicleProfile => {
                        match tag.as_str() {
                            "speed_factor" => State::SpeedFactor,
                            "max_travel_time" => State::MaxTravelTime,
                            "consumption_rate" => State::ConsumptionRate,
                            "battery_capacity" => State::BatteryCapacity,
                            "function" => {
                                let cs_t = get_attr(e, "cs_type").unwrap_or_default();
                                State::ChargingFunction(cs_t, Vec::new())
                            }
                            _ => State::VehicleProfile,
                        }
                    }
                    State::ChargingFunction(cs_t, bps) => {
                        match tag.as_str() {
                            "breakpoint" => State::Breakpoint(cs_t, bps, 0.0, 0.0),
                            _ => State::ChargingFunction(cs_t, bps),
                        }
                    }
                    State::Breakpoint(cs_t, bps, bl, ct) => {
                        match tag.as_str() {
                            "battery_level" => State::BatteryLevel(cs_t, bps, bl, ct),
                            "charging_time" => State::ChargingTime(cs_t, bps, bl, ct),
                            _ => State::Breakpoint(cs_t, bps, bl, ct),
                        }
                    }
                    other => other,
                };
            }
            Ok(Event::Text(ref e)) => {
                let txt = e.unescape().unwrap_or_default().trim().to_string();
                state = match state {
                    State::NodeCx(id, nt, _, cy, cs) => {
                        State::NodeCx(id, nt, txt.parse().unwrap_or(0.0), cy, cs)
                    }
                    State::NodeCy(id, nt, cx, _, cs) => {
                        State::NodeCy(id, nt, cx, txt.parse().unwrap_or(0.0), cs)
                    }
                    State::NodeCsType(id, nt, cx, cy, _) => {
                        State::NodeCsType(id, nt, cx, cy, Some(txt))
                    }
                    State::RequestServiceTime(node_id, _) => {
                        State::RequestServiceTime(node_id, Some(txt.parse().unwrap_or(0.0)))
                    }
                    State::SpeedFactor => {
                        speed = txt.parse().unwrap_or(0.0);
                        State::SpeedFactor
                    }
                    State::MaxTravelTime => {
                        max_travel_time = Some(txt.parse().unwrap_or(0.0));
                        State::MaxTravelTime
                    }
                    State::ConsumptionRate => {
                        consump_rate = txt.parse().unwrap_or(0.0);
                        State::ConsumptionRate
                    }
                    State::BatteryCapacity => {
                        battery_capacity = txt.parse().unwrap_or(0.0);
                        State::BatteryCapacity
                    }
                    State::BatteryLevel(cs_t, bps, _, ct) => {
                        State::BatteryLevel(cs_t, bps, txt.parse().unwrap_or(0.0), ct)
                    }
                    State::ChargingTime(cs_t, bps, bl, _) => {
                        State::ChargingTime(cs_t, bps, bl, txt.parse().unwrap_or(0.0))
                    }
                    other => other,
                };
            }
            Ok(Event::End(ref e)) => {
                let tag = String::from_utf8_lossy(e.name().as_ref()).to_string();
                state = match state {
                    State::NodeCx(id, nt, cx, cy, cs) => State::Node(id, nt, cx, cy, cs),
                    State::NodeCy(id, nt, cx, cy, cs) => State::Node(id, nt, cx, cy, cs),
                    State::NodeCsType(id, nt, cx, cy, cs) => State::Node(id, nt, cx, cy, cs),
                    State::Node(id, nt, cx, cy, cs) if tag == "node" => {
                        nodes.push(XmlNode {
                            id,
                            cx,
                            cy,
                            node_type: nt,
                            cs_type: cs,
                        });
                        State::Root
                    }
                    State::Node(id, nt, cx, cy, cs) => State::Node(id, nt, cx, cy, cs),
                    State::RequestServiceTime(node_id, st) => State::Request(node_id, st),
                    State::Request(node_id, st) if tag == "request" => {
                        requests.push(XmlRequest {
                            node: node_id,
                            service_time: st,
                        });
                        State::Root
                    }
                    State::Request(node_id, st) => State::Request(node_id, st),
                    State::SpeedFactor => State::VehicleProfile,
                    State::MaxTravelTime => State::VehicleProfile,
                    State::ConsumptionRate => State::VehicleProfile,
                    State::BatteryCapacity => State::VehicleProfile,
                    State::BatteryLevel(cs_t, bps, bl, ct) => State::Breakpoint(cs_t, bps, bl, ct),
                    State::ChargingTime(cs_t, bps, bl, ct) => State::Breakpoint(cs_t, bps, bl, ct),
                    State::Breakpoint(cs_t, mut bps, bl, ct) if tag == "breakpoint" => {
                        bps.push(XmlBreakpoint {
                            battery_level: bl,
                            charging_time: ct,
                        });
                        State::ChargingFunction(cs_t, bps)
                    }
                    State::Breakpoint(cs_t, bps, bl, ct) => State::Breakpoint(cs_t, bps, bl, ct),
                    State::ChargingFunction(cs_t, bps) if tag == "function" => {
                        charging_functions.push(XmlChargingFunction {
                            cs_type: cs_t,
                            breakpoints: bps,
                        });
                        State::VehicleProfile
                    }
                    State::ChargingFunction(cs_t, bps) => State::ChargingFunction(cs_t, bps),
                    State::VehicleProfile if tag == "vehicle_profile" => State::Root,
                    other => other,
                };
            }
            Err(e) => panic!("Error parsing VRP-REP XML instance: {:?}", e),
            _ => {}
        }
        buf.clear();
    }

    if dist_type.is_empty() {
        eprintln!(
            "WARNING: An unrecognized (or no) distance type was listed in the instance. \
             Assuming Euclidean distance calculations."
        );
        dist_type = "euclidean".to_string();
    }

    ParsedVrpRep {
        nodes,
        charging_functions,
        requests,
        dist_type,
        max_travel_time,
        speed,
        consump_rate,
        battery_capacity,
    }
}

// ───────────────────── Public API ─────────────────────

/// Translate a VRP-REP XML instance file into a `RawInstance`.
///
/// # Arguments
/// * `from_filename` - Path to the VRP-REP XML instance file.
/// * `depot_charging` - Whether the depot should be treated as a CS (default: true).
///
/// # Returns
/// A `RawInstance` suitable for passing to the Solver.
pub fn translate(from_filename: &str, depot_charging: bool) -> RawInstance {
    let xml_content = fs::read_to_string(from_filename)
        .unwrap_or_else(|_| panic!("Unable to read VRP-REP instance file: {}", from_filename));

    let parsed = parse_vrprep_xml(&xml_content);

    // Compute type_to_speed (speed rank): rank 0 = fastest
    let type_to_speed = get_type_to_speed(&parsed.charging_functions);

    // Append depot as a CS if requested
    let mut nodes = parsed.nodes;
    if depot_charging {
        let fastest = type_to_speed
            .iter()
            .find(|(_, &rank)| rank == 0)
            .map(|(t, _)| t.clone())
            .expect("No charging function types found; cannot determine fastest charger for depot");
        let depot = &nodes[0];
        let depot_cs = XmlNode {
            id: nodes.len(),
            cx: depot.cx,
            cy: depot.cy,
            node_type: "2".to_string(),
            cs_type: Some(fastest),
        };
        nodes.push(depot_cs);
    }

    // Collect CS nodes
    let css: Vec<&XmlNode> = nodes.iter().filter(|n| n.node_type == "2").collect();

    // Build process times from requests
    let mut process_times = vec![0.0_f64; nodes.len()];
    for req in &parsed.requests {
        if let Some(st) = req.service_time {
            if req.node < process_times.len() {
                process_times[req.node] = st;
            }
        }
    }

    // Build CS details
    let css_out: Vec<RawCsDetail> = css
        .iter()
        .map(|cs| {
            let cs_type_name = cs.cs_type.as_deref().unwrap_or("");
            RawCsDetail {
                node_id: cs.id,
                cs_type: *type_to_speed.get(cs_type_name).unwrap_or(&0),
            }
        })
        .collect();

    // Build breakpoints by type
    let breakpoints: Vec<RawBreakpoint> = parsed.charging_functions
        .iter()
        .map(|cf| {
            let rank = *type_to_speed.get(&cf.cs_type).unwrap_or(&0);
            RawBreakpoint {
                cs_type: rank,
                time: cf.breakpoints.iter().map(|bp| bp.charging_time).collect(),
                charge: cf.breakpoints.iter().map(|bp| bp.battery_level).collect(),
            }
        })
        .collect();

    // Build energy and time matrices
    let n = nodes.len();
    let mut energy_matrix = vec![vec![0.0; n]; n];
    let mut time_matrix = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            energy_matrix[i][j] = energy_consumed(&nodes[i], &nodes[j], parsed.consump_rate, &parsed.dist_type);
            time_matrix[i][j] = travel_time(&nodes[i], &nodes[j], parsed.speed, &parsed.dist_type);
        }
    }

    RawInstance {
        max_q: parsed.battery_capacity,
        t_max: parsed.max_travel_time,
        css: css_out,
        process_times: Some(process_times),
        breakpoints_by_type: breakpoints,
        energy_matrix,
        time_matrix,
    }
}

/// Translate and write the result to a JSON file.
pub fn translate_to_file(from_filename: &str, to_filename: &str, depot_charging: bool) {
    let instance = translate(from_filename, depot_charging);
    let json = serde_json::to_string(&instance).expect("Failed to serialize instance to JSON");
    fs::write(to_filename, json).expect("Failed to write output file");
}

/// Given a list of charging functions, returns a map from CS type name to speed rank.
/// Speed rank 0 = fastest, 1 = second fastest, etc.
fn get_type_to_speed(cfs: &[XmlChargingFunction]) -> HashMap<String, usize> {
    let mut rates: Vec<(String, f64)> = cfs
        .iter()
        .map(|cf| {
            let max_rate = if cf.breakpoints.len() >= 2 {
                (cf.breakpoints[1].battery_level - cf.breakpoints[0].battery_level)
                    / (cf.breakpoints[1].charging_time - cf.breakpoints[0].charging_time)
            } else {
                0.0
            };
            (cf.cs_type.clone(), max_rate)
        })
        .collect();

    // Sort by rate descending (fastest first)
    rates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    rates
        .into_iter()
        .enumerate()
        .map(|(rank, (cs_type, _))| (cs_type, rank))
        .collect()
}
