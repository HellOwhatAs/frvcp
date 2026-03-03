use std::collections::HashMap;
use std::fs;

use approx::assert_abs_diff_eq;
use serde::Deserialize;

use frvcpy::core::{RawBreakpoint, RawCsDetail, RawInstance};
use frvcpy::solver::Solver;

#[derive(Deserialize)]
struct RouteInfo {
    obj: f64,
    route: Vec<usize>,
}

fn load_test_data() -> HashMap<String, RouteInfo> {
    let data = fs::read_to_string("frvcpy/test/data/testdata.json")
        .expect("Unable to read testdata.json");
    serde_json::from_str(&data).expect("Invalid testdata JSON")
}

fn load_reference_instance() -> RawInstance {
    let data = fs::read_to_string("frvcpy/test/data/frvcpy-instance.json")
        .expect("Unable to read frvcpy-instance.json");
    serde_json::from_str(&data).expect("Invalid instance JSON")
}

#[test]
fn test_manuscript_instance() {
    let test_data = load_test_data();
    let instance = load_reference_instance();
    let q_init = instance.max_q;

    let route_info = &test_data["route_tc0c40s8cf0_23"];
    let mut solver = Solver::new(instance, route_info.route.clone(), q_init, true);
    let (obj, _) = solver.solve();

    assert_abs_diff_eq!(obj, route_info.obj, epsilon = 1e-3);
}

#[test]
fn test_evrpnl_instances() {
    let test_data = load_test_data();
    let ref_instance = load_reference_instance();
    let q_init = ref_instance.max_q;

    for (_name, route_info) in &test_data {
        let mut solver = Solver::new(ref_instance.clone(), route_info.route.clone(), q_init, true);
        let (obj, _) = solver.solve();

        assert_abs_diff_eq!(obj, route_info.obj, epsilon = 1e-3);
    }
    assert_eq!(test_data.len(), 133);
}

fn make_toy_instance(max_q: f64) -> RawInstance {
    RawInstance {
        max_q,
        t_max: Some(f64::INFINITY),
        css: vec![RawCsDetail {
            node_id: 4,
            cs_type: 0,
        }],
        process_times: Some(vec![0.0, 0.5, 0.5, 0.5, 0.0]),
        breakpoints_by_type: vec![RawBreakpoint {
            cs_type: 0,
            time: vec![0.0, 0.31, 0.39, 0.51],
            charge: vec![0.0, 13600.0, 15200.0, 16000.0],
        }],
        energy_matrix: vec![
            vec![0.0, 1.0, 2.0, 0.0, 2.0],
            vec![1.0, 0.0, 1.0, 1.0, 1.0],
            vec![2.0, 1.0, 0.0, 2.0, 0.0],
            vec![0.0, 1.0, 2.0, 0.0, 2.0],
            vec![2.0, 1.0, 0.0, 2.0, 0.0],
        ],
        time_matrix: vec![vec![0.0; 5]; 5],
    }
}

#[test]
fn test_destination_no_min_energy() {
    let instance = make_toy_instance(4.0);
    let route = vec![0, 1, 2, 3];
    let q_init = 4.0;

    let mut solver = Solver::new(instance, route, q_init, false);
    let (_, feas_route) = solver.solve();

    let expected: Vec<(usize, Option<f64>)> =
        vec![(0, None), (1, None), (2, None), (3, None)];
    assert_eq!(feas_route, Some(expected));
}

#[test]
fn test_destination_cs_detour_feasibility() {
    let instance = make_toy_instance(3.9);
    let route = vec![0, 1, 2, 3];
    let q_init = 3.9;

    let mut solver = Solver::new(instance, route, q_init, false);
    let (duration, feas_route) = solver.solve();

    assert!(feas_route.is_some(), "Expected a feasible route");
    assert!(duration < f64::INFINITY, "Expected finite duration");
}
