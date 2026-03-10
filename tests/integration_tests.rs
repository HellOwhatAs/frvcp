use std::collections::HashMap;
use std::fs;
use std::time::Instant;

use approx::assert_abs_diff_eq;
use serde::Deserialize;

use frvcpy::core::{FrvcpInstance, RawBreakpoint, RawCsDetail, RawInstance};
use frvcpy::solver::Solver;
use frvcpy::translator;

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

/// Test translation of VRP-REP XML instance to frvcpy format.
/// Compares the translated instance against the known reference instance.
#[test]
fn test_translation() {
    let translated = translator::translate("frvcpy/test/data/vrprep-instance.xml", true);
    let reference = load_reference_instance();

    // Compare key fields
    assert_abs_diff_eq!(translated.max_q, reference.max_q, epsilon = 1e-3);
    assert_eq!(translated.t_max, reference.t_max);
    assert_eq!(translated.css.len(), reference.css.len());
    assert_eq!(
        translated.breakpoints_by_type.len(),
        reference.breakpoints_by_type.len()
    );

    // Compare process_times
    let trans_pt = translated.process_times.as_deref().unwrap_or(&[]);
    let ref_pt = reference.process_times.as_deref().unwrap_or(&[]);
    assert_eq!(trans_pt.len(), ref_pt.len());
    for (a, b) in trans_pt.iter().zip(ref_pt.iter()) {
        assert_abs_diff_eq!(a, b, epsilon = 1e-3);
    }

    // Compare CSS details (sorted by node_id for deterministic comparison)
    let mut trans_css = translated.css.clone();
    trans_css.sort_by_key(|cs| cs.node_id);
    let mut ref_css = reference.css.clone();
    ref_css.sort_by_key(|cs| cs.node_id);
    for (a, b) in trans_css.iter().zip(ref_css.iter()) {
        assert_eq!(a.node_id, b.node_id);
        assert_eq!(a.cs_type, b.cs_type);
    }

    // Compare breakpoints (sorted by cs_type)
    let mut trans_bps = translated.breakpoints_by_type.clone();
    trans_bps.sort_by_key(|bp| bp.cs_type);
    let mut ref_bps = reference.breakpoints_by_type.clone();
    ref_bps.sort_by_key(|bp| bp.cs_type);
    for (a, b) in trans_bps.iter().zip(ref_bps.iter()) {
        assert_eq!(a.cs_type, b.cs_type);
        assert_eq!(a.time.len(), b.time.len());
        for (t1, t2) in a.time.iter().zip(b.time.iter()) {
            assert_abs_diff_eq!(t1, t2, epsilon = 1e-3);
        }
        for (c1, c2) in a.charge.iter().zip(b.charge.iter()) {
            assert_abs_diff_eq!(c1, c2, epsilon = 1e-3);
        }
    }

    // Compare energy matrix
    assert_eq!(translated.energy_matrix.len(), reference.energy_matrix.len());
    for (row_t, row_r) in translated.energy_matrix.iter().zip(reference.energy_matrix.iter()) {
        assert_eq!(row_t.len(), row_r.len());
        for (a, b) in row_t.iter().zip(row_r.iter()) {
            assert_abs_diff_eq!(a, b, epsilon = 1e-3);
        }
    }

    // Compare time matrix
    assert_eq!(translated.time_matrix.len(), reference.time_matrix.len());
    for (row_t, row_r) in translated.time_matrix.iter().zip(reference.time_matrix.iter()) {
        assert_eq!(row_t.len(), row_r.len());
        for (a, b) in row_t.iter().zip(row_r.iter()) {
            assert_abs_diff_eq!(a, b, epsilon = 1e-3);
        }
    }
}

/// Test triangle inequality check.
#[test]
fn test_triangle_inequality_check() {
    let instance = load_reference_instance();
    // The reference instance should satisfy the triangle inequality
    // (this should not panic)
    let _inst = FrvcpInstance::new_checked(instance, true);
}

/// Test write_solution produces valid output.
#[test]
fn test_write_solution() {
    let test_data = load_test_data();
    let instance = load_reference_instance();
    let q_init = instance.max_q;

    let route_info = &test_data["route_tc0c40s8cf0_23"];
    let mut solver = Solver::new(instance, route_info.route.clone(), q_init, true);
    solver.solve();

    let tmp_dir = std::env::temp_dir();
    let tmp_file = tmp_dir.join("frvcpy_test_solution.xml");
    let tmp_path = tmp_file.to_str().unwrap();
    solver.write_solution(tmp_path, "test_instance");

    // Verify file was created and contains expected content
    let content = fs::read_to_string(tmp_path).expect("Could not read solution file");
    assert!(content.contains("solution"));
    assert!(content.contains("test_instance"));
    assert!(content.contains("route"));
    assert!(content.contains("node"));

    // Clean up
    let _ = fs::remove_file(tmp_path);
}

/// Benchmark test: time all 133 E-VRP-NL instances to verify efficiency.
#[test]
fn test_benchmark_all_instances() {
    let test_data = load_test_data();
    let ref_instance = load_reference_instance();
    let q_init = ref_instance.max_q;

    let start = Instant::now();

    for (_name, route_info) in &test_data {
        let mut solver = Solver::new(ref_instance.clone(), route_info.route.clone(), q_init, true);
        let (obj, _) = solver.solve();
        assert_abs_diff_eq!(obj, route_info.obj, epsilon = 1e-3);
    }

    let elapsed = start.elapsed();
    let count = test_data.len();
    eprintln!(
        "\n=== BENCHMARK: Solved {} instances in {:.3}s ({:.3}ms avg) ===",
        count,
        elapsed.as_secs_f64(),
        elapsed.as_secs_f64() * 1000.0 / count as f64
    );

    // Efficiency assertion: all 133 instances should solve in under 5 seconds
    // (typically well under 1 second in release mode)
    assert!(
        elapsed.as_secs_f64() < 5.0,
        "All 133 instances took {:.3}s, expected < 5s",
        elapsed.as_secs_f64()
    );
}

/// Test that translated instance produces the same solver results.
#[test]
fn test_translation_solver_consistency() {
    let test_data = load_test_data();
    let translated = translator::translate("frvcpy/test/data/vrprep-instance.xml", true);
    let q_init = translated.max_q;

    // Test a few routes with the translated instance
    let route_info = &test_data["route_tc0c40s8cf0_23"];
    let mut solver = Solver::new(translated, route_info.route.clone(), q_init, true);
    let (obj, _) = solver.solve();
    assert_abs_diff_eq!(obj, route_info.obj, epsilon = 1e-3);
}
