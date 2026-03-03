//! Solver module: wraps the labeling algorithm with preprocessing
//! (building graph G', computing bounds, etc.).

use crate::algorithm::FrvcpAlgo;
use crate::core::{FrvcpInstance, Node, RawInstance};

use std::fs;

/// A solver for the Fixed Route Vehicle Charging Problem (FRVCP).
pub struct Solver {
    pub instance: FrvcpInstance,
    q_init: f64,
    route: Vec<usize>,
    multi_insert: bool,
    pub solution: Option<(f64, Option<Vec<(usize, Option<f64>)>>)>,
}

impl Solver {
    /// Create a new Solver from a `RawInstance`.
    pub fn new(
        instance: RawInstance,
        route: Vec<usize>,
        q_init: f64,
        multi_insert: bool,
    ) -> Self {
        Solver {
            instance: FrvcpInstance::new(instance),
            q_init,
            route,
            multi_insert,
            solution: None,
        }
    }

    /// Create a new Solver by loading a JSON instance file.
    pub fn from_file(
        filename: &str,
        route: Vec<usize>,
        q_init: f64,
        multi_insert: bool,
    ) -> Self {
        let data = fs::read_to_string(filename).expect("Unable to read instance file");
        let raw: RawInstance =
            serde_json::from_str(&data).expect("Invalid instance JSON");
        Self::new(raw, route, q_init, multi_insert)
    }

    /// Solve the FRVCP defined by the instance, route, and initial energy.
    ///
    /// Returns `(obj, feas_route)` where `obj` is the duration of the
    /// energy-feasible route (or `f64::INFINITY` if infeasible) and
    /// `feas_route` is the list of stops.
    pub fn solve(&mut self) -> (f64, Option<Vec<(usize, Option<f64>)>>) {
        let max_detour_charge_time = self._compute_max_avail_time_detour_charge();

        let mut min_soc_at_departure = self._compute_min_soc_at_departure();
        // destination has no min energy requirement
        min_soc_at_departure[*self.route.last().unwrap()] = 0.0;

        let max_allowed_soc_at_arrival = self._compute_max_soc_at_arrival();

        let possible_direct_connect =
            self._compute_possible_direct_connect(&min_soc_at_departure, &max_allowed_soc_at_arrival);

        let possible_cs_connect =
            self._compute_possible_cs_connections(&min_soc_at_departure, &max_allowed_soc_at_arrival);

        let possible_cs_detour =
            self._compute_possible_cs_detour(max_detour_charge_time, !self.multi_insert);

        let possible_cs_link =
            self._compute_possible_cs_link(max_detour_charge_time, !self.multi_insert);

        // In G', route nodes come first; departure is the first, arrival is the last.
        let node_local_id_dep: usize = 0;
        let node_local_id_arr: usize = self.route.len() - 1;

        let nodes_gpr = self._build_gpr_nodes();

        let adjacencies = self._compute_adjacencies(
            &nodes_gpr,
            &possible_direct_connect,
            &possible_cs_connect,
            &possible_cs_detour,
            &possible_cs_link,
        );

        let (
            min_travel_time_after_node,
            min_energy_consumed_after_node,
            min_travel_charge_time_after_node,
            latest_departure_time,
            min_energy_at_departure,
            max_energy_at_departure,
        ) = self._compute_bounds(&nodes_gpr);

        let mut label_algo = FrvcpAlgo::new(
            self.instance.clone(),
            self.q_init,
            nodes_gpr,
            adjacencies,
            node_local_id_dep,
            node_local_id_arr,
            self.instance.max_slope,
            min_energy_consumed_after_node,
            min_travel_time_after_node,
            min_travel_charge_time_after_node,
            max_energy_at_departure,
            latest_departure_time,
            min_energy_at_departure,
        );

        label_algo.run_algo();

        let obj = label_algo.get_objective_value();
        let opt_route = label_algo.get_optimized_route();
        self.solution = Some((obj, opt_route.clone()));

        (obj, opt_route)
    }

    /// Max amount of time available to detour and charge.
    fn _compute_max_avail_time_detour_charge(&self) -> f64 {
        let travel_time: f64 = self
            .route
            .windows(2)
            .map(|w| self.instance.time_matrix[w[0]][w[1]])
            .sum();
        let process_time: f64 = self
            .route
            .iter()
            .map(|&stop| self.instance.process_times[stop])
            .sum();
        self.instance.t_max - travel_time - process_time
    }

    /// Minimum allowable energy with which we can depart each node.
    fn _compute_min_soc_at_departure(&self) -> Vec<f64> {
        self.instance
            .nodes_g
            .iter()
            .map(|node| self.instance.get_min_energy_to_cs(node.node_id))
            .collect()
    }

    /// Max allowable charge at arrival to any node.
    fn _compute_max_soc_at_arrival(&self) -> Vec<f64> {
        vec![self.instance.max_q; self.instance.n_nodes_g]
    }

    /// For each stop in the route, can we go directly to the next stop?
    fn _compute_possible_direct_connect(
        &self,
        min_soc: &[f64],
        max_soc: &[f64],
    ) -> Vec<bool> {
        (0..self.route.len() - 1)
            .map(|i| {
                let curr = self.route[i];
                let next = self.route[i + 1];
                max_soc[curr] - self.instance.energy_matrix[curr][next]
                    >= min_soc[next]
            })
            .collect()
    }

    /// From each stop, can we detour to a CS, then from that CS to the next stop?
    /// Shape: (route_len-1) x n_cs x 2
    fn _compute_possible_cs_connections(
        &self,
        min_soc: &[f64],
        max_soc: &[f64],
    ) -> Vec<Vec<[bool; 2]>> {
        let route_len = self.route.len();
        let n_cs = self.instance.n_cs;

        let mut result = vec![vec![[false, false]; n_cs]; route_len - 1];
        for i in 0..route_len - 1 {
            let init_loc = self.route[i];
            let next_loc = self.route[i + 1];
            for i_cs in 0..n_cs {
                let cs_id = self.instance.cs_details[i_cs].node_id;
                // can arrive at CS with nonneg energy
                result[i][i_cs][0] =
                    max_soc[init_loc] - self.instance.energy_matrix[init_loc][cs_id] >= 0.0;
                // can depart CS and reach next stop
                result[i][i_cs][1] = self.instance.max_q
                    - self.instance.energy_matrix[cs_id][next_loc]
                    >= min_soc[next_loc];
            }
        }
        result
    }

    /// Is it possible to visit each CS between stops in the route?
    /// Shape: (route_len-1) x n_cs
    fn _compute_possible_cs_detour(
        &self,
        max_detour_charge_time: f64,
        only_one: bool,
    ) -> Vec<Vec<bool>> {
        let route_len = self.route.len();
        let n_cs = self.instance.n_cs;

        let mut result = vec![vec![false; n_cs]; route_len - 1];
        for i in 0..route_len - 1 {
            let stop = self.route[i];
            let next_stop = self.route[i + 1];
            for i_cs in 0..n_cs {
                let cs = self.instance.cs_details[i_cs].node_id;
                let mut detour_time = self.instance.time_matrix[stop][cs]
                    + self.instance.time_matrix[cs][next_stop]
                    - self.instance.time_matrix[stop][next_stop];
                if only_one {
                    let min_charge_amt = self.instance.energy_matrix[stop][cs]
                        + self.instance.energy_matrix[cs][next_stop]
                        - self.instance.energy_matrix[stop][next_stop];
                    if min_charge_amt > self.instance.max_q {
                        detour_time = f64::INFINITY;
                    } else {
                        detour_time += self.instance.get_charging_time(
                            &self.instance.nodes_g[cs],
                            0.0,
                            min_charge_amt,
                        );
                    }
                }
                if detour_time <= max_detour_charge_time {
                    result[i][i_cs] = true;
                }
            }
        }
        result
    }

    /// Can two CSs be connected between stops in the route?
    /// Shape: (route_len-1) x n_cs x n_cs
    fn _compute_possible_cs_link(
        &self,
        max_detour_charge_time: f64,
        only_one: bool,
    ) -> Vec<Vec<Vec<bool>>> {
        let route_len = self.route.len();
        let n_cs = self.instance.n_cs;

        let mut result = vec![vec![vec![false; n_cs]; n_cs]; route_len - 1];

        // if only one CS allowed between stops, no links possible
        if only_one {
            return result;
        }

        for i in 0..route_len - 1 {
            let curr_stop = self.route[i];
            let next_stop = self.route[i + 1];
            for cs1_idx in 0..n_cs {
                let cs1 = self.instance.cs_details[cs1_idx].node_id;
                for cs2_idx in 0..n_cs {
                    if cs1_idx == cs2_idx {
                        continue;
                    }
                    let cs2 = self.instance.cs_details[cs2_idx].node_id;
                    if self.instance.energy_matrix[cs1][cs2] <= self.instance.max_q
                        && (self.instance.time_matrix[curr_stop][cs1]
                            + self.instance.time_matrix[cs1][cs2]
                            + self.instance.time_matrix[cs2][next_stop]
                            - self.instance.time_matrix[curr_stop][next_stop]
                            <= max_detour_charge_time)
                    {
                        result[i][cs1_idx][cs2_idx] = true;
                    }
                }
            }
        }
        result
    }

    /// Build the list of nodes that define the graph G'.
    fn _build_gpr_nodes(&self) -> Vec<Node> {
        let route_nodes: Vec<Node> = self
            .route
            .iter()
            .map(|&i| self.instance.nodes_g[i].clone())
            .collect();
        let cs_nodes = self.instance.get_cs_nodes();
        // Each segment between consecutive route stops gets its own copy of all CS nodes.
        let mut nodes = route_nodes;
        for _ in 0..self.route.len() - 1 {
            nodes.extend(cs_nodes.iter().cloned());
        }
        nodes
    }

    /// Compute the adjacency list for the nodes in G'.
    fn _compute_adjacencies(
        &self,
        nodes: &[Node],
        direct: &[bool],
        cs_conn: &[Vec<[bool; 2]>],
        cs_detour: &[Vec<bool>],
        cs_link: &[Vec<Vec<bool>>],
    ) -> Vec<Vec<usize>> {
        let route_len = self.route.len();
        let n_cs = self.instance.n_cs;
        let mut adjs: Vec<Vec<usize>> = vec![vec![]; nodes.len()];

        // direct connections and one-off detours
        for i in 0..route_len - 1 {
            if direct[i] {
                adjs[i].push(i + 1);
            }
            for j in (route_len + i * n_cs)..(route_len + (i + 1) * n_cs) {
                let i_cs = (j - route_len) % n_cs;
                if cs_detour[i][i_cs] {
                    if cs_conn[i][i_cs][0] {
                        adjs[i].push(j);
                    }
                    if cs_conn[i][i_cs][1] {
                        adjs[j].push(i + 1);
                    }
                }
            }
        }
        // intra-cs links
        for i in 0..route_len - 1 {
            let begin_idx = route_len + i * n_cs;
            let end_idx = route_len + (i + 1) * n_cs;
            for mid_idx1 in begin_idx..end_idx {
                let i_cs1 = (mid_idx1 - route_len) % n_cs;
                for mid_idx2 in begin_idx..end_idx {
                    let i_cs2 = (mid_idx2 - route_len) % n_cs;
                    if cs_link[i][i_cs1][i_cs2] {
                        adjs[mid_idx1].push(mid_idx2);
                    }
                }
            }
        }
        adjs
    }

    /// Compute the 6-tuple of bounds used in the labeling algorithm.
    fn _compute_bounds(
        &self,
        nodes: &[Node],
    ) -> (Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>, Vec<f64>) {
        let n = nodes.len();
        let route_len = self.route.len();
        let n_cs = self.instance.n_cs;

        let mut min_travel_time_after_node = vec![0.0; n];
        let mut min_energy_consumed_after_node = vec![0.0; n];
        let mut min_travel_charge_time_after_node = vec![0.0; n];
        let mut latest_departure_time = vec![0.0; n];
        let mut min_energy_at_departure = vec![0.0; n];
        let mut max_energy_at_departure = vec![0.0; n];

        // initialize trackers
        let mut energy: f64 = 0.0;
        let mut time: f64 = 0.0;
        let mut time_charge: f64 = 0.0;
        let mut next_id = *self.route.last().unwrap();

        // set entries for last stop in route
        min_travel_time_after_node[route_len - 1] = time;
        min_energy_consumed_after_node[route_len - 1] = energy;
        min_travel_charge_time_after_node[route_len - 1] = time_charge;

        // set entries for all others
        for i in (0..route_len - 1).rev() {
            for j in (route_len + i * n_cs)..(route_len + (i + 1) * n_cs) {
                let curr_id = nodes[j].node_id;
                min_travel_time_after_node[j] = time
                    + self.instance.time_matrix[curr_id][next_id]
                    + self.instance.process_times[next_id];
                min_energy_consumed_after_node[j] =
                    energy + self.instance.energy_matrix[curr_id][next_id];
                min_travel_charge_time_after_node[j] = time_charge
                    + self.instance.time_matrix[curr_id][next_id]
                    + self.instance.process_times[next_id]
                    + self.instance.energy_matrix[curr_id][next_id]
                        / self.instance.max_slope;
            }
            let curr_id = self.route[i];
            time += self.instance.time_matrix[curr_id][next_id]
                + self.instance.process_times[next_id];
            energy += self.instance.energy_matrix[curr_id][next_id];
            time_charge += self.instance.time_matrix[curr_id][next_id]
                + self.instance.process_times[next_id]
                + self.instance.energy_matrix[curr_id][next_id]
                    / self.instance.max_slope;
            min_travel_time_after_node[i] = time;
            min_energy_consumed_after_node[i] = energy;
            min_travel_charge_time_after_node[i] = time_charge;
            next_id = curr_id;
        }

        // bounds on time and charge when departing nodes
        for i in 0..n {
            let curr_id = nodes[i].node_id;
            let min_energy_to_charge =
                min_energy_consumed_after_node[i] - self.instance.max_q;
            latest_departure_time[i] =
                self.instance.t_max - min_travel_time_after_node[i];
            if min_energy_to_charge > 0.0 {
                latest_departure_time[i] -=
                    min_energy_to_charge / self.instance.max_slope;
            }
            min_energy_at_departure[i] =
                self.instance.get_min_energy_to_cs(curr_id);
            max_energy_at_departure[i] = self.instance.max_q;
        }

        // Destination has no min energy requirement at departure.
        // (Line 58 zeroes the entry in the original nodes_g array by node_id;
        // here we zero the entry in the bounds array by local G' index.)
        min_energy_at_departure[route_len - 1] = 0.0;

        (
            min_travel_time_after_node,
            min_energy_consumed_after_node,
            min_travel_charge_time_after_node,
            latest_departure_time,
            min_energy_at_departure,
            max_energy_at_departure,
        )
    }
}
