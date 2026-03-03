//! Implementation of the labeling algorithm for the FRVCP from Froger et al. (2019).

use crate::core::{FrvcpInstance, LabelHeap, Node, NodeHeap, NodeLabel, NodeType, OrdF64};

/// Compare two keys lexicographically.
fn key_lt(a: (f64, f64), b: (f64, f64)) -> bool {
    (OrdF64(a.0), OrdF64(a.1)) < (OrdF64(b.0), OrdF64(b.1))
}

/// The Froger, et al. (2019) labeling algorithm to solve the FRVCP.
pub struct FrvcpAlgo {
    pub instance: FrvcpInstance,
    pub init_soc: f64,
    pub nodes_gpr: Vec<Node>,
    pub n_nodes: usize,
    pub adjacency_list: Vec<Vec<usize>>,
    pub node_local_id_dep: usize,
    pub node_local_id_arr: usize,
    pub max_slope: f64,
    pub min_energy_consumed_after_node: Vec<f64>,
    pub min_travel_time_after_node: Vec<f64>,
    pub min_travel_charge_time_after_node: Vec<f64>,
    pub max_energy_at_departure: Vec<f64>,
    pub latest_departure_time: Vec<f64>,
    pub min_energy_at_departure: Vec<f64>,
    // Algorithm state
    in_heap: Vec<bool>,
    key: Vec<Option<(f64, f64)>>,
    unset_labels: Vec<LabelHeap>,
    set_labels: Vec<Vec<NodeLabel>>,
    heap: NodeHeap,
}

impl FrvcpAlgo {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        instance: FrvcpInstance,
        init_soc: f64,
        nodes_gpr: Vec<Node>,
        adjacency_list: Vec<Vec<usize>>,
        node_local_id_dep: usize,
        node_local_id_arr: usize,
        max_slope: f64,
        min_energy_consumed_after_node: Vec<f64>,
        min_travel_time_after_node: Vec<f64>,
        min_travel_charge_time_after_node: Vec<f64>,
        max_energy_at_departure: Vec<f64>,
        latest_departure_time: Vec<f64>,
        min_energy_at_departure: Vec<f64>,
    ) -> Self {
        let n_nodes = nodes_gpr.len();
        FrvcpAlgo {
            instance,
            init_soc,
            n_nodes,
            nodes_gpr,
            adjacency_list,
            node_local_id_dep,
            node_local_id_arr,
            max_slope,
            min_energy_consumed_after_node,
            min_travel_time_after_node,
            min_travel_charge_time_after_node,
            max_energy_at_departure,
            latest_departure_time,
            min_energy_at_departure,
            in_heap: Vec::new(),
            key: Vec::new(),
            unset_labels: Vec::new(),
            set_labels: Vec::new(),
            heap: NodeHeap::new(),
        }
    }

    /// Execute the labeling algorithm.
    pub fn run_algo(&mut self) {
        self.in_heap = vec![false; self.n_nodes];
        self.key = vec![None; self.n_nodes];
        self.unset_labels = (0..self.n_nodes).map(|_| LabelHeap::new()).collect();
        self.set_labels = vec![Vec::new(); self.n_nodes];
        self.heap = NodeHeap::new();

        // Build first label
        let first_label = self.build_first_label();
        let first_key = first_label.get_key();
        self.heap.add_task(self.node_local_id_dep, first_key);
        self.key[self.node_local_id_dep] = Some(first_key);
        self.in_heap[self.node_local_id_dep] = true;
        self.unset_labels[self.node_local_id_dep].add_task(first_label);

        while !self.heap.is_empty() {
            let min_node_local_id = self.heap.pop_task().unwrap();
            let min_node_id = self.nodes_gpr[min_node_local_id].node_id;
            self.in_heap[min_node_local_id] = false;
            self.key[min_node_local_id] = None;

            // Return smallest unset label associated with current node
            let label_to_set = self.unset_labels[min_node_local_id].pop_task().unwrap();

            // Compute supporting points of label
            let label_to_set = self.compute_supporting_points(label_to_set);

            // Check if label dominated by previously set label
            if self.is_dominated(&label_to_set, min_node_local_id) {
                self.insert_new_node_in_heap(min_node_local_id);
                continue;
            }

            // If label is a CS and hasn't been "switched" yet, build the list of labels
            if self.nodes_gpr[min_node_local_id].node_type == NodeType::ChargingStation
                && label_to_set.last_visited_cs != Some(min_node_local_id)
            {
                let new_labels = self.build_label_list(&label_to_set);
                for new_label in new_labels {
                    self.unset_labels[min_node_local_id].add_task(new_label);
                }
                self.insert_new_node_in_heap(min_node_local_id);
                continue;
            }

            // If current node is the destination node
            if min_node_local_id == self.node_local_id_arr {
                self.set_labels[min_node_local_id].push(label_to_set);
                break;
            }

            // Extend current label to adjacent nodes
            let n_adj_nodes = self.adjacency_list[min_node_local_id].len();
            for k in 0..n_adj_nodes {
                let next_node_local_id = self.adjacency_list[min_node_local_id][k];
                let next_node_id = self.nodes_gpr[next_node_local_id].node_id;

                if !self.can_be_extended_to(&label_to_set, next_node_local_id) {
                    continue;
                }

                let new_label = self.relax_arc(
                    &label_to_set,
                    next_node_local_id,
                    self.instance.energy_matrix[min_node_id][next_node_id],
                    self.instance.process_times[next_node_id]
                        + self.instance.time_matrix[min_node_id][next_node_id],
                );

                if let Some(new_label) = new_label {
                    let new_key = new_label.get_key();
                    self.unset_labels[next_node_local_id].add_task(new_label);

                    if self.in_heap[next_node_local_id] {
                        let current_key = self.key[next_node_local_id].unwrap();
                        if key_lt(new_key, current_key) {
                            self.heap.add_task(next_node_local_id, new_key);
                            self.key[next_node_local_id] = Some(new_key);
                        }
                    } else {
                        self.heap.add_task(next_node_local_id, new_key);
                        self.in_heap[next_node_local_id] = true;
                        self.key[next_node_local_id] = Some(new_key);
                    }
                }
            }

            // Mark current label as set
            self.set_labels[min_node_local_id].push(label_to_set);

            // Add min node to the heap
            self.insert_new_node_in_heap(min_node_local_id);
        }
    }

    /// Can we extend curr_label to node given by next_node_local_id?
    fn can_be_extended_to(
        &self,
        curr_label: &NodeLabel,
        next_node_local_id: usize,
    ) -> bool {
        let next_node = &self.nodes_gpr[next_node_local_id];

        if next_node.node_type == NodeType::ChargingStation {
            let parents = curr_label.get_path_from_last_customer();
            let mut cs_nodes: Vec<&Node> = Vec::new();
            let mut check = false;

            for i in (0..parents.len()).rev() {
                let parent_node = &self.nodes_gpr[parents[i]];
                if parent_node.node_id == next_node.node_id {
                    check = true;
                    break;
                }
                if parent_node.node_type == NodeType::ChargingStation {
                    cs_nodes.push(parent_node);
                } else {
                    break;
                }
            }

            if check {
                for cs_parent_node in &cs_nodes {
                    if self.instance.is_cs_faster(cs_parent_node, next_node) {
                        return true;
                    }
                }
                return false;
            }

            return true;
        }

        true
    }

    /// Returns the label built by the extension of curr_label to the
    /// node given by next_node_local_id.
    fn relax_arc(
        &self,
        curr_label: &NodeLabel,
        next_node_local_id: usize,
        energy_arc: f64,
        time_arc: f64,
    ) -> Option<NodeLabel> {
        let max_q = self.instance.max_q;

        let new_e_consumed_since_last_cs;
        let new_soc_at_last_cs;
        let new_last_visited;

        if self.nodes_gpr[curr_label.node_id_for_label].node_type == NodeType::ChargingStation {
            new_e_consumed_since_last_cs = energy_arc;
            new_last_visited = Some(curr_label.node_id_for_label);
            new_soc_at_last_cs = curr_label.get_first_supp_pt_soc();
        } else {
            new_e_consumed_since_last_cs =
                curr_label.energy_consumed_since_last_cs + energy_arc;
            new_last_visited = curr_label.last_visited_cs;
            new_soc_at_last_cs = curr_label.soc_arr_to_last_cs;
        }

        // Compute max soc reachable at next node
        let max_soc_at_next = curr_label.get_last_supp_pt_soc() - energy_arc;
        if max_soc_at_next < self.min_energy_at_departure[next_node_local_id] {
            return None;
        }

        // Compute min soc reachable or needed at next node
        let min_soc_at_next = f64::max(
            self.min_energy_at_departure[next_node_local_id],
            new_soc_at_last_cs - new_e_consumed_since_last_cs,
        );

        // Determine if we need to charge at the last visited CS
        let e_to_charge_at_last_cs = f64::max(
            0.0,
            self.min_energy_at_departure[next_node_local_id] + new_e_consumed_since_last_cs
                - new_soc_at_last_cs,
        );
        let trip_time = curr_label.trip_time + time_arc;
        let mut min_time = trip_time;

        if e_to_charge_at_last_cs > 0.0 {
            if new_last_visited.is_none() {
                return None;
            }
            if new_soc_at_last_cs + e_to_charge_at_last_cs > max_q {
                return None;
            }
            let cs_node = &self.nodes_gpr[new_last_visited.unwrap()];
            let charging_time =
                self.instance
                    .get_charging_time(cs_node, new_soc_at_last_cs, e_to_charge_at_last_cs);
            min_time += charging_time;
        }

        if min_time > self.latest_departure_time[next_node_local_id] {
            return None;
        }

        // Compute key time
        let key_time =
            if min_soc_at_next > self.min_energy_consumed_after_node[next_node_local_id] {
                min_time + self.min_travel_time_after_node[next_node_local_id]
            } else {
                min_time + self.min_travel_charge_time_after_node[next_node_local_id]
                    - min_soc_at_next / self.max_slope
            };

        Some(NodeLabel::new(
            next_node_local_id,
            key_time,
            trip_time,
            new_last_visited,
            new_soc_at_last_cs,
            new_e_consumed_since_last_cs,
            curr_label.supporting_pts.clone(),
            curr_label.slope.clone(),
            time_arc,
            energy_arc,
            Some(Box::new(curr_label.clone())),
            curr_label.y_intercept.clone(),
        ))
    }

    /// Builds list of labels to extend from the curr_label. Creates one new
    /// label for each supporting point of the current SOC function.
    fn build_label_list(&self, curr_label: &NodeLabel) -> Vec<NodeLabel> {
        let curr_local_id = curr_label.node_id_for_label;
        let curr_node = &self.nodes_gpr[curr_local_id];

        if curr_node.node_type != NodeType::ChargingStation {
            return vec![curr_label.clone()];
        }

        let mut label_list = Vec::new();

        let cs_supp_pts = self.instance.get_cf_breakpoints(curr_node);
        let cs_slope = self.instance.get_slopes(curr_node);
        let cs_n_pts = cs_supp_pts[0].len();

        let lbl_supp_pts = &curr_label.supporting_pts;
        let lbl_n_pts = curr_label.get_num_supp_pts();

        for k in 0..lbl_n_pts {
            let trip_time = lbl_supp_pts[0][k];
            let soc_at_cs = lbl_supp_pts[1][k];

            // Don't switch if it leads to a time-infeasible path
            if trip_time > self.latest_departure_time[curr_local_id] {
                continue;
            }

            // Don't switch if energy is sufficient to finish the route
            if soc_at_cs > self.max_energy_at_departure[curr_local_id] {
                continue;
            }

            // Switch only if the new slope is better than the current
            if k < lbl_n_pts - 1 {
                let curr_slope = curr_label.slope.as_ref().unwrap()[k];
                let new_slope =
                    self.instance
                        .get_slope_at_soc(curr_node, f64::max(0.0, soc_at_cs));
                if curr_slope >= new_slope {
                    continue;
                }
            }

            // Don't switch if soc when departing from last CS was sufficient
            if let Some(last_cs) = curr_label.last_visited_cs {
                if soc_at_cs + curr_label.energy_consumed_since_last_cs
                    > self.max_energy_at_departure[last_cs]
                {
                    continue;
                }
            }

            // Compute first breakpoint of charging function above the SOC
            if soc_at_cs < 0.0 {
                panic!("Supporting point has negative energy");
            }
            let mut first_k = 0;
            while first_k < cs_n_pts && cs_supp_pts[1][first_k] <= soc_at_cs {
                first_k += 1;
            }

            let e_to_end = self.min_energy_consumed_after_node[curr_local_id];
            let n_pts_new = cs_n_pts - first_k + 1;
            let mut supp_pts_new = [vec![0.0; n_pts_new], vec![0.0; n_pts_new]];
            let slope_new = if n_pts_new > 1 {
                Some(
                    (first_k..cs_n_pts)
                        .map(|l| cs_slope[l - 1])
                        .collect::<Vec<f64>>(),
                )
            } else {
                None
            };

            supp_pts_new[0][0] = trip_time;
            supp_pts_new[1][0] = soc_at_cs;

            let shift_time = self
                .instance
                .get_time_to_charge_from_zero(curr_node, soc_at_cs);
            for supp_pt_idx in first_k..cs_n_pts {
                supp_pts_new[0][supp_pt_idx - first_k + 1] =
                    trip_time + cs_supp_pts[0][supp_pt_idx] - shift_time;
                supp_pts_new[1][supp_pt_idx - first_k + 1] = cs_supp_pts[1][supp_pt_idx];
            }

            // Compute key
            let key_time = if soc_at_cs > e_to_end {
                trip_time + self.min_travel_time_after_node[curr_local_id]
            } else {
                trip_time + self.min_travel_charge_time_after_node[curr_local_id]
                    - soc_at_cs / self.max_slope
            };

            label_list.push(NodeLabel::new(
                curr_local_id,
                key_time,
                trip_time,
                Some(curr_local_id),
                curr_label.soc_arr_to_last_cs,
                curr_label.energy_consumed_since_last_cs,
                supp_pts_new,
                slope_new,
                0.0,
                0.0,
                curr_label.parent.clone(),
                None,
            ));
        }

        label_list
    }

    /// If current node has unset labels, insert this node in the heap with a new key.
    fn insert_new_node_in_heap(&mut self, local_node_id: usize) {
        if !self.unset_labels[local_node_id].is_empty() {
            let new_key = self.unset_labels[local_node_id].peek().unwrap().get_key();
            self.heap.add_task(local_node_id, new_key);
            self.in_heap[local_node_id] = true;
            self.key[local_node_id] = Some(new_key);
        }
    }

    /// Is label dominated by any of the set labels at min_node_local_id?
    fn is_dominated(&self, label: &NodeLabel, min_node_local_id: usize) -> bool {
        for other in &self.set_labels[min_node_local_id] {
            if other.dominates(label) {
                return true;
            }
        }
        false
    }

    /// Provides a new label identical to the argument, but with the supporting
    /// points, slopes, and y-intercepts updated.
    fn compute_supporting_points(&self, label: NodeLabel) -> NodeLabel {
        if label.time_last_arc == 0.0 && label.energy_last_arc == 0.0 {
            return label;
        }

        let NodeLabel {
            node_id_for_label: local_id,
            key_time,
            trip_time,
            last_visited_cs,
            soc_arr_to_last_cs,
            energy_consumed_since_last_cs,
            supporting_pts: supp_pts,
            slope: label_slope,
            time_last_arc,
            energy_last_arc,
            parent,
            y_intercept: label_y_intercept,
            n_pts,
        } = label;

        let mut new_supp_pts_temp = [vec![0.0; n_pts], vec![0.0; n_pts]];

        let mut last_k_opt: Option<usize> = None;
        let mut compute_first_point = false;

        for k in 0..n_pts {
            new_supp_pts_temp[0][k] = supp_pts[0][k] + time_last_arc;
            new_supp_pts_temp[1][k] = supp_pts[1][k] - energy_last_arc;

            if last_k_opt.is_none()
                && (new_supp_pts_temp[1][k] > self.max_energy_at_departure[local_id]
                    || new_supp_pts_temp[0][k] > self.latest_departure_time[local_id])
            {
                last_k_opt = Some(k);
                break;
            }
        }

        let mut last_k = last_k_opt.unwrap_or(n_pts);

        let mut first_k: usize = 0;
        while first_k < last_k
            && new_supp_pts_temp[1][first_k] < self.min_energy_at_departure[local_id]
        {
            first_k += 1;
        }

        if first_k == last_k {
            if first_k == 0 || first_k == n_pts {
                panic!("No feasible supporting points for label");
            }
            compute_first_point = true;
        }

        if first_k != 0 {
            compute_first_point = true;
            first_k -= 1;
        }
        if last_k != n_pts {
            last_k += 1;
        }

        let mut slope_now: Option<Vec<f64>> = None;
        let mut y_int_now: Option<Vec<f64>> = None;
        let new_supp_pts;

        if !compute_first_point {
            new_supp_pts = [
                new_supp_pts_temp[0][..last_k].to_vec(),
                new_supp_pts_temp[1][..last_k].to_vec(),
            ];

            if let Some(ref s) = label_slope {
                let n_pts_rmd = n_pts - last_k;
                let n_seg_initial = s.len();
                let n_seg_now = n_seg_initial.saturating_sub(n_pts_rmd);

                if n_seg_now > 0 {
                    slope_now = Some(s[..n_seg_now].to_vec());
                    let yi = label_y_intercept.as_ref().unwrap();
                    y_int_now = Some(
                        (0..n_seg_now)
                            .map(|k| yi[k] - energy_last_arc - time_last_arc * s[k])
                            .collect(),
                    );
                }
            }
        } else {
            let n_pts_now = last_k - first_k;
            let mut new_supp_pts_inner = [vec![0.0; n_pts_now], vec![0.0; n_pts_now]];

            let slope_ref = label_slope.as_ref().unwrap();
            let yi_ref = label_y_intercept.as_ref().unwrap();
            let slope_val = slope_ref[first_k];
            let y_int =
                yi_ref[first_k] - energy_last_arc - time_last_arc * slope_ref[first_k];
            let computed_trip_time =
                (self.min_energy_at_departure[local_id] - y_int) / slope_val;

            new_supp_pts_inner[0][0] = computed_trip_time;
            new_supp_pts_inner[1][0] = self.min_energy_at_departure[local_id];

            for k in (first_k + 1)..last_k {
                new_supp_pts_inner[0][k - first_k] = new_supp_pts_temp[0][k];
                new_supp_pts_inner[1][k - first_k] = new_supp_pts_temp[1][k];
            }

            if n_pts_now > 1 {
                slope_now = Some(
                    (first_k..last_k - 1)
                        .map(|k| slope_ref[k])
                        .collect(),
                );
                y_int_now = Some(
                    (first_k..last_k - 1)
                        .map(|k| yi_ref[k] - energy_last_arc - time_last_arc * slope_ref[k])
                        .collect(),
                );
            }

            new_supp_pts = new_supp_pts_inner;
        }

        NodeLabel::new(
            local_id,
            key_time,
            trip_time,
            last_visited_cs,
            soc_arr_to_last_cs,
            energy_consumed_since_last_cs,
            new_supp_pts,
            slope_now,
            0.0,
            0.0,
            parent,
            y_int_now,
        )
    }

    /// Constructs the initial label to kick off the labeling algorithm.
    fn build_first_label(&self) -> NodeLabel {
        let supp_pts = [vec![0.0], vec![self.init_soc]];
        let energy_to_end = self.min_energy_consumed_after_node[self.node_local_id_dep];
        let key_time = if self.init_soc >= energy_to_end {
            self.min_travel_time_after_node[self.node_local_id_dep]
        } else {
            self.min_travel_charge_time_after_node[self.node_local_id_dep]
        };
        NodeLabel::new(
            self.node_local_id_dep,
            key_time,
            0.0,
            None,
            self.init_soc,
            0.0,
            supp_pts,
            None,
            0.0,
            0.0,
            None,
            None,
        )
    }

    /// Returns the optimal route found by the algorithm if one exists; None otherwise.
    /// If returned, the optimal route is a list of 2-tuples whose first entry is the node
    /// ID and second entry is the amount to charge at that node.
    pub fn get_optimized_route(&self) -> Option<Vec<(usize, Option<f64>)>> {
        if self.set_labels[self.node_local_id_arr].is_empty() {
            return None;
        }

        let label = &self.set_labels[self.node_local_id_arr][0];
        let path = label.get_path();
        let charge_amts = label.get_charging_amounts();
        let mut route = Vec::new();
        let mut charging_index = 0;
        for &k in &path {
            let node = &self.nodes_gpr[k];
            if node.node_type == NodeType::ChargingStation {
                route.insert(0, (node.node_id, Some(charge_amts[charging_index])));
                charging_index += 1;
            } else {
                route.insert(0, (node.node_id, None));
            }
        }
        Some(route)
    }

    /// Returns the key time for the first set label at the destination node
    /// if a label exists; infinity otherwise.
    pub fn get_objective_value(&self) -> f64 {
        if !self.set_labels[self.node_local_id_arr].is_empty() {
            self.set_labels[self.node_local_id_arr][0].key_time
        } else {
            f64::INFINITY
        }
    }

    /// Returns True if the destination node has at least one set label;
    /// False otherwise.
    pub fn solution_found(&self) -> bool {
        !self.set_labels[self.node_local_id_arr].is_empty()
    }
}
