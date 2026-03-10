//! Core data structures for the FRVCP (Fixed Route Vehicle Charging Problem) solver.

use std::cmp::Ordering;
use std::cmp::Reverse;
use std::collections::{BinaryHeap, HashMap};

/// Default maximum route duration when not specified in the instance.
const DEFAULT_T_MAX: f64 = f64::INFINITY;
use std::fmt;
use std::fs;

use serde::{Deserialize, Serialize};

// ───────────────────── OrdF64 ─────────────────────

/// Wrapper around `f64` that implements `Ord` (treats NaN as equal).
#[derive(Clone, Copy, Debug)]
pub struct OrdF64(pub f64);

impl PartialEq for OrdF64 {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl Eq for OrdF64 {}

impl PartialOrd for OrdF64 {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for OrdF64 {
    fn cmp(&self, other: &Self) -> Ordering {
        self.0.partial_cmp(&other.0).unwrap_or(Ordering::Equal)
    }
}

// ───────────────────── NodeType ─────────────────────

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum NodeType {
    Depot = 0,
    Customer = 1,
    ChargingStation = 2,
}

impl fmt::Display for NodeType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            NodeType::Depot => write!(f, "NodeType.DEPOT"),
            NodeType::Customer => write!(f, "NodeType.CUSTOMER"),
            NodeType::ChargingStation => write!(f, "NodeType.CHARGING_STATION"),
        }
    }
}

// ───────────────────── Node ─────────────────────

#[derive(Clone, Debug)]
pub struct Node {
    pub node_id: usize,
    pub name: String,
    pub node_type: NodeType,
}

impl fmt::Display for Node {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "({}; {})", self.node_id, self.node_type)
    }
}

// ───────────────────── NodeHeap ─────────────────────

/// Priority queue for node IDs with updatable keys.
/// Mirrors Python's `PseudoFibonacciHeap` when used with `HeapElement` tasks.
pub struct NodeHeap {
    pq: BinaryHeap<Reverse<(OrdF64, OrdF64, u64, usize)>>,
    entry_finder: HashMap<usize, u64>,
    counter: u64,
}

impl NodeHeap {
    pub fn new() -> Self {
        NodeHeap {
            pq: BinaryHeap::new(),
            entry_finder: HashMap::new(),
            counter: 0,
        }
    }

    /// Remove stale entries from the top of the heap.
    fn clean_top(&mut self) {
        while let Some(&Reverse((_, _, count, task_id))) = self.pq.peek() {
            match self.entry_finder.get(&task_id) {
                Some(&expected) if expected == count => break,
                _ => {
                    self.pq.pop();
                }
            }
        }
    }

    pub fn is_empty(&mut self) -> bool {
        self.clean_top();
        self.pq.is_empty()
    }

    /// Add a new task or update the priority of an existing task.
    pub fn add_task(&mut self, task_id: usize, priority: (f64, f64)) {
        self.counter += 1;
        let count = self.counter;
        self.entry_finder.insert(task_id, count);
        self.pq.push(Reverse((
            OrdF64(priority.0),
            OrdF64(priority.1),
            count,
            task_id,
        )));
    }

    /// Mark an existing task as removed.
    pub fn remove_task(&mut self, task_id: usize) {
        self.entry_finder.remove(&task_id);
    }

    /// Remove and return the lowest priority task.
    pub fn pop_task(&mut self) -> Option<usize> {
        self.clean_top();
        if let Some(Reverse((_, _, _, task_id))) = self.pq.pop() {
            self.entry_finder.remove(&task_id);
            Some(task_id)
        } else {
            None
        }
    }

    /// Returns the lowest priority task without removing it.
    pub fn peek(&mut self) -> Option<usize> {
        self.clean_top();
        self.pq.peek().map(|Reverse((_, _, _, task_id))| *task_id)
    }
}

// ───────────────────── NodeLabel ─────────────────────

/// A label for the labeling algorithm of the FRVCP solver.
#[derive(Clone, Debug)]
pub struct NodeLabel {
    pub node_id_for_label: usize,
    pub key_time: f64,
    pub trip_time: f64,
    pub last_visited_cs: Option<usize>,
    pub soc_arr_to_last_cs: f64,
    pub energy_consumed_since_last_cs: f64,
    pub supporting_pts: [Vec<f64>; 2],
    pub slope: Option<Vec<f64>>,
    pub time_last_arc: f64,
    pub energy_last_arc: f64,
    pub parent: Option<Box<NodeLabel>>,
    pub y_intercept: Option<Vec<f64>>,
    pub n_pts: usize,
}

impl NodeLabel {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        node_id_for_label: usize,
        key_time: f64,
        trip_time: f64,
        last_visited_cs: Option<usize>,
        soc_arr_to_last_cs: f64,
        energy_consumed_since_last_cs: f64,
        supporting_pts: [Vec<f64>; 2],
        slope: Option<Vec<f64>>,
        time_last_arc: f64,
        energy_last_arc: f64,
        parent: Option<Box<NodeLabel>>,
        y_intercept: Option<Vec<f64>>,
    ) -> Self {
        let y_intercept = match y_intercept {
            Some(yi) => Some(yi),
            None => match &slope {
                Some(s) => Some(
                    (0..s.len())
                        .map(|b| supporting_pts[1][b] - s[b] * supporting_pts[0][b])
                        .collect(),
                ),
                None => None,
            },
        };
        let n_pts = supporting_pts[0].len();
        NodeLabel {
            node_id_for_label,
            key_time,
            trip_time,
            last_visited_cs,
            soc_arr_to_last_cs,
            energy_consumed_since_last_cs,
            supporting_pts,
            slope,
            time_last_arc,
            energy_last_arc,
            parent,
            y_intercept,
            n_pts,
        }
    }

    /// Returns the key associated with the label.
    pub fn get_key(&self) -> (f64, f64) {
        let first_soc = self.supporting_pts[1][0];
        (
            self.key_time,
            if first_soc == 0.0 {
                f64::INFINITY
            } else {
                1.0 / first_soc
            },
        )
    }

    /// Does this label dominate `other`?
    pub fn dominates(&self, other: &NodeLabel) -> bool {
        if self.trip_time > other.trip_time {
            return false;
        }
        if *self.supporting_pts[1].last().unwrap() < *other.supporting_pts[1].last().unwrap() {
            return false;
        }
        for k in 0..self.n_pts {
            let soc_other = other.get_soc(self.supporting_pts[0][k]);
            if self.supporting_pts[1][k] < soc_other {
                return false;
            }
        }
        for k in 0..other.n_pts {
            let soc = self.get_soc(other.supporting_pts[0][k]);
            if soc < other.supporting_pts[1][k] {
                return false;
            }
        }
        true
    }

    /// Given a time, returns the corresponding SOC from the label's supporting points.
    pub fn get_soc(&self, time: f64) -> f64 {
        if time < self.trip_time {
            return f64::NEG_INFINITY;
        }
        if time >= *self.supporting_pts[0].last().unwrap() {
            return *self.supporting_pts[1].last().unwrap();
        }
        let mut low: usize = 0;
        let mut high: usize = self.n_pts - 1;
        while low + 1 < high {
            let mid = (low + high) / 2;
            if self.supporting_pts[0][mid] < time {
                low = mid;
            } else {
                high = mid;
            }
        }
        let slope = self.slope.as_ref().unwrap();
        let y_intercept = self.y_intercept.as_ref().unwrap();
        slope[low] * time + y_intercept[low]
    }

    /// Get the min charge over all supporting points.
    pub fn get_first_supp_pt_soc(&self) -> f64 {
        self.supporting_pts[1][0]
    }

    /// Get the max charge over all supporting points.
    pub fn get_last_supp_pt_soc(&self) -> f64 {
        *self.supporting_pts[1].last().unwrap()
    }

    /// Returns the number of supporting points.
    pub fn get_num_supp_pts(&self) -> usize {
        self.supporting_pts[0].len()
    }

    /// Get the path from this label backwards to the origin.
    pub fn get_path(&self) -> Vec<usize> {
        let mut path = Vec::new();
        let mut curr: &NodeLabel = self;
        loop {
            path.push(curr.node_id_for_label);
            match &curr.parent {
                Some(p) => curr = p.as_ref(),
                None => break,
            }
        }
        path
    }

    /// Provides a list of the node IDs that the vehicle has visited since the
    /// last time it either a) visited a customer, b) visited a depot, or
    /// c) visited the node at which it currently resides.
    pub fn get_path_from_last_customer(&self) -> Vec<usize> {
        if self.last_visited_cs.is_none() {
            return vec![];
        }
        let mut path = Vec::new();
        let mut curr = self;

        loop {
            let next_prev_cs = curr.last_visited_cs;
            path.push(curr.node_id_for_label);
            match &curr.parent {
                None => break,
                Some(parent) => {
                    curr = parent.as_ref();
                    let curr_prev_cs = curr.last_visited_cs;
                    if curr_prev_cs.is_none() || curr_prev_cs == next_prev_cs {
                        break;
                    }
                }
            }
        }
        path
    }

    /// For the path traveled to get to this label, the amounts that it has
    /// recharged along the way.
    pub fn get_charging_amounts(&self) -> Vec<f64> {
        if self.last_visited_cs.is_none() {
            return vec![];
        }
        let mut charge_amts = vec![
            self.energy_consumed_since_last_cs + self.get_first_supp_pt_soc()
                - self.soc_arr_to_last_cs,
        ];

        let mut curr_label: &NodeLabel = self;
        'outer: loop {
            let s_last_vis_cs = curr_label.last_visited_cs;
            let mut charge_reqd;
            loop {
                charge_reqd = curr_label.energy_consumed_since_last_cs
                    + curr_label.get_first_supp_pt_soc()
                    - curr_label.soc_arr_to_last_cs;
                match &curr_label.parent {
                    Some(parent) => {
                        curr_label = parent.as_ref();
                        if curr_label.last_visited_cs != s_last_vis_cs {
                            break;
                        }
                    }
                    None => break 'outer,
                }
            }
            if curr_label.last_visited_cs.is_none() {
                break;
            }
            charge_amts.push(charge_reqd);
        }
        charge_amts
    }

    /// Returns -1 if self is a "better" label than other, 1 if vice versa, 0 if the same.
    pub fn compare_to(&self, other: &NodeLabel) -> i32 {
        if self.key_time < other.key_time {
            return -1;
        }
        if self.key_time > other.key_time {
            return 1;
        }
        let diff = other.supporting_pts[1][0] - self.supporting_pts[1][0];
        if diff > 0.0 {
            return 1;
        }
        if diff < 0.0 {
            return -1;
        }
        0
    }
}

impl PartialEq for NodeLabel {
    fn eq(&self, other: &Self) -> bool {
        self.compare_to(other) == 0
    }
}

impl Eq for NodeLabel {}

impl PartialOrd for NodeLabel {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for NodeLabel {
    fn cmp(&self, other: &Self) -> Ordering {
        match self.key_time.partial_cmp(&other.key_time) {
            Some(Ordering::Less) => Ordering::Less,
            Some(Ordering::Greater) => Ordering::Greater,
            _ => {
                let self_soc = self.supporting_pts[1][0];
                let other_soc = other.supporting_pts[1][0];
                // Higher SOC is "less" (better priority)
                other_soc
                    .partial_cmp(&self_soc)
                    .unwrap_or(Ordering::Equal)
            }
        }
    }
}

impl fmt::Display for NodeLabel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "---- Label for node {}", self.node_id_for_label)?;
        writeln!(f, "keyTime = {}", self.key_time)?;
        writeln!(f, "tripTime = {}", self.trip_time)?;
        writeln!(f, "timeLastArc = {}", self.time_last_arc)?;
        writeln!(f, "energyLastArc = {}", self.energy_last_arc)?;
        writeln!(f, "lastVisitedCS = {:?}", self.last_visited_cs)?;
        writeln!(f, "socAtArrLastCS = {}", self.soc_arr_to_last_cs)?;
        writeln!(
            f,
            "energyConsumedSinceLastCS = {}",
            self.energy_consumed_since_last_cs
        )?;
        writeln!(f, "Supporting points")?;
        writeln!(f, "{:?}", self.supporting_pts[0])?;
        writeln!(f, "{:?}", self.supporting_pts[1])?;
        if let Some(ref slope) = self.slope {
            writeln!(f, "Slope")?;
            writeln!(f, "{:?}", slope)?;
            writeln!(f, "Intercept")?;
            writeln!(f, "{:?}", self.y_intercept.as_ref().unwrap())?;
        }
        writeln!(f, "Path")?;
        write!(f, "{:?}", self.get_path())
    }
}

// ───────────────────── LabelHeap ─────────────────────

/// Min-heap of `NodeLabel` values, sorted by label ordering.
pub struct LabelHeap {
    pq: BinaryHeap<Reverse<NodeLabel>>,
}

impl LabelHeap {
    pub fn new() -> Self {
        LabelHeap {
            pq: BinaryHeap::new(),
        }
    }

    pub fn add_task(&mut self, label: NodeLabel) {
        self.pq.push(Reverse(label));
    }

    pub fn pop_task(&mut self) -> Option<NodeLabel> {
        self.pq.pop().map(|Reverse(l)| l)
    }

    pub fn peek(&self) -> Option<&NodeLabel> {
        self.pq.peek().map(|Reverse(l)| l)
    }

    pub fn is_empty(&self) -> bool {
        self.pq.is_empty()
    }
}

// ───────────────────── Serde raw types ─────────────────────

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RawCsDetail {
    pub node_id: usize,
    pub cs_type: usize,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RawBreakpoint {
    pub cs_type: usize,
    pub time: Vec<f64>,
    pub charge: Vec<f64>,
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct RawInstance {
    pub max_q: f64,
    pub t_max: Option<f64>,
    pub css: Vec<RawCsDetail>,
    pub process_times: Option<Vec<f64>>,
    pub breakpoints_by_type: Vec<RawBreakpoint>,
    pub energy_matrix: Vec<Vec<f64>>,
    pub time_matrix: Vec<Vec<f64>>,
}

// ───────────────────── CsDetail / CsBkptInfo ─────────────────────

#[derive(Clone, Debug)]
pub struct CsDetail {
    pub node_id: usize,
    pub cs_type: usize,
}

#[derive(Clone, Debug)]
pub struct CsBkptInfo {
    pub cs_type: usize,
    pub time: Vec<f64>,
    pub charge: Vec<f64>,
}

// ───────────────────── FrvcpInstance ─────────────────────

/// An frvcp-compliant problem instance.
#[derive(Clone)]
pub struct FrvcpInstance {
    pub energy_matrix: Vec<Vec<f64>>,
    pub time_matrix: Vec<Vec<f64>>,
    pub process_times: Vec<f64>,
    pub n_nodes_g: usize,
    pub max_q: f64,
    pub t_max: f64,
    pub cs_bkpt_info: HashMap<usize, CsBkptInfo>,
    pub cs_details: Vec<CsDetail>,
    pub n_cs: usize,
    pub type_to_supp_pts: HashMap<usize, [Vec<f64>; 2]>,
    pub type_to_slopes: HashMap<usize, Vec<f64>>,
    pub type_to_yints: HashMap<usize, Vec<f64>>,
    pub max_slope: f64,
    pub cs_id_to_type: HashMap<usize, usize>,
    pub nodes_g: Vec<Node>,
}

impl FrvcpInstance {
    /// Load an instance from a JSON file.
    pub fn from_file(filename: &str) -> Self {
        let data = fs::read_to_string(filename).expect("Unable to read instance file");
        let raw: RawInstance = serde_json::from_str(&data).expect("Invalid instance JSON");
        Self::new(raw)
    }

    /// Build an instance from deserialized raw data.
    pub fn new(instance: RawInstance) -> Self {
        let energy_matrix = instance.energy_matrix;
        let time_matrix = instance.time_matrix;
        let n = energy_matrix.len();

        let process_times = instance.process_times.unwrap_or_else(|| vec![0.0; n]);
        let n_nodes_g = process_times.len();
        let max_q = instance.max_q;
        let t_max = instance.t_max.unwrap_or(DEFAULT_T_MAX);

        let cs_bkpt_info: HashMap<usize, CsBkptInfo> = instance
            .breakpoints_by_type
            .iter()
            .map(|bp| {
                (
                    bp.cs_type,
                    CsBkptInfo {
                        cs_type: bp.cs_type,
                        time: bp.time.clone(),
                        charge: bp.charge.clone(),
                    },
                )
            })
            .collect();

        let cs_details: Vec<CsDetail> = instance
            .css
            .iter()
            .map(|cs| CsDetail {
                node_id: cs.node_id,
                cs_type: cs.cs_type,
            })
            .collect();

        let n_cs = cs_details.len();

        let type_to_supp_pts: HashMap<usize, [Vec<f64>; 2]> = cs_bkpt_info
            .iter()
            .map(|(&cs_type, info)| (cs_type, [info.time.clone(), info.charge.clone()]))
            .collect();

        let type_to_slopes: HashMap<usize, Vec<f64>> = type_to_supp_pts
            .iter()
            .map(|(&cs_type, arr)| {
                let slopes: Vec<f64> = (0..arr[0].len() - 1)
                    .map(|i| (arr[1][i + 1] - arr[1][i]) / (arr[0][i + 1] - arr[0][i]))
                    .collect();
                (cs_type, slopes)
            })
            .collect();

        let type_to_yints: HashMap<usize, Vec<f64>> = type_to_supp_pts
            .iter()
            .map(|(&cs_type, arr)| {
                let yints: Vec<f64> = (0..arr[0].len() - 1)
                    .map(|i| arr[1][i] - type_to_slopes[&cs_type][i] * arr[0][i])
                    .collect();
                (cs_type, yints)
            })
            .collect();

        let max_slope = type_to_slopes
            .values()
            .map(|s| s[0])
            .fold(f64::NEG_INFINITY, f64::max);

        let cs_id_to_type: HashMap<usize, usize> =
            cs_details.iter().map(|cs| (cs.node_id, cs.cs_type)).collect();

        let nodes_g: Vec<Node> = (0..energy_matrix.len())
            .map(|i| {
                let node_type = if cs_id_to_type.contains_key(&i) {
                    NodeType::ChargingStation
                } else {
                    NodeType::Customer
                };
                Node {
                    node_id: i,
                    name: format!("node-{}", i),
                    node_type,
                }
            })
            .collect();

        FrvcpInstance {
            energy_matrix,
            time_matrix,
            process_times,
            n_nodes_g,
            max_q,
            t_max,
            cs_bkpt_info,
            cs_details,
            n_cs,
            type_to_supp_pts,
            type_to_slopes,
            type_to_yints,
            max_slope,
            cs_id_to_type,
            nodes_g,
        }
    }

    /// Build an instance from deserialized raw data, optionally checking
    /// that the triangle inequality holds for both the energy and time matrices.
    pub fn new_checked(instance: RawInstance, check_tri: bool) -> Self {
        let inst = Self::new(instance);
        if check_tri {
            if !inst.triangle_inequality_holds(&inst.energy_matrix) {
                panic!("The triangle inequality does not hold for the instance's energy_matrix.");
            }
            if !inst.triangle_inequality_holds(&inst.time_matrix) {
                panic!("The triangle inequality does not hold for the instance's time_matrix.");
            }
        }
        inst
    }

    /// Verifies that the triangle inequality holds for the given square matrix.
    pub fn triangle_inequality_holds(&self, matrix: &[Vec<f64>]) -> bool {
        let n = matrix.len();
        assert!(
            matrix.iter().all(|row| row.len() == n),
            "Matrix must be square."
        );
        for i in 0..n {
            for j in 0..n {
                for k in 0..n {
                    if matrix[i][k] + matrix[k][j] < matrix[i][j] {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Calculates the energy needed to get from `node_id` to the nearest CS.
    pub fn get_min_energy_to_cs(&self, node_id: usize) -> f64 {
        if self.cs_id_to_type.contains_key(&node_id) {
            0.0
        } else {
            self.cs_id_to_type
                .keys()
                .map(|&cs_id| self.energy_matrix[node_id][cs_id])
                .fold(f64::INFINITY, f64::min)
        }
    }

    /// Returns the list of CS nodes.
    pub fn get_cs_nodes(&self) -> Vec<Node> {
        self.nodes_g
            .iter()
            .filter(|n| n.node_type == NodeType::ChargingStation)
            .cloned()
            .collect()
    }

    /// True if `node1` is a faster CS type than `node2`.
    pub fn is_cs_faster(&self, node1: &Node, node2: &Node) -> bool {
        self.type_to_slopes[&self.cs_id_to_type[&node1.node_id]][0]
            > self.type_to_slopes[&self.cs_id_to_type[&node2.node_id]][0]
    }

    /// Get the breakpoints in the charging function of `cs_node`'s CS type.
    pub fn get_cf_breakpoints(&self, cs_node: &Node) -> &[Vec<f64>; 2] {
        &self.type_to_supp_pts[&self.cs_id_to_type[&cs_node.node_id]]
    }

    /// Returns all slopes for the charging function of `node`'s CS type.
    pub fn get_slopes(&self, node: &Node) -> &Vec<f64> {
        &self.type_to_slopes[&self.cs_id_to_type[&node.node_id]]
    }

    /// Returns the slope at a specific SOC level for `node`'s CS type.
    pub fn get_slope_at_soc(&self, node: &Node, soc: f64) -> f64 {
        let cs_type = self.cs_id_to_type[&node.node_id];
        let idx = self.get_cf_segment_idx(cs_type, soc, 1);
        self.type_to_slopes[&cs_type][idx]
    }

    /// How long does it take to charge `acquired_energy` starting from `starting_energy` at `node`.
    pub fn get_charging_time(
        &self,
        node: &Node,
        starting_energy: f64,
        acquired_energy: f64,
    ) -> f64 {
        self.get_time_to_charge_from_zero(node, starting_energy + acquired_energy)
            - self.get_time_to_charge_from_zero(node, starting_energy)
    }

    /// How long does it take to charge to level `soc` at `node` (starting from zero).
    pub fn get_time_to_charge_from_zero(&self, node: &Node, soc: f64) -> f64 {
        let cs_type = self.cs_id_to_type[&node.node_id];
        let seg_idx = self.get_cf_segment_idx(cs_type, soc, 1);
        (soc - self.type_to_yints[&cs_type][seg_idx]) / self.type_to_slopes[&cs_type][seg_idx]
    }

    /// Returns the segment index for a given value on the given axis (0 = time, 1 = charge).
    pub fn get_cf_segment_idx(&self, cs_type: usize, value: f64, axis: usize) -> usize {
        let pts = &self.type_to_supp_pts[&cs_type][axis];
        let mut idx = 0;
        loop {
            if pts[idx] <= value && value < pts[idx + 1] {
                return idx;
            }
            idx += 1;
            if idx == pts.len() - 1 {
                if value == pts[idx] {
                    return idx - 1;
                }
                panic!(
                    "Request out of bounds for segment index. Value passed: {}",
                    value
                );
            }
        }
    }
}
