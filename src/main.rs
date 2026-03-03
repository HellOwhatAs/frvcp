use std::process;

use clap::Parser;
use frvcpy::solver::Solver;

#[derive(Parser)]
#[command(name = "frvcpy", about = "Solves a Fixed Route Vehicle Charging Problem")]
struct Cli {
    /// Filename for the frvcpy-compatible problem instance (JSON)
    #[arg(short, long)]
    instance: String,

    /// Comma-separated list of node IDs defining the route
    #[arg(short, long)]
    route: String,

    /// The initial energy of the EV traveling the route
    #[arg(short, long)]
    qinit: f64,

    /// Allow multiple CSs to be inserted between stops (default)
    #[arg(long, default_value_t = true)]
    multi: bool,

    /// Allow only one CS to be inserted between stops
    #[arg(long)]
    one: bool,
}

fn main() {
    let cli = Cli::parse();

    let multi_insert = if cli.one { false } else { cli.multi };

    let route: Vec<usize> = cli
        .route
        .split(',')
        .map(|s| s.trim().parse().expect("Route must be comma-separated integers"))
        .collect();

    let mut frvcp_solver = Solver::from_file(&cli.instance, route, cli.qinit, multi_insert);
    let (duration, feas_route) = frvcp_solver.solve();

    println!("Duration: {:.4}", duration);
    match feas_route {
        Some(route) => println!("Energy-feasible route:\n{:?}", route),
        None => println!("No feasible route found."),
    }

    process::exit(0);
}
