use std::path::Path;
use std::process;

use clap::Parser;
use frvcp::solver::Solver;

#[derive(Parser)]
#[command(name = "frvcp", about = "Solves a Fixed Route Vehicle Charging Problem")]
struct Cli {
    /// Filename for the frvcp-compatible problem instance (JSON or XML)
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

    /// Check that the triangle inequality holds for the passed instance
    #[arg(short = 'c', long = "check-tri")]
    check_tri: bool,

    /// Write output to file (specify name with -o NAME)
    #[arg(short = 'w', long = "write")]
    write: bool,

    /// Name of file to which to write solution (with -w flag)
    #[arg(short = 'o', long = "output")]
    output: Option<String>,
}

fn main() {
    let mut cli = Cli::parse();

    let multi_insert = if cli.one { false } else { cli.multi };

    let route: Vec<usize> = cli
        .route
        .split(',')
        .map(|s| s.trim().parse().expect("Route must be comma-separated integers"))
        .collect();

    // If an output file was specified, then writing results to file
    if cli.output.is_some() {
        cli.write = true;
    }

    // If passed an XML file, translate it first
    let instance = if cli.instance.ends_with(".xml") {
        eprintln!("INFO: Passed instance is an XML file. \
                   Assuming it is a VRP-REP instance and attempting to translate it...");
        let raw = frvcp::translator::translate(&cli.instance, true);
        eprintln!("INFO: Instance translated.");
        raw
    } else {
        let data = std::fs::read_to_string(&cli.instance)
            .expect("Unable to read instance file");
        serde_json::from_str(&data).expect("Invalid instance JSON")
    };

    let mut frvcp_solver = Solver::new_checked(
        instance,
        route.clone(),
        cli.qinit,
        multi_insert,
        cli.check_tri,
    );
    let (duration, feas_route) = frvcp_solver.solve();

    if cli.write {
        let instance_wo_ext = Path::new(&cli.instance)
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_default();

        let output_filename = cli.output.unwrap_or_else(|| {
            // Use first 4 stops of route for default filename (matches Python behavior)
            let route_substr: String = route.iter().take(4).map(|s| s.to_string()).collect::<Vec<_>>().join("");
            let q_substr = cli.qinit as i64;
            format!("{}-r{}-q{}.xml", instance_wo_ext, route_substr, q_substr)
        });

        frvcp_solver.write_solution(&output_filename, &instance_wo_ext);
        eprintln!("INFO: Solution written to file: {}", output_filename);
    }

    println!("Duration: {:.4}", duration);
    match feas_route {
        Some(route) => println!("Energy-feasible route:\n{:?}", route),
        None => println!("No feasible route found."),
    }

    process::exit(0);
}
