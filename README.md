# Rocket Propulsion Simulator

[![Tests](https://github.com/anthroptic/rocket-propulsion-simulator/actions/workflows/tests.yml/badge.svg)](https://github.com/anthroptic/rocket-propulsion-simulator/actions/workflows/tests.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/python-3.10+-blue.svg)](https://www.python.org/downloads/)

A dependency-free command-line simulator for vertical single-stage rocket flight.

Enter rocket mass, thrust, propellant, specific impulse, burn time, throttle, drag,
and initial flight conditions. The simulator calculates thrust-to-weight behavior,
acceleration, propellant use, burnout, apogee, flight time, and ground impact.
Interactive mode can replay the entire flight in the terminal.

## Features

- Numeric CLI inputs and interactive prompts
- Fixed-step one-dimensional flight simulation
- Throttle comparison from 25% through 100%
- Optional atmospheric drag
- Terminal altitude chart
- Animated terminal replay with live thrust and velocity
- JSON output for scripts and external tools
- No third-party runtime dependencies
- Classical RK4 integration with Euler compatibility
- JSON rocket configuration load/save
- CSV trajectory export and detailed JSON reports
- SQLite simulation history
- Sensitivity and reproducible Monte Carlo analysis
- Professional terminal menu

## Quick start

### Requirements

- Python 3.10 or newer

### Run without installation

```bash
git clone https://github.com/anthroptic/rocket-propulsion-simulator.git
cd rocket-propulsion-simulator
python3 rocket_sim.py --interactive
```

### Install as a command

```bash
python3 -m pip install .
rocket-sim --interactive
```

## Usage

Run the default flight:

```bash
rocket-sim
```

Replay a custom flight in the terminal:

```bash
rocket-sim --animate --thrust 25000 --throttle 75 --playback-speed 4 --fps 30
```

Run with detailed command-line inputs:

```bash
rocket-sim \
  --dry-mass 50 \
  --propellant 100 \
  --thrust 20000 \
  --isp 250 \
  --burn-time 12 \
  --throttle 100 \
  --drag-coefficient 0.4 \
  --area 0.5
```

Compare propulsion levels:

```bash
rocket-sim --compare
```

Export machine-readable results:

```bash
rocket-sim --json
```

Use RK4 integration:

```bash
rocket-sim --method rk4 --step 0.01
```

Export a trajectory and detailed report:

```bash
rocket-sim --csv exports/flight.csv --output-json exports/flight.json
```

Save and restore a rocket configuration:

```bash
rocket-sim --save-config configs/example.json
rocket-sim --config configs/example.json
```

Run sensitivity or seeded Monte Carlo analysis:

```bash
rocket-sim --sensitivity thrust_newtons
rocket-sim --monte-carlo 100 --seed 42
```

Save and view simulation history:

```bash
rocket-sim --save-history
rocket-sim --history
```

Open the terminal menu:

```bash
rocket-sim --menu
```

See every option:

```bash
rocket-sim --help
```

## Physics model

The model performs a one-dimensional vertical simulation with a fixed time step.

- Thrust: `maximum thrust × throttle percentage`
- Mass flow: `thrust / (specific impulse × standard gravity)`
- Gravity: `9.80665 m/s²`
- Drag: optional quadratic atmospheric drag using `Cd`, reference area, sea-level density, and scale height
- Integration: semi-implicit Euler
- Ground impact: simulation stops when altitude reaches 0 m after launch

The values are educational estimates, not a substitute for engine design or launch engineering tools.

## Project structure

```text
rocket-propulsion-simulator/
├── rocket_sim.py                 Compatibility CLI and terminal interface
├── rocket_simulator/
│   ├── models.py                 Flight and rocket dataclasses
│   ├── simulation.py             Euler simulation and validation
│   ├── atmosphere.py             Atmospheric density model
│   ├── analysis.py               Sensitivity and Monte Carlo analysis
│   ├── reporting.py              JSON and CSV reporting
│   ├── storage.py                SQLite simulation history
│   └── numerics/rk4.py           Classical fourth-order RK4 integrator
├── configs/                      Example JSON rocket configurations
├── tests/
├── pyproject.toml
├── LICENSE
└── README.md
```

## Development

Run the test suite:

```bash
python3 -m unittest discover -s tests -v
```

Compile all Python files:

```bash
python3 -m compileall -q rocket_sim.py tests
```

## License

MIT License. See [LICENSE](LICENSE).
