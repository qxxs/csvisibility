# Rocket Propulsion Simulator

A dependency-free command-line simulator for vertical single-stage rocket flight.

## Run

Requires Python 3.10 or newer.

```bash
python3 rocket_sim.py
```

Run interactively:

```bash
python3 rocket_sim.py --interactive
```

Run with command-line inputs:

```bash
python3 rocket_sim.py \
  --dry-mass 50 \
  --propellant 100 \
  --thrust 20000 \
  --isp 250 \
  --burn-time 12 \
  --throttle 100
```

Compare propulsion levels:

```bash
python3 rocket_sim.py --compare
```

## Model

The model performs a one-dimensional vertical simulation with a fixed time step.

- Thrust: `maximum thrust × throttle percentage`
- Mass flow: `thrust / (specific impulse × standard gravity)`
- Gravity: `9.80665 m/s²`
- Drag: optional quadratic atmospheric drag using `Cd`, reference area, sea-level density, and scale height
- Integration: semi-implicit Euler
- Ground impact: simulation stops when altitude reaches 0 m after launch

The values are educational estimates, not a substitute for engine design or launch engineering tools.

## Test

```bash
python3 -m unittest discover -s tests -v
```
