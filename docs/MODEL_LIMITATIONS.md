# Model Limitations

The Rocket Propulsion Simulator is an educational, one-dimensional vertical-flight
model. It is not suitable for rocket construction, engine certification, flight
safety decisions or launch operations.

## Physics assumptions

- Vertical motion only
- Flat-planet constant gravity
- Constant specific impulse
- Constant, horizontally symmetric thrust direction
- Exponential atmospheric density
- Constant drag coefficient
- No wind, pitch, roll or lateral motion
- No rocket staging
- No heat transfer or engine transient model

## Numerical assumptions

Euler is the backward-compatible default. RK4 is available with `--method rk4`.
Both methods use fixed steps and are only approximations. Reducing the time step
is necessary to estimate numerical convergence.

Ground impact and burnout currently occur on integration steps. Event times and
impact velocity should be treated as numerical estimates until event interpolation
is implemented.

## Atmospheric and uncertainty results

Density follows an exponential sea-level model and is intended for educational
scale estimates, not operational atmospheric prediction.

Monte Carlo results describe configured input uncertainty. They do not describe
numerical integration error. Report both types of uncertainty separately.
