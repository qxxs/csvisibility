"""Classical fourth-order Runge-Kutta flight integrator."""

import math

from rocket_simulator.atmosphere import density_at_altitude
from rocket_simulator.models import FlightPoint, FlightResult, RocketConfig
from rocket_simulator.simulation import G0, validate_config


def simulate_rk4(
    config: RocketConfig,
    max_flight_time_s: float = 600.0,
) -> FlightResult:
    """Integrate vertical flight with fixed-step RK4."""
    validate_config(config)
    if not math.isfinite(max_flight_time_s) or max_flight_time_s <= 0:
        raise ValueError("max_flight_time_s must be a finite number greater than zero.")

    throttle = config.throttle_percent / 100.0
    mass = config.dry_mass_kg + config.propellant_mass_kg
    propellant = config.propellant_mass_kg
    altitude = config.initial_altitude_m
    velocity = config.initial_velocity_m_s
    time = 0.0

    apogee = altitude
    max_velocity = velocity
    initial_thrust = (
        config.thrust_newtons * throttle
        if propellant > 1e-12 and config.burn_time_s > 0
        else 0.0
    )
    initial_drag = (
        0.5
        * density_at_altitude(config, altitude)
        * config.drag_coefficient
        * config.reference_area_m2
        * velocity
        * abs(velocity)
        / mass
    )
    max_acceleration = initial_thrust / mass - G0 - initial_drag
    impact_velocity = velocity
    status = "coasting"
    burnout_time = 0.0
    burnout_altitude = altitude
    burnout_velocity = velocity
    burnout_recorded = False
    points = [
        FlightPoint(
            time_s=0.0,
            altitude_m=altitude,
            velocity_m_s=velocity,
            acceleration_m_s2=max_acceleration,
            mass_kg=mass,
            thrust_n=initial_thrust,
            propellant_kg=propellant,
        )
    ]

    def derivatives(y_altitude: float, y_velocity: float, y_mass: float, y_time: float):
        if y_time < config.burn_time_s and propellant > 1e-12 and throttle > 0:
            thrust = config.thrust_newtons * throttle
            mass_flow = thrust / (config.specific_impulse_s * G0)
        else:
            thrust = 0.0
            mass_flow = 0.0

        drag_acceleration = 0.0
        if y_velocity != 0:
            density = density_at_altitude(config, y_altitude)
            drag_acceleration = (
                0.5
                * density
                * config.drag_coefficient
                * config.reference_area_m2
                * y_velocity
                * abs(y_velocity)
                / y_mass
            )
        acceleration = thrust / y_mass - G0 - drag_acceleration
        propellant_derivative = -mass_flow
        mass_derivative = propellant_derivative
        return y_velocity, acceleration, mass_derivative, thrust

    dt = config.time_step_s
    while time < max_flight_time_s:
        current_thrust = (
            config.thrust_newtons * throttle
            if time < config.burn_time_s and propellant > 1e-12 and throttle > 0
            else 0.0
        )

        k1_v, k1_a, k1_m, _ = derivatives(altitude, velocity, mass, time)
        k1_p = k1_m

        k2_v, k2_a, k2_m, _ = derivatives(
            altitude + 0.5 * dt * k1_v,
            velocity + 0.5 * dt * k1_a,
            mass + 0.5 * dt * k1_m,
            time + 0.5 * dt,
        )
        k2_p = k2_m

        k3_v, k3_a, k3_m, _ = derivatives(
            altitude + 0.5 * dt * k2_v,
            velocity + 0.5 * dt * k2_a,
            mass + 0.5 * dt * k2_m,
            time + 0.5 * dt,
        )
        k3_p = k3_m

        k4_v, k4_a, k4_m, _ = derivatives(
            altitude + dt * k3_v,
            velocity + dt * k3_a,
            mass + dt * k3_m,
            time + dt,
        )
        k4_p = k4_m

        altitude += dt / 6 * (k1_v + 2 * k2_v + 2 * k3_v + k4_v)
        velocity += dt / 6 * (k1_a + 2 * k2_a + 2 * k3_a + k4_a)
        mass += dt / 6 * (k1_m + 2 * k2_m + 2 * k3_m + k4_m)
        propellant += dt / 6 * (k1_p + 2 * k2_p + 2 * k3_p + k4_p)

        propellant = max(0.0, propellant)
        mass = config.dry_mass_kg + propellant
        time += dt
        acceleration = derivatives(altitude, velocity, mass, time)[1]

        apogee = max(apogee, altitude)
        max_velocity = max(max_velocity, velocity)
        max_acceleration = max(max_acceleration, acceleration)

        if current_thrust == 0 and not burnout_recorded:
            burnout_time = time
            burnout_altitude = altitude
            burnout_velocity = velocity
            burnout_recorded = True

        points.append(
            FlightPoint(
                time_s=time,
                altitude_m=altitude,
                velocity_m_s=velocity,
                acceleration_m_s2=acceleration,
                mass_kg=mass,
                thrust_n=current_thrust,
                propellant_kg=propellant,
            )
        )

        if altitude <= 0 and time > dt:
            altitude = 0.0
            impact_velocity = velocity
            status = "impact"
            points[-1] = FlightPoint(
                time_s=time,
                altitude_m=0.0,
                velocity_m_s=velocity,
                acceleration_m_s2=acceleration,
                mass_kg=mass,
                thrust_n=current_thrust,
                propellant_kg=propellant,
            )
            break
    else:
        status = "time_limit"
        impact_velocity = velocity

    if not burnout_recorded:
        burnout_time = time
        burnout_altitude = altitude
        burnout_velocity = velocity

    return FlightResult(
        status=status,
        flight_time_s=time,
        apogee_m=apogee,
        max_velocity_m_s=max_velocity,
        max_acceleration_m_s2=max_acceleration,
        impact_velocity_m_s=impact_velocity,
        final_mass_kg=mass,
        burnout_time_s=burnout_time,
        burnout_altitude_m=burnout_altitude,
        burnout_velocity_m_s=burnout_velocity,
        propellant_used_kg=config.propellant_mass_kg - propellant,
        points=tuple(points),
    )
