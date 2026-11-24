"""Scientific core for fixed-step one-dimensional flight simulation."""

import math

from .atmosphere import density_at_altitude
from .models import FlightPoint, FlightResult, RocketConfig


G0 = 9.80665


def validate_config(config: RocketConfig) -> None:
    positive_fields = {
        "dry_mass_kg": config.dry_mass_kg,
        "thrust_newtons": config.thrust_newtons,
        "specific_impulse_s": config.specific_impulse_s,
        "burn_time_s": config.burn_time_s,
        "time_step_s": config.time_step_s,
    }
    for name, value in positive_fields.items():
        if not math.isfinite(value) or value <= 0:
            raise ValueError(f"{name} must be a finite number greater than zero.")
    if not math.isfinite(config.propellant_mass_kg) or config.propellant_mass_kg < 0:
        raise ValueError("propellant_mass_kg must be zero or greater.")
    if not math.isfinite(config.initial_altitude_m) or config.initial_altitude_m < 0:
        raise ValueError("initial_altitude_m must be zero or greater.")
    if not math.isfinite(config.initial_velocity_m_s):
        raise ValueError("initial_velocity_m_s must be finite.")
    if not math.isfinite(config.throttle_percent) or not 0 <= config.throttle_percent <= 100:
        raise ValueError("throttle_percent must be between 0 and 100.")
    if not math.isfinite(config.drag_coefficient) or config.drag_coefficient < 0:
        raise ValueError("drag_coefficient must be zero or greater.")
    if not math.isfinite(config.reference_area_m2) or config.reference_area_m2 < 0:
        raise ValueError("reference_area_m2 must be zero or greater.")
    if config.reference_area_m2 > 0 and (
        not math.isfinite(config.air_density_kg_m3)
        or config.air_density_kg_m3 < 0
    ):
        raise ValueError("air_density_kg_m3 must be zero or greater.")
    if not math.isfinite(config.scale_height_m) or config.scale_height_m <= 0:
        raise ValueError("scale_height_m must be greater than zero.")


def simulate(
    config: RocketConfig,
    max_flight_time_s: float = 600.0,
    method: str = "euler",
) -> FlightResult:
    """Simulate vertical flight using fixed-step semi-implicit Euler."""
    if method not in {"euler", "rk4"}:
        raise ValueError("method must be 'euler' or 'rk4'.")
    if method == "rk4":
        from .numerics.rk4 import simulate_rk4
        return simulate_rk4(config, max_flight_time_s)
    validate_config(config)
    if (
        not math.isfinite(max_flight_time_s)
        or max_flight_time_s <= 0
    ):
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
    initial_drag = 0.0
    if velocity != 0:
        density = density_at_altitude(config, altitude)
        drag_force = (
            0.5
            * density
            * config.drag_coefficient
            * config.reference_area_m2
            * velocity
            * abs(velocity)
        )
        initial_drag = drag_force / mass
    max_acceleration = initial_thrust / mass - G0 - initial_drag
    burnout_time = 0.0
    burnout_altitude = altitude
    burnout_velocity = velocity
    burnout_recorded = False
    mass_flow = 0.0
    impact_velocity = velocity
    status = "coasting"
    points = [
        FlightPoint(
            time_s=0.0,
            altitude_m=altitude,
            velocity_m_s=velocity,
            acceleration_m_s2=0.0,
            mass_kg=mass,
            thrust_n=0.0,
            propellant_kg=propellant,
        )
    ]

    while time < max_flight_time_s:
        thrust = 0.0
        if time < config.burn_time_s and propellant > 1e-12 and throttle > 0:
            thrust = config.thrust_newtons * throttle
            mass_flow = thrust / (config.specific_impulse_s * G0)

        drag_acceleration = 0.0
        if velocity != 0:
            density = density_at_altitude(config, altitude)
            drag_force = (
                0.5
                * density
                * config.drag_coefficient
                * config.reference_area_m2
                * velocity
                * abs(velocity)
            )
            drag_acceleration = drag_force / mass

        acceleration = thrust / mass - G0 - drag_acceleration
        velocity += acceleration * config.time_step_s
        altitude += velocity * config.time_step_s
        time += config.time_step_s

        if thrust > 0:
            used = min(mass_flow * config.time_step_s, propellant)
            propellant -= used
            mass -= used
        else:
            mass = config.dry_mass_kg + propellant
        if propellant < 1e-9:
            propellant = 0.0

        apogee = max(apogee, altitude)
        max_velocity = max(max_velocity, velocity)
        max_acceleration = max(max_acceleration, acceleration)

        if thrust == 0 and not burnout_recorded:
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
                thrust_n=thrust,
                propellant_kg=propellant,
            )
        )

        if altitude <= 0 and time > config.time_step_s:
            altitude = 0.0
            impact_velocity = velocity
            status = "impact"
            points[-1] = FlightPoint(
                time_s=time,
                altitude_m=0.0,
                velocity_m_s=velocity,
                acceleration_m_s2=acceleration,
                mass_kg=mass,
                thrust_n=thrust,
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
