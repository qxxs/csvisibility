"""Rocket propulsion trajectory simulator.

A dependency-free, one-dimensional vertical-flight model for educational use.
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import asdict, dataclass
from typing import Iterable


G0 = 9.80665


@dataclass(frozen=True)
class RocketConfig:
    dry_mass_kg: float
    propellant_mass_kg: float
    thrust_newtons: float
    specific_impulse_s: float
    burn_time_s: float
    initial_altitude_m: float = 0.0
    initial_velocity_m_s: float = 0.0
    drag_coefficient: float = 0.0
    reference_area_m2: float = 0.0
    air_density_kg_m3: float = 1.225
    scale_height_m: float = 8500.0
    throttle_percent: float = 100.0
    time_step_s: float = 0.1


@dataclass(frozen=True)
class FlightPoint:
    time_s: float
    altitude_m: float
    velocity_m_s: float
    acceleration_m_s2: float
    mass_kg: float
    thrust_n: float
    propellant_kg: float


@dataclass(frozen=True)
class FlightResult:
    status: str
    flight_time_s: float
    apogee_m: float
    max_velocity_m_s: float
    max_acceleration_m_s2: float
    impact_velocity_m_s: float
    final_mass_kg: float
    burnout_time_s: float
    burnout_altitude_m: float
    burnout_velocity_m_s: float
    propellant_used_kg: float
    points: tuple[FlightPoint, ...]


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
    if not 0 <= config.throttle_percent <= 100:
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


def _density_at_altitude(config: RocketConfig, altitude_m: float) -> float:
    if config.drag_coefficient == 0 or config.reference_area_m2 == 0:
        return 0.0
    return config.air_density_kg_m3 * math.exp(
        -max(0.0, altitude_m) / config.scale_height_m
    )


def simulate(config: RocketConfig, max_flight_time_s: float = 600.0) -> FlightResult:
    """Simulate vertical flight using a fixed-step semi-implicit Euler method."""
    validate_config(config)
    if max_flight_time_s <= 0:
        raise ValueError("max_flight_time_s must be greater than zero.")

    throttle = config.throttle_percent / 100.0
    mass = config.dry_mass_kg + config.propellant_mass_kg
    propellant = config.propellant_mass_kg
    altitude = config.initial_altitude_m
    velocity = config.initial_velocity_m_s
    time = 0.0
    apogee = altitude
    max_velocity = velocity
    max_acceleration = 0.0
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
            density = _density_at_altitude(config, altitude)
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


def print_ascii_chart(result: FlightResult, width: int = 68, height: int = 18) -> None:
    points = result.points
    if not points:
        return
    maximum = max(point.altitude_m for point in points)
    if maximum <= 0:
        maximum = 1.0
    selected = [
        points[min(len(points) - 1, round(i * (len(points) - 1) / (width - 1)))]
        for i in range(width)
    ]

    print(f"\nAltitude vs. time (max {maximum:,.1f} m)")
    for row in range(height, -1, -1):
        threshold = maximum * row / height
        line = []
        for point in selected:
            line.append("█" if point.altitude_m >= threshold else " ")
        label = f"{threshold:8,.0f} m │"
        print(label + "".join(line))
    print("         " + "└" + "─" * width)
    print("          0" + " " * max(0, width - 12) + f"{result.flight_time_s:.1f} s")


def print_result(
    config: RocketConfig,
    result: FlightResult,
    chart_width: int = 68,
    chart_height: int = 18,
) -> None:
    print("\n" + "=" * 58)
    print(f"ROCKET SIMULATION — {config.throttle_percent:g}% THROTTLE")
    print("=" * 58)
    print(f"  Status             {result.status}")
    print(f"  Flight time        {result.flight_time_s:,.2f} s")
    print(f"  Apogee             {result.apogee_m:,.2f} m")
    print(f"  Max velocity       {result.max_velocity_m_s:,.2f} m/s")
    print(f"  Max acceleration   {result.max_acceleration_m_s2:,.2f} m/s²")
    print(f"  Burnout time       {result.burnout_time_s:,.2f} s")
    print(f"  Burnout altitude   {result.burnout_altitude_m:,.2f} m")
    print(f"  Burnout velocity   {result.burnout_velocity_m_s:,.2f} m/s")
    print(f"  Propellant used    {result.propellant_used_kg:,.2f} kg")
    print(f"  Final mass         {result.final_mass_kg:,.2f} kg")
    if result.status == "impact":
        print(f"  Impact velocity    {result.impact_velocity_m_s:,.2f} m/s")
    print_ascii_chart(result, chart_width, chart_height)


def prompt_float(prompt: str, default: float) -> float:
    while True:
        raw = input(f"{prompt} [{default:g}]: ").strip()
        if not raw:
            return default
        try:
            value = float(raw)
        except ValueError:
            print("Please enter a number.")
            continue
        if math.isfinite(value):
            return value
        print("Please enter a finite number.")


def interactive_config() -> RocketConfig:
    print("\nRocket Propulsion Simulator")
    print("Enter values or press Enter to accept the suggested default.\n")
    dry_mass = prompt_float("Dry mass (kg)", 50.0)
    propellant = prompt_float("Propellant mass (kg)", 100.0)
    thrust = prompt_float("Maximum thrust (N)", 20_000.0)
    isp = prompt_float("Specific impulse (s)", 250.0)
    burn_time = prompt_float("Maximum burn time (s)", 12.0)
    altitude = prompt_float("Initial altitude (m)", 0.0)
    velocity = prompt_float("Initial vertical velocity (m/s)", 0.0)
    drag_coefficient = prompt_float("Drag coefficient Cd", 0.0)
    area = prompt_float("Reference area (m²)", 0.0)
    throttle = prompt_float("Throttle level (%)", 100.0)
    step = prompt_float("Simulation time step (s)", 0.1)

    config = RocketConfig(
        dry_mass_kg=dry_mass,
        propellant_mass_kg=propellant,
        thrust_newtons=thrust,
        specific_impulse_s=isp,
        burn_time_s=burn_time,
        initial_altitude_m=altitude,
        initial_velocity_m_s=velocity,
        drag_coefficient=drag_coefficient,
        reference_area_m2=area,
        throttle_percent=throttle,
        time_step_s=step,
    )
    try:
        validate_config(config)
    except ValueError as error:
        print(f"\nInvalid configuration: {error}", file=sys.stderr)
        raise
    return config


def base_config_from_args(args: argparse.Namespace) -> RocketConfig:
    return RocketConfig(
        dry_mass_kg=args.dry_mass,
        propellant_mass_kg=args.propellant,
        thrust_newtons=args.thrust,
        specific_impulse_s=args.isp,
        burn_time_s=args.burn_time,
        initial_altitude_m=args.altitude,
        initial_velocity_m_s=args.velocity,
        drag_coefficient=args.drag_coefficient,
        reference_area_m2=args.area,
        air_density_kg_m3=args.air_density,
        scale_height_m=args.scale_height,
        throttle_percent=args.throttle,
        time_step_s=args.step,
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Simulate a vertical single-stage rocket flight."
    )
    parser.add_argument("--interactive", action="store_true", help="Prompt for each input.")
    parser.add_argument("--compare", action="store_true", help="Compare 25%%, 50%%, 75%%, and 100%% throttle.")
    parser.add_argument("--dry-mass", type=float, default=50.0, help="Dry mass in kg.")
    parser.add_argument("--propellant", type=float, default=100.0, help="Propellant mass in kg.")
    parser.add_argument("--thrust", type=float, default=20_000.0, help="Maximum thrust in N.")
    parser.add_argument("--isp", type=float, default=250.0, help="Specific impulse in s.")
    parser.add_argument("--burn-time", type=float, default=12.0, help="Maximum burn time in s.")
    parser.add_argument("--altitude", type=float, default=0.0, help="Initial altitude in m.")
    parser.add_argument("--velocity", type=float, default=0.0, help="Initial velocity in m/s.")
    parser.add_argument("--drag-coefficient", type=float, default=0.0, help="Dimensionless drag coefficient.")
    parser.add_argument("--area", type=float, default=0.0, help="Reference area in m².")
    parser.add_argument("--air-density", type=float, default=1.225, help="Sea-level density in kg/m³.")
    parser.add_argument("--scale-height", type=float, default=8500.0, help="Atmospheric scale height in m.")
    parser.add_argument("--throttle", type=float, default=100.0, help="Throttle percentage.")
    parser.add_argument("--step", type=float, default=0.1, help="Simulation step in s.")
    parser.add_argument("--max-time", type=float, default=600.0, help="Maximum flight time in s.")
    parser.add_argument("--chart-width", type=int, default=68, help="ASCII chart width.")
    parser.add_argument("--chart-height", type=int, default=18, help="ASCII chart height.")
    parser.add_argument("--json", action="store_true", help="Print JSON instead of human-readable output.")
    return parser


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        config = (
            interactive_config()
            if args.interactive
            else base_config_from_args(args)
        )
        validate_config(config)
        if args.compare:
            throttle_levels = (25.0, 50.0, 75.0, 100.0)
            rows = []
            for throttle in throttle_levels:
                level_config = RocketConfig(
                    **{**asdict(config), "throttle_percent": throttle}
                )
                result = simulate(level_config, args.max_time)
                if not args.json:
                    print_result(
                        level_config,
                        result,
                        args.chart_width,
                        args.chart_height,
                    )
                rows.append({
                    "throttle_percent": throttle,
                    "flight_time_s": result.flight_time_s,
                    "apogee_m": result.apogee_m,
                    "max_velocity_m_s": result.max_velocity_m_s,
                    "burnout_altitude_m": result.burnout_altitude_m,
                    "status": result.status,
                })
            if args.json:
                import json
                print(json.dumps(rows, indent=2))
            return 0

        result = simulate(config, args.max_time)
        if args.json:
            import json
            print(json.dumps({
                "config": asdict(config),
                "result": {
                    key: value
                    for key, value in asdict(result).items()
                    if key != "points"
                },
            }, indent=2))
        else:
            print_result(config, result, args.chart_width, args.chart_height)
        return 0
    except (ValueError, KeyboardInterrupt) as error:
        if isinstance(error, KeyboardInterrupt):
            print("\nSimulation cancelled.")
        else:
            parser.error(str(error))
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
