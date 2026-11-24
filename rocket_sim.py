"""Rocket propulsion trajectory simulator.

A dependency-free, one-dimensional vertical-flight model for educational use.
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import asdict
from typing import Iterable

from rocket_simulator.models import FlightPoint, FlightResult, RocketConfig
from rocket_simulator.simulation import G0, simulate, validate_config
from rocket_simulator.atmosphere import density_at_altitude
from rocket_simulator.analysis import sensitivity_analysis, monte_carlo
from rocket_simulator.reporting import (
    export_result_json,
    export_trajectory_csv,
    load_rocket_config,
    save_rocket_config,
)
from rocket_simulator.storage import SimulationHistory


def _density_at_altitude(config: RocketConfig, altitude_m: float) -> float:
    """Compatibility alias for the original private atmosphere helper."""
    return density_at_altitude(config, altitude_m)


def print_ascii_chart(result: FlightResult, width: int = 68, height: int = 18) -> None:
    if not isinstance(width, int) or isinstance(width, bool) or width < 2:
        raise ValueError("chart width must be an integer greater than or equal to 2.")
    if not isinstance(height, int) or isinstance(height, bool) or height < 1:
        raise ValueError("chart height must be an integer greater than or equal to 1.")
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


def select_animation_frames(
    result: FlightResult,
    target_fps: int = 30,
    playback_speed: float = 1.0,
    max_frames: int = 900,
) -> list[FlightPoint]:
    """Select evenly spaced flight points for a bounded terminal replay."""
    if not result.points:
        return []
    if playback_speed <= 0:
        playback_speed = 1.0

    estimated_frames = max(
        2,
        round(result.flight_time_s * target_fps * playback_speed),
    )
    frame_count = min(estimated_frames, max_frames)
    if frame_count >= len(result.points):
        return list(result.points)

    return [
        result.points[
            min(
                len(result.points) - 1,
                round(index * (len(result.points) - 1) / (frame_count - 1)),
            )
        ]
        for index in range(frame_count)
    ]


def animate_flight(
    config: RocketConfig,
    result: FlightResult,
    playback_speed: float = 1.0,
    target_fps: int = 30,
) -> None:
    """Replay a trajectory with a visible rocket and live propulsion readouts."""
    frames = select_animation_frames(result, target_fps, playback_speed)
    if not frames:
        return

    maximum = max(result.apogee_m, config.initial_altitude_m, 1.0)
    chart_height = 16
    frame_delay = 1.0 / target_fps
    print("\033[2J\033[H")

    for frame in frames:
        position = int(round(frame.altitude_m / maximum * (chart_height - 1)))
        position = max(0, min(chart_height - 1, position))
        lines = []
        for row in range(chart_height - 1, -1, -1):
            if row == position:
                flame = ""
                if frame.thrust_n > 0:
                    flame_level = min(3, max(1, round(config.throttle_percent / 34)))
                    flame = "  " + ("!" * flame_level)
                lines.append(f"{row * maximum / (chart_height - 1):9,.0f} m │        /\\{flame}")
                lines.append("          │       <🚀>")
            else:
                lines.append(f"{row * maximum / (chart_height - 1):9,.0f} m │")

        thrust_percent = (
            frame.thrust_n / config.thrust_newtons * 100
            if config.thrust_newtons
            else 0
        )
        state = "POWERED" if frame.thrust_n > 0 else "COASTING"
        print("\033[2J\033[H", end="")
        print(f"ROCKET FLIGHT  |  {state}  |  {playback_speed:g}x playback")
        print("\n".join(lines))
        print("          └────────────────────────────────")
        print(
            f"Time {frame.time_s:7.1f} s   "
            f"Altitude {frame.altitude_m:10,.1f} m   "
            f"Velocity {frame.velocity_m_s:9,.1f} m/s"
        )
        print(
            f"Thrust {frame.thrust_n:10,.0f} N ({thrust_percent:5.1f}%)   "
            f"Accel {frame.acceleration_m_s2:8,.2f} m/s²   "
            f"Propellant {frame.propellant_kg:7,.2f} kg"
        )
        time.sleep(frame_delay)

    print("\nFlight replay complete.")


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


def interactive_config() -> tuple[RocketConfig, float]:
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
    replay_speed = prompt_float("Replay speed multiplier", 1.0)
    if replay_speed < 0.1 or replay_speed > 20:
        raise ValueError("Replay speed multiplier must be between 0.1 and 20.")

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
    return config, replay_speed


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
    parser.add_argument("--animate", action="store_true", help="Replay the flight in the terminal.")
    parser.add_argument("--no-animation", action="store_true", help="Skip the terminal flight replay.")
    parser.add_argument("--playback-speed", type=float, default=1.0, help="Replay speed multiplier (0.1 to 20).")
    parser.add_argument("--fps", type=int, default=30, help="Terminal replay frames per second.")
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
    parser.add_argument("--method", choices=("euler", "rk4"), default="euler", help="Numerical integration method.")
    parser.add_argument("--output-json", metavar="PATH", help="Export a detailed JSON report.")
    parser.add_argument("--csv", metavar="PATH", help="Export the trajectory as CSV.")
    parser.add_argument("--save-history", action="store_true", help="Save this simulation to SQLite history.")
    parser.add_argument("--history", action="store_true", help="List recent simulation history.")
    parser.add_argument("--history-db", default="data/simulation_history.sqlite3", help="SQLite history database path.")
    parser.add_argument("--sensitivity", metavar="FIELD", help="Run one-at-a-time sensitivity analysis.")
    parser.add_argument("--monte-carlo", type=int, metavar="SAMPLES", help="Run Monte Carlo uncertainty analysis.")
    parser.add_argument("--seed", type=int, default=0, help="Random seed for Monte Carlo analysis.")
    parser.add_argument("--menu", action="store_true", help="Open the professional interactive main menu.")
    parser.add_argument("--config", metavar="PATH", help="Load a rocket configuration from JSON.")
    parser.add_argument("--save-config", metavar="PATH", help="Save the active rocket configuration to JSON.")
    return parser


def menu_main() -> int:
    while True:
        print("\nROCKET PROPULSION SIMULATOR")
        print("[1] Launch Simulation")
        print("[2] Compare Throttle Levels")
        print("[3] Sensitivity Analysis")
        print("[4] Monte Carlo Analysis")
        print("[5] Simulation History")
        print("[0] Exit")
        choice = input("Select option: ").strip()
        if choice == "0":
            return 0
        if choice == "1":
            try:
                config, replay_speed = interactive_config()
                validate_config(config)
                result = simulate(config)
                animate_flight(config, result, replay_speed, 30)
                print_result(config, result)
            except (ValueError, KeyboardInterrupt) as error:
                print(f"\nError: {error}", file=sys.stderr)
        elif choice == "2":
            try:
                config, _ = interactive_config()
                for throttle in (25.0, 50.0, 75.0, 100.0):
                    level_config = RocketConfig(
                        **{**asdict(config), "throttle_percent": throttle}
                    )
                    result = simulate(level_config)
                    print(
                        f"{throttle:5.0f}%  apogee {result.apogee_m:12,.1f} m  "
                        f"max velocity {result.max_velocity_m_s:10,.1f} m/s"
                    )
            except (ValueError, KeyboardInterrupt) as error:
                print(f"\nError: {error}", file=sys.stderr)
        elif choice in {"3", "4"}:
            print("Use the corresponding command-line options for batch analysis.")
        elif choice == "5":
            with SimulationHistory("data/simulation_history.sqlite3") as history:
                rows = history.list()
                if not rows:
                    print("No saved simulations.")
                for row in rows:
                    print(
                        f"{row['id']:>3}  {row['created_at']}  "
                        f"{row['rocket_name']}  {row['status']}  "
                        f"{row['apogee_m']:,.1f} m"
                    )
        else:
            print("Please select a displayed option.")


def main(argv: Iterable[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        if args.history:
            with SimulationHistory(args.history_db) as history:
                rows = history.list()
                if not rows:
                    print("No saved simulations.")
                else:
                    print("ID  Date                 Rocket              Method  Status      Apogee m")
                    for row in rows:
                        print(
                            f"{row['id']:>2}  {row['created_at']}  "
                            f"{row['rocket_name']:<18} {row['integration_method']:<6} "
                            f"{row['status']:<10} {row['apogee_m']:,.1f}"
                        )
            return 0
        if args.menu:
            return menu_main()
        replay_speed = args.playback_speed
        if args.interactive:
            config, replay_speed = interactive_config()
        elif args.config:
            config = load_rocket_config(args.config)
        else:
            config = base_config_from_args(args)
        validate_config(config)
        if args.save_config:
            save_rocket_config(config, args.save_config)
        if not 0.1 <= replay_speed <= 20:
            raise ValueError("Replay speed multiplier must be between 0.1 and 20.")
        if args.fps < 1 or args.fps > 60:
            raise ValueError("fps must be between 1 and 60.")
        if args.compare:
            throttle_levels = (25.0, 50.0, 75.0, 100.0)
            rows = []
            for throttle in throttle_levels:
                level_config = RocketConfig(
                    **{**asdict(config), "throttle_percent": throttle}
                )
                result = simulate(level_config, args.max_time, args.method)
                if not args.json:
                    print_result(
                        level_config,
                        result,
                        args.chart_width,
                        args.chart_height,
                    )
                    if args.animate:
                        animate_flight(
                            level_config,
                            result,
                            replay_speed,
                            args.fps,
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

        if args.sensitivity:
            rows = sensitivity_analysis(config, args.sensitivity)
            print("Sensitivity analysis")
            for row in rows:
                print(
                    f"factor {row['factor']:4.2f}  "
                    f"apogee {row['apogee_m']:12,.1f} m  "
                    f"max velocity {row['max_velocity_m_s']:10,.1f} m/s"
                )
            return 0
        if args.monte_carlo is not None:
            results = monte_carlo(config, args.monte_carlo, args.seed)
            apogees = [result.apogee_m for result in results]
            velocities = [result.max_velocity_m_s for result in results]
            print(f"Monte Carlo samples: {len(results)}")
            print(f"Seed: {args.seed}")
            print(f"Apogee mean: {sum(apogees) / len(apogees):,.2f} m")
            print(f"Apogee range: {min(apogees):,.2f} to {max(apogees):,.2f} m")
            print(
                f"Maximum velocity range: "
                f"{min(velocities):,.2f} to {max(velocities):,.2f} m/s"
            )
            return 0

        result = simulate(config, args.max_time, args.method)
        if args.output_json:
            export_result_json(config, result, args.output_json, args.method)
        if args.csv:
            export_trajectory_csv(result, args.csv)
        if args.save_history:
            with SimulationHistory(args.history_db) as history:
                history.add(config, result, args.method)
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
            should_animate = (
                not args.no_animation
                and (args.animate or args.interactive)
                and not args.json
            )
            if should_animate:
                animate_flight(config, result, replay_speed, args.fps)
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
