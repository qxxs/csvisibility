import unittest
import tempfile
from pathlib import Path

from rocket_sim import (
    G0,
    RocketConfig,
    print_ascii_chart,
    select_animation_frames,
    simulate,
)
from rocket_simulator.models import RocketConfig as PackageRocketConfig
from rocket_simulator.atmosphere import density_at_altitude
from rocket_simulator.reporting import load_rocket_config, save_rocket_config, export_trajectory_csv
from rocket_simulator.storage import SimulationHistory


class RocketSimulationTests(unittest.TestCase):
    def test_rk4_conserves_propellant_and_matches_burnout_scale(self):
        config = RocketConfig(
            dry_mass_kg=50,
            propellant_mass_kg=100,
            thrust_newtons=20_000,
            specific_impulse_s=250,
            burn_time_s=12,
            time_step_s=0.1,
        )
        result = simulate(config, method="rk4")
        self.assertGreater(result.propellant_used_kg, 0)
        self.assertLessEqual(result.propellant_used_kg, config.propellant_mass_kg)
        self.assertLess(abs(result.final_mass_kg - (
            config.dry_mass_kg + config.propellant_mass_kg - result.propellant_used_kg
        )), 1e-9)

    def test_json_config_round_trip(self):
        config = RocketConfig(
            dry_mass_kg=50,
            propellant_mass_kg=100,
            thrust_newtons=20_000,
            specific_impulse_s=250,
            burn_time_s=12,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = str(Path(directory) / "rocket.json")
            save_rocket_config(config, path)
            loaded = load_rocket_config(path)
            self.assertEqual(config, loaded)

    def test_csv_and_sqlite_history(self):
        config = RocketConfig(
            dry_mass_kg=100,
            propellant_mass_kg=0,
            thrust_newtons=100,
            specific_impulse_s=250,
            burn_time_s=1,
        )
        result = simulate(config)
        with tempfile.TemporaryDirectory() as directory:
            csv_path = str(Path(directory) / "flight.csv")
            db_path = str(Path(directory) / "history.sqlite")
            export_trajectory_csv(result, csv_path)
            self.assertTrue(Path(csv_path).stat().st_size > 0)
            with SimulationHistory(db_path) as history:
                simulation_id = history.add(config, result)
                rows = history.list()
                self.assertEqual(rows[0]["id"], simulation_id)
                self.assertTrue(history.delete(simulation_id))
    def test_package_models_match_compatibility_api(self):
        legacy = RocketConfig(
            dry_mass_kg=50,
            propellant_mass_kg=100,
            thrust_newtons=20_000,
            specific_impulse_s=250,
            burn_time_s=12,
        )
        package = PackageRocketConfig(
            dry_mass_kg=50,
            propellant_mass_kg=100,
            thrust_newtons=20_000,
            specific_impulse_s=250,
            burn_time_s=12,
            drag_coefficient=0.4,
            reference_area_m2=0.5,
        )
        self.assertEqual(legacy.dry_mass_kg, package.dry_mass_kg)
        self.assertTrue(G0 > 9)
        self.assertEqual(density_at_altitude(package, 0.0), 1.225)

    def test_non_finite_max_flight_time_is_rejected(self):
        config = RocketConfig(
            dry_mass_kg=100,
            propellant_mass_kg=0,
            thrust_newtons=100,
            specific_impulse_s=250,
            burn_time_s=1,
        )
        with self.assertRaises(ValueError):
            simulate(config, float("nan"))

    def test_chart_width_must_allow_interpolation(self):
        result = simulate(RocketConfig(
            dry_mass_kg=100,
            propellant_mass_kg=0,
            thrust_newtons=100,
            specific_impulse_s=250,
            burn_time_s=1,
        ))
        with self.assertRaises(ValueError):
            print_ascii_chart(result, width=1)
    def test_no_thrust_returns_to_ground(self):
        config = RocketConfig(
            dry_mass_kg=100,
            propellant_mass_kg=0,
            thrust_newtons=100,
            specific_impulse_s=250,
            burn_time_s=1,
            throttle_percent=0,
            time_step_s=0.1,
        )
        result = simulate(config)
        self.assertEqual(result.status, "impact")
        self.assertLess(result.impact_velocity_m_s, 0)
        self.assertEqual(result.apogee_m, 0)

    def test_thrust_increases_altitude(self):
        config = RocketConfig(
            dry_mass_kg=50,
            propellant_mass_kg=100,
            thrust_newtons=20_000,
            specific_impulse_s=250,
            burn_time_s=12,
            throttle_percent=100,
            time_step_s=0.1,
        )
        result = simulate(config)
        self.assertGreater(result.apogee_m, 0)
        self.assertGreater(result.max_velocity_m_s, 0)
        self.assertGreater(result.propellant_used_kg, 0)

    def test_higher_throttle_reaches_higher_apogee(self):
        base = dict(
            dry_mass_kg=50,
            propellant_mass_kg=100,
            thrust_newtons=20_000,
            specific_impulse_s=250,
            burn_time_s=12,
            time_step_s=0.1,
        )
        low = simulate(RocketConfig(**base, throttle_percent=50))
        high = simulate(RocketConfig(**base, throttle_percent=100))
        self.assertGreater(high.apogee_m, low.apogee_m)

    def test_negative_mass_is_rejected(self):
        config = RocketConfig(
            dry_mass_kg=-1,
            propellant_mass_kg=100,
            thrust_newtons=20_000,
            specific_impulse_s=250,
            burn_time_s=12,
        )
        with self.assertRaises(ValueError):
            simulate(config)

    def test_animation_frames_are_bounded_and_includes_ending(self):
        config = RocketConfig(
            dry_mass_kg=50,
            propellant_mass_kg=100,
            thrust_newtons=20_000,
            specific_impulse_s=250,
            burn_time_s=12,
            throttle_percent=100,
            time_step_s=0.1,
        )
        result = simulate(config)
        frames = select_animation_frames(
            result,
            target_fps=30,
            playback_speed=1,
            max_frames=900,
        )
        self.assertGreater(len(frames), 1)
        self.assertLessEqual(len(frames), 900)
        self.assertEqual(frames[-1].time_s, result.points[-1].time_s)


if __name__ == "__main__":
    unittest.main()
