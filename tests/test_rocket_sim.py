import math
import unittest

from rocket_sim import G0, RocketConfig, simulate


class RocketSimulationTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
