"""Sensitivity, Monte Carlo and comparison analysis helpers."""

import random
from dataclasses import replace
from typing import Iterable

from .models import FlightResult, RocketConfig


def compare_configurations(configurations: Iterable[RocketConfig]) -> list[dict[str, float | str]]:
    from .simulation import simulate

    rows: list[dict[str, float | str]] = []
    for config in configurations:
        result = simulate(config)
        rows.append({
            "thrust_n": config.thrust_newtons,
            "specific_impulse_s": config.specific_impulse_s,
            "burn_time_s": config.burn_time_s,
            "apogee_m": result.apogee_m,
            "max_velocity_m_s": result.max_velocity_m_s,
            "max_acceleration_m_s2": result.max_acceleration_m_s2,
            "propellant_used_kg": result.propellant_used_kg,
            "status": result.status,
        })
    return rows


def sensitivity_analysis(
    base: RocketConfig,
    field: str,
    factors: Iterable[float] = (0.8, 0.9, 1.0, 1.1, 1.2),
) -> list[dict[str, float]]:
    """Scale one numeric RocketConfig field and report outcome metrics."""
    if not hasattr(base, field):
        raise ValueError(f"unknown rocket configuration field: {field}")
    base_value = getattr(base, field)
    if not isinstance(base_value, (int, float)):
        raise ValueError(f"field must be numeric: {field}")

    from .simulation import simulate

    rows: list[dict[str, float]] = []
    for factor in factors:
        config = replace(base, **{field: base_value * factor})
        result = simulate(config)
        rows.append({
            "factor": factor,
            "apogee_m": result.apogee_m,
            "max_velocity_m_s": result.max_velocity_m_s,
            "max_acceleration_m_s2": result.max_acceleration_m_s2,
            "flight_time_s": result.flight_time_s,
        })
    return rows


def monte_carlo(
    base: RocketConfig,
    samples: int,
    seed: int,
    dry_mass_sigma_fraction: float = 0.02,
    thrust_sigma_fraction: float = 0.03,
) -> list[FlightResult]:
    """Run reproducible normal-distribution Monte Carlo samples."""
    if samples <= 0:
        raise ValueError("samples must be greater than zero.")
    if dry_mass_sigma_fraction < 0 or thrust_sigma_fraction < 0:
        raise ValueError("uncertainty fractions must be zero or greater.")

    from .simulation import simulate

    rng = random.Random(seed)
    results: list[FlightResult] = []
    for _ in range(samples):
        config = replace(
            base,
            dry_mass_kg=max(0.0, rng.gauss(base.dry_mass_kg, base.dry_mass_kg * dry_mass_sigma_fraction)),
            thrust_newtons=max(0.0, rng.gauss(base.thrust_newtons, base.thrust_newtons * thrust_sigma_fraction)),
        )
        results.append(simulate(config))
    return results
