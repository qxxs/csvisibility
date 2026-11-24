"""Atmospheric models for the vertical-flight simulator."""

import math

from .models import RocketConfig


def density_at_altitude(config: RocketConfig, altitude_m: float) -> float:
    """Return exponential-atmosphere density, or zero when drag is disabled."""
    if config.drag_coefficient == 0 or config.reference_area_m2 == 0:
        return 0.0
    return config.air_density_kg_m3 * math.exp(
        -max(0.0, altitude_m) / config.scale_height_m
    )
