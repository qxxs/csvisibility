"""Data models shared by the rocket propulsion simulator."""

from dataclasses import dataclass


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
