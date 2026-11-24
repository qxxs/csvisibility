"""CSV and JSON reporting utilities."""

import csv
import json
from dataclasses import asdict
from pathlib import Path

from .models import FlightResult, RocketConfig


def load_rocket_config(path: str) -> RocketConfig:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if "config" in data:
        data = data["config"]
    try:
        return RocketConfig(**data)
    except TypeError as error:
        raise ValueError(f"invalid rocket configuration: {error}") from error


def save_rocket_config(config: RocketConfig, path: str) -> None:
    Path(path).write_text(
        json.dumps(asdict(config), indent=2) + "\n",
        encoding="utf-8",
    )


def export_result_json(
    config: RocketConfig,
    result: FlightResult,
    path: str,
    integration_method: str = "euler",
) -> None:
    report = {
        "model": "vertical_single_stage_educational",
        "integration_method": integration_method,
        "config": asdict(config),
        "result": {
            key: value
            for key, value in asdict(result).items()
            if key != "points"
        },
    }
    Path(path).write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")


def export_trajectory_csv(result: FlightResult, path: str) -> None:
    with open(path, "w", newline="", encoding="utf-8") as output:
        writer = csv.writer(output)
        writer.writerow([
            "time_s",
            "altitude_m",
            "velocity_m_s",
            "acceleration_m_s2",
            "mass_kg",
            "thrust_n",
            "propellant_kg",
        ])
        for point in result.points:
            writer.writerow([
                point.time_s,
                point.altitude_m,
                point.velocity_m_s,
                point.acceleration_m_s2,
                point.mass_kg,
                point.thrust_n,
                point.propellant_kg,
            ])
