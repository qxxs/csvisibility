"""SQLite persistence for simulation history."""

import json
import sqlite3
from dataclasses import asdict
from pathlib import Path

from .models import FlightResult, RocketConfig


SCHEMA = """
CREATE TABLE IF NOT EXISTS simulations (
    id INTEGER PRIMARY KEY AUTOINCREMENT,
    created_at TEXT NOT NULL DEFAULT CURRENT_TIMESTAMP,
    rocket_name TEXT NOT NULL,
    config_json TEXT NOT NULL,
    integration_method TEXT NOT NULL,
    status TEXT NOT NULL,
    flight_time_s REAL NOT NULL,
    apogee_m REAL NOT NULL,
    max_velocity_m_s REAL NOT NULL,
    max_acceleration_m_s2 REAL NOT NULL,
    burnout_time_s REAL NOT NULL,
    propellant_used_kg REAL NOT NULL,
    result_json TEXT NOT NULL
);
"""


class SimulationHistory:
    def __init__(self, path: str = "data/simulation_history.sqlite3"):
        self.path = path
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.connection = sqlite3.connect(path)
        self.connection.row_factory = sqlite3.Row
        self.connection.execute("PRAGMA foreign_keys = ON")
        self.connection.executescript(SCHEMA)
        self.connection.commit()

    def close(self) -> None:
        self.connection.close()

    def add(
        self,
        config: RocketConfig,
        result: FlightResult,
        integration_method: str = "euler",
        rocket_name: str = "Unnamed Rocket",
    ) -> int:
        cursor = self.connection.execute(
            """
            INSERT INTO simulations (
                rocket_name, config_json, integration_method, status,
                flight_time_s, apogee_m, max_velocity_m_s,
                max_acceleration_m_s2, burnout_time_s,
                propellant_used_kg, result_json
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                rocket_name,
                json.dumps(asdict(config)),
                integration_method,
                result.status,
                result.flight_time_s,
                result.apogee_m,
                result.max_velocity_m_s,
                result.max_acceleration_m_s2,
                result.burnout_time_s,
                result.propellant_used_kg,
                json.dumps(asdict(result)),
            ),
        )
        self.connection.commit()
        return int(cursor.lastrowid)

    def list(self, limit: int = 50) -> list[sqlite3.Row]:
        return self.connection.execute(
            """
            SELECT id, created_at, rocket_name, integration_method, status,
                   flight_time_s, apogee_m, max_velocity_m_s,
                   max_acceleration_m_s2, propellant_used_kg
            FROM simulations
            ORDER BY id DESC
            LIMIT ?
            """,
            (limit,),
        ).fetchall()

    def delete(self, simulation_id: int) -> bool:
        cursor = self.connection.execute(
            "DELETE FROM simulations WHERE id = ?",
            (simulation_id,),
        )
        self.connection.commit()
        return cursor.rowcount > 0

    def __enter__(self):
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.close()
