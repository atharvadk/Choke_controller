"""
Simple CSV logger for simulation data.
"""

import csv
import os
from typing import Dict, Any, List


class CSVLogger:
    def __init__(self, filepath: str, fieldnames: List[str]):
        """
        Args:
            filepath: where to write the CSV file.
            fieldnames: list of column names (order matters).
        """
        self.filepath = filepath
        self.fieldnames = fieldnames
        self.file = None
        self.writer = None
        self._open_file()

    def _open_file(self) -> None:
        # Ensure directory exists
        dir_name = os.path.dirname(self.filepath)
        if dir_name:
            os.makedirs(dir_name, exist_ok=True)
        # Open in write mode, write header
        self.file = open(self.filepath, mode='w', newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=self.fieldnames)
        self.writer.writeheader()

    def log(self, row: Dict[str, Any]) -> None:
        """Write a single row (dict) to the CSV."""
        if self.writer is None:
            raise RuntimeError("Logger not initialized")
        # Ensure all fields are present; missing ones become empty
        filtered = {k: row.get(k, '') for k in self.fieldnames}
        self.writer.writerow(filtered)

    def close(self) -> None:
        if self.file:
            self.file.flush()
            self.file.close()
            self.file = None
            self.writer = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.close()


def build_default_fieldnames(state) -> List[str]:
    """
    Helper to create a sensible list of columns from a WellState instance.
    Called once at start to define the CSV header.
    """
    # We'll manually list for clarity; could auto‑extract via dataclasses.fields.
    return [
        'time',
        'Pr', 'Tr',
        'Pwf',
        'Pth',
        'opening_target', 'opening_actual', 'effective_area', 'pressure_drop',
        'Pwh', 'Twh', 'separator_pressure',
        'oil_rate', 'gas_rate', 'water_rate', 'total_flow',
        'density', 'viscosity', 'water_cut', 'gor',
        'sand_rate',
        'reward'
    ]