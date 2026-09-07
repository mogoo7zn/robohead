"""YAML configuration loading and validation.

All tunable parameters (field, routes, cameras, strategy, hardware) live in
config/*.yaml — nothing is hardcoded in Python.
"""
from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parents[2]
CONFIG_DIR = REPO_ROOT / "config"


class ConfigError(Exception):
    """Raised when a required configuration value is missing or invalid."""


def load_yaml(path: str | Path) -> dict:
    path = Path(path)
    if not path.exists():
        raise ConfigError(f"config file not found: {path}")
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if data is None:
        return {}
    if not isinstance(data, dict):
        raise ConfigError(f"config root must be a mapping: {path}")
    return data


def config_path(name: str) -> Path:
    """Resolve a config file by name, e.g. config_path('strategy.yaml')."""
    return CONFIG_DIR / name


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge override into a copy of base."""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def require(data: dict, key: str, context: str = "") -> Any:
    """Fail fast when a mandatory key is missing (competition config check)."""
    if key not in data or data[key] is None:
        raise ConfigError(f"missing required config key '{key}'"
                          + (f" in {context}" if context else ""))
    return data[key]


def is_filled(value: Any) -> bool:
    """A field is 'filled' when it is neither None nor the TODO placeholder."""
    return value is not None and value != "TODO" and value != ""
