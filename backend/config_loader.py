"""Centralized configuration loader for the King Phoenix GCS project.

Loads config.json from the project root and exposes it as a module-level dict.
Any module can import this to access settings without hardcoding.
"""

import json
import logging
from pathlib import Path

logger = logging.getLogger(__name__)


def _find_config() -> Path:
    """Locate config.json by searching upward from this file."""
    here = Path(__file__).resolve().parent
    for root in [here, here.parent, here.parent.parent]:
        candidate = root / "config.json"
        if candidate.exists():
            return candidate
    raise FileNotFoundError("config.json not found in project root")


def _load() -> dict:
    """Load and return the configuration dictionary."""
    path = _find_config()
    with open(path, "r", encoding="utf-8") as f:
        cfg = json.load(f)
    logger.info("Configuration loaded from %s", path)
    return cfg


# Module-level config dict — imported once at startup
CONFIG = _load()


# Convenience accessors
NETWORK = CONFIG["network"]
MISSION = CONFIG["mission"]
PATHS = CONFIG["paths"]
HARDWARE = CONFIG["hardware"]
DETECTION = CONFIG["detection"]
