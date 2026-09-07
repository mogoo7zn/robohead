"""Central logging: timestamp, node, state, task, result, error."""
from __future__ import annotations

import logging
import sys

_CONFIGURED = False


def _configure() -> None:
    global _CONFIGURED
    if _CONFIGURED:
        return
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(logging.Formatter(
        "[%(asctime).11s] %(name)-22s %(levelname)-7s %(message)s",
        datefmt="%H:%M:%S",
    ))
    root = logging.getLogger("robogame")
    root.setLevel(logging.INFO)
    root.addHandler(handler)
    root.propagate = False
    _CONFIGURED = True


def get_logger(name: str) -> logging.Logger:
    _configure()
    if not name.startswith("robogame."):
        name = f"robogame.{name}"
    return logging.getLogger(name)
