"""
planning/base_planner.py  —  MiniBot Chess Swarm Coordinator

Abstract base class for all path planning algorithms.
New planners must subclass BasePlanner and implement plan_moves().

Chess engine integration point:
    BasePlanner.plan_moves() accepts an optional ``validator`` callable.
    When provided, it is called before each move is added to the output list.
    Signature: validator(piece_id, target_x_mm, target_y_mm) -> bool
    This is the injection point for a chess rules engine.
"""

from __future__ import annotations

import math
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from config import PLANNING


# ---------------------------------------------------------------------------
# MoveCommand — the unit of work returned by a planner
# ---------------------------------------------------------------------------

@dataclass
class MoveCommand:
    """Represents a single motor move to be sent to one robot.

    Attributes:
        piece_id:      Target robot ID (0x01–0x22).
        target_x_mm:   Destination X in playing-area mm.
        target_y_mm:   Destination Y in playing-area mm.
        target_theta:  Target orientation in degrees. None = don't change.
        duration_ms:   Time budget for the move in milliseconds.
        sequence_num:  Planner-assigned ordering. Lower numbers move first.
        uci_move:      Optional algebraic notation for chess engine logging.
    """
    piece_id:     int
    target_x_mm:  float
    target_y_mm:  float
    target_theta: Optional[float] = None
    duration_ms:  int             = PLANNING.DEFAULT_MOVE_DURATION_MS
    sequence_num: int             = 0
    uci_move:     Optional[str]   = None

    def distance_mm(self, from_x: float, from_y: float) -> float:
        """Euclidean distance from a given origin to this command's target."""
        return math.hypot(self.target_x_mm - from_x, self.target_y_mm - from_y)


# ---------------------------------------------------------------------------
# BasePlanner
# ---------------------------------------------------------------------------

class BasePlanner(ABC):
    """Abstract base class for path planning algorithms.

    Subclasses implement plan_moves() to translate a dict of desired target
    positions into an ordered list of MoveCommand objects.

    The validator callback is the integration point for the chess rules engine.
    Pass a callable that returns False for illegal moves; the planner must
    then skip or handle those moves appropriately.
    """

    @property
    def name(self) -> str:
        """Human-readable planner name shown in the GUI dropdown."""
        return self.__class__.__name__

    @abstractmethod
    def plan_moves(
        self,
        piece_positions: Dict[int, Tuple[float, float]],
        targets: Dict[int, Tuple[float, float]],
        orientations: Optional[Dict[int, float]] = None,
        validator: Optional[Callable[[int, float, float], bool]] = None,
    ) -> List[MoveCommand]:
        """Generate an ordered list of move commands.

        CHESS ENGINE INTEGRATION POINT:
            If ``validator`` is provided, call it before including each move:
                if not validator(piece_id, target_x, target_y):
                    # skip or handle illegal move
                    continue

        Args:
            piece_positions:
                Dict mapping piece_id → current (x_mm, y_mm).
            targets:
                Dict mapping piece_id → desired (target_x_mm, target_y_mm).
                Only IDs present in this dict need to be planned.
            orientations:
                Optional dict mapping piece_id → current orientation_deg.
                Planners may use this to decide facing direction at target.
            validator:
                Optional callable(piece_id, target_x_mm, target_y_mm) → bool.
                Return False to block an illegal move (chess rules engine hook).

        Returns:
            Ordered list of MoveCommand objects.  The sequence_num field
            determines dispatch order (lower = earlier).
        """
        ...

    # ------------------------------------------------------------------
    # Shared helpers available to all planners
    # ------------------------------------------------------------------

    @staticmethod
    def duration_for_distance(distance_mm: float, speed_mm_per_s: float = 200.0) -> int:
        """Estimate a move duration in ms based on distance and nominal speed.

        Args:
            distance_mm:    Straight-line travel distance.
            speed_mm_per_s: Nominal robot speed (default 200 mm/s).

        Returns:
            Duration in milliseconds, clamped to a sensible minimum.
        """
        if speed_mm_per_s <= 0:
            raise ValueError("speed_mm_per_s must be positive")
        ms = int((distance_mm / speed_mm_per_s) * 1000)
        return max(ms, 500)  # 500 ms minimum to allow settling


# ---------------------------------------------------------------------------
# Factory helper — load a planner by display name from the registry
# ---------------------------------------------------------------------------

def load_planner(display_name: str) -> BasePlanner:
    """Instantiate a planner by its display name as defined in PLANNING.PLANNERS.

    Args:
        display_name: One of the keys in PLANNING.PLANNERS.

    Returns:
        An instance of the corresponding BasePlanner subclass.

    Raises:
        KeyError:      display_name not found in PLANNING.PLANNERS.
        ImportError:   Module not found.
        AttributeError: Class not found in module.
    """
    import importlib
    module_path, class_name = PLANNING.PLANNERS[display_name]
    module = importlib.import_module(module_path)
    cls    = getattr(module, class_name)
    return cls()
