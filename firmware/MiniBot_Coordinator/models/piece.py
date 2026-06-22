"""
models/piece.py  —  MiniBot Chess Swarm Coordinator

Defines the Piece dataclass and BoardState container.

Chess engine integration point:
    BoardState.validate_move() is a stub that always returns True.
    To integrate a rules engine (e.g., python-chess), replace the body
    of validate_move() with a call to your engine adapter, or assign
    CHESS.RULES_ENGINE_ADAPTER in config.py.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

from config import PIECES, CHESS

# ---------------------------------------------------------------------------
# Piece
# ---------------------------------------------------------------------------


@dataclass
class Piece:
    """Represents a single chess robot on the board.

    Attributes:
        piece_id:      Unique hardware ID (0x01–0x22).
        color:         'white' or 'black'.
        rank:          Piece type string, e.g. 'pawn', 'queen'.
        position_mm:   (x_mm, y_mm) center of piece in playing-area coordinates.
                       Origin (0,0) is the bottom-left corner of the playing
                       area. x increases rightward, y increases upward.
        orientation_deg: Heading in degrees (0 = facing +Y / toward opponent).
        is_captured:   True once the piece has been removed from play.
        is_staged:     True when the piece is in the off-board staging zone.
        last_updated:  Unix timestamp of the most recent position update.
    """

    piece_id: int
    color: str  # 'white' | 'black'
    rank: str  # 'pawn' | 'rook' | 'knight' | 'bishop' | 'queen' | 'king'
    position_mm: Tuple[float, float] = field(default_factory=lambda: (0.0, 0.0))
    orientation_deg: float = 0.0
    battery_v: float = 0.0
    is_captured: bool = False
    is_staged: bool = False
    last_updated: float = field(default_factory=time.time)

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------

    @property
    def rank_char(self) -> str:
        """Single-character label for the piece rank, e.g. 'P', 'Q'."""
        return PIECES.RANK_CHAR.get(self.rank, "?")

    @property
    def id_hex(self) -> str:
        """Piece ID formatted as a two-digit hex string, e.g. '0A'."""
        return f"{self.piece_id:02X}"

    @property
    def x_mm(self) -> float:
        return self.position_mm[0]

    @property
    def y_mm(self) -> float:
        return self.position_mm[1]

    def update_position(
        self, x_mm: float, y_mm: float, theta_deg: float, battery_v: float = 0.0
    ) -> None:
        """Update position, orientation and battery voltage, recording the current timestamp."""
        self.position_mm = (x_mm, y_mm)
        self.orientation_deg = theta_deg
        self.battery_v = battery_v
        self.last_updated = time.time()


# ---------------------------------------------------------------------------
# BoardState
# ---------------------------------------------------------------------------


class BoardState:
    """Container for all 34 piece objects plus board-level helpers.

    Usage:
        board = BoardState()
        board.reset_to_home()          # move all pieces to starting positions
        piece = board.get_piece(0x0D)  # fetch piece by ID
    """

    def __init__(self) -> None:
        self._pieces: Dict[int, Piece] = {}
        self._build_pieces()

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def _build_pieces(self) -> None:
        """Construct all 34 Piece objects with home positions and metadata."""
        for pid in range(PIECES.WHITE_ID_START, PIECES.WHITE_ID_END + 1):
            rank = PIECES.PIECE_RANKS[pid]
            pos = PIECES.HOME_POSITIONS[pid]
            piece = Piece(
                piece_id=pid,
                color="white",
                rank=rank,
                position_mm=(float(pos[0]), float(pos[1])),
                orientation_deg=float(pos[2]),
                is_staged=pos[0] < 0,
            )
            self._pieces[pid] = piece

        for pid in range(PIECES.BLACK_ID_START, PIECES.BLACK_ID_END + 1):
            rank = PIECES.PIECE_RANKS[pid]
            pos = PIECES.HOME_POSITIONS[pid]
            piece = Piece(
                piece_id=pid,
                color="black",
                rank=rank,
                position_mm=(float(pos[0]), float(pos[1])),
                orientation_deg=float(pos[2]),
                is_staged=pos[0] < 0,
            )
            self._pieces[pid] = piece

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    def get_piece(self, piece_id: int) -> Optional[Piece]:
        return self._pieces.get(piece_id)

    def all_pieces(self) -> List[Piece]:
        return list(self._pieces.values())

    def active_pieces(self) -> List[Piece]:
        """Pieces that are on the board and not captured."""
        return [p for p in self._pieces.values() if not p.is_captured]

    def pieces_by_color(self, color: str) -> List[Piece]:
        return [p for p in self._pieces.values() if p.color == color]

    # ------------------------------------------------------------------
    # State management
    # ------------------------------------------------------------------

    def reset_to_home(self) -> None:
        """Move all pieces to their standard starting positions."""
        for pid, piece in self._pieces.items():
            home = PIECES.HOME_POSITIONS[pid]
            piece.position_mm = (float(home[0]), float(home[1]))
            piece.orientation_deg = float(home[2])
            piece.is_captured = False
            piece.is_staged = home[0] < 0
            piece.last_updated = time.time()

    def update_piece_position(
        self,
        piece_id: int,
        x_mm: float,
        y_mm: float,
        theta_deg: float,
        battery_v: float = 0.0,
    ) -> None:
        """Update a single piece's position (typically called from serial handler)."""
        piece = self._pieces.get(piece_id)
        if piece is not None:
            piece.update_position(x_mm, y_mm, theta_deg, battery_v)

    # ------------------------------------------------------------------
    # Chess engine integration — FEN export
    # ------------------------------------------------------------------

    def to_fen(self) -> str:
        """Export the current board position as a FEN string.

        CHESS ENGINE INTEGRATION POINT:
            This stub returns the starting FEN regardless of actual piece
            positions. Replace this method body (or call super()) when
            integrating a chess rules engine that tracks logical positions.

        Returns:
            A FEN string representing the current position.
        """
        # TODO: map piece positions to algebraic squares and build FEN.
        return CHESS.STARTING_FEN

    # ------------------------------------------------------------------
    # Chess engine integration — move validation
    # ------------------------------------------------------------------

    def validate_move(
        self,
        piece_id: int,
        target_x_mm: float,
        target_y_mm: float,
        rules_engine: Optional[Callable[[str, str], bool]] = None,
    ) -> bool:
        """Validate whether a move is legal according to chess rules.

        CHESS ENGINE INTEGRATION POINT:
            Currently always returns True (no rules enforcement).

            To integrate a chess engine:
            1. Convert (piece_id, target_x_mm, target_y_mm) → UCI move string.
            2. Obtain the current FEN via self.to_fen().
            3. Call your engine adapter: adapter(fen, uci_move) → bool.
            4. Assign the adapter to CHESS.RULES_ENGINE_ADAPTER in config.py,
               or pass it explicitly via the ``rules_engine`` argument.

        Args:
            piece_id:      The robot piece being moved.
            target_x_mm:   Target X position in playing-area mm.
            target_y_mm:   Target Y position in playing-area mm.
            rules_engine:  Optional callable(fen, uci_move) -> bool.
                           Falls back to CHESS.RULES_ENGINE_ADAPTER if None.

        Returns:
            True if the move is permitted (or if no engine is configured).
        """
        adapter = rules_engine or CHESS.RULES_ENGINE_ADAPTER
        if adapter is None:
            return True

        fen = self.to_fen()
        uci_move = self._to_uci(piece_id, target_x_mm, target_y_mm)
        if uci_move is None:
            return False
        return adapter(fen, uci_move)

    def _to_uci(
        self,
        piece_id: int,
        target_x_mm: float,
        target_y_mm: float,
    ) -> Optional[str]:
        """Convert piece ID + target mm → UCI move string (e.g. 'e2e4').

        CHESS ENGINE INTEGRATION POINT:
            Returns None until board position → algebraic mapping is implemented.
        """
        # TODO: implement mm → algebraic square mapping using BOARD.SQUARE_SIZE_MM
        return None
