"""
gui/tabs/play_chess_tab.py  —  MiniBot Chess Swarm Coordinator

Play Chess tab: hot-seat legal chess played on the physical board. python-chess
provides all rules (legal moves, check/mate/stalemate, castling, en passant,
promotion). Clicking a piece highlights its legal destination squares; clicking a
legal square pushes the move and drives the robots there with the selected
planner (captures go to the graveyard, castling moves the rook, en passant clears
the bypassed pawn, and a promotion sends the pawn to the graveyard while a spare
staged queen drives onto the square).

This tab is additive: when Chess Mode is off, board clicks behave exactly as
before. The chess logic is kept in plain methods (no Qt required) so it can be
tested headlessly; the board widget is optional.
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple

import chess

from PyQt6.QtCore import pyqtSignal
from PyQt6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QGroupBox,
    QLabel,
    QListWidget,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from config import BOARD, PIECES, PLANNING
from models.piece import BoardState
from planning.base_planner import BasePlanner, MoveCommand, load_planner

# The two spare queens are staged off-board as promotion reserves.
_SPARE_QUEENS = {"white": 0x11, "black": 0x22}
# Every id except the spare queens starts on a standard square.
_STANDARD_IDS = [pid for pid in range(0x01, 0x23) if pid not in _SPARE_QUEENS.values()]


class PlayChessTab(QWidget):
    """Hot-seat chess control panel driving the robots via the selected planner."""

    # Same signature as the planning tab so main_window routes it identically.
    send_commands = pyqtSignal(list, bool)  # list[MoveCommand], is_trajectory
    chess_mode_changed = pyqtSignal(bool)
    status_log = pyqtSignal(str)

    def __init__(
        self,
        board_state: BoardState,
        board_widget: Optional[QWidget] = None,
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._board = board_state
        self._widget = board_widget
        self._chess = chess.Board()
        self._sq_to_id: Dict[int, int] = {}
        self._spare: Dict[str, Optional[int]] = dict(_SPARE_QUEENS)
        self._grave_free = self._build_graveyard_pools()
        self._selected: Optional[int] = None  # selected source square, or None
        self._build_ui()
        self._build_initial_map()
        self._refresh_status()

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(8)

        algo_group = QGroupBox("Algorithm")
        algo_layout = QVBoxLayout(algo_group)
        self._algo_combo = QComboBox()
        for name in PLANNING.PLANNERS:
            self._algo_combo.addItem(name)
        # Default to our planner for the smooth trajectory playback.
        swarm_idx = self._algo_combo.findText("Swarm (PIBT+LNS)")
        if swarm_idx >= 0:
            self._algo_combo.setCurrentIndex(swarm_idx)
        algo_layout.addWidget(self._algo_combo)
        root.addWidget(algo_group)

        self._chk_chess = QCheckBox("Chess Mode (click a piece, then a legal square)")
        self._chk_chess.toggled.connect(self._on_chess_mode_toggled)
        root.addWidget(self._chk_chess)

        self._btn_new = QPushButton("New Game")
        self._btn_new.clicked.connect(self.new_game)
        root.addWidget(self._btn_new)

        self._turn_label = QLabel()
        root.addWidget(self._turn_label)
        self._status_label = QLabel()
        root.addWidget(self._status_label)

        root.addWidget(QLabel("Moves:"))
        self._move_list = QListWidget()
        root.addWidget(self._move_list, stretch=1)

    # ------------------------------------------------------------------
    # Game lifecycle
    # ------------------------------------------------------------------

    def new_game(self) -> None:
        """Reset pieces to the standard start and begin a fresh game."""
        self._board.reset_to_home()
        self._chess = chess.Board()
        self._spare = dict(_SPARE_QUEENS)
        self._grave_free = self._build_graveyard_pools()
        self._selected = None
        self._move_list.clear()
        self._build_initial_map()
        if self._widget is not None:
            self._widget.clear_legal_highlights()
            self._widget.refresh()
        self._refresh_status()

    def _build_initial_map(self) -> None:
        """Map each standard piece id to the chess square it starts on."""
        self._sq_to_id = {}
        for pid in _STANDARD_IDS:
            hx, hy, _theta = PIECES.HOME_POSITIONS[pid]
            sq = self._mm_to_square(float(hx), float(hy))
            if sq is not None:
                self._sq_to_id[sq] = pid

    # ------------------------------------------------------------------
    # Coordinate helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _square_center(sq: int) -> Tuple[float, float]:
        """Chess square → playing-area mm center (matches the drawn grid)."""
        s = float(BOARD.SQUARE_SIZE_MM)
        return ((chess.square_file(sq) + 0.5) * s, (chess.square_rank(sq) + 0.5) * s)

    @staticmethod
    def _mm_to_square(x_mm: float, y_mm: float) -> Optional[int]:
        """Playing-area mm → chess square, or None if off the 8x8 board."""
        s = float(BOARD.SQUARE_SIZE_MM)
        if x_mm < 0 or y_mm < 0:
            return None
        f = int(x_mm // s)
        r = int(y_mm // s)
        if 0 <= f < BOARD.NUM_SQUARES and 0 <= r < BOARD.NUM_SQUARES:
            return chess.square(f, r)
        return None

    @staticmethod
    def _color_of(pid: int) -> str:
        """'white' or 'black' for a piece id."""
        return "white" if pid in PIECES.WHITE_IDS else "black"

    @staticmethod
    def _build_graveyard_pools() -> Dict[str, List[Tuple[float, float]]]:
        """Free graveyard slots per color, ordered farthest-from-board first.

        Filling the farthest slots first means a newly captured piece always drives
        to the nearest free slot while every occupied slot is beyond it, so its
        straight-in approach never crosses a piece already in the graveyard.
        """
        center_x = float(BOARD.PLAYING_AREA_MM) / 2.0
        pools: Dict[str, List[Tuple[float, float]]] = {"white": [], "black": []}
        for pid, (gx, gy, _t) in PIECES.GRAVEYARD_POSITIONS.items():
            pools[PlayChessTab._color_of(pid)].append((float(gx), float(gy)))
        for slots in pools.values():
            slots.sort(key=lambda p: -abs(p[0] - center_x))
        return pools

    def _assign_graveyard(self, color: str) -> Tuple[float, float]:
        """Take the next free graveyard slot for a color (farthest-first)."""
        pool = self._grave_free.get(color) or []
        return pool.pop(0) if pool else (0.0, 0.0)

    # ------------------------------------------------------------------
    # Interaction
    # ------------------------------------------------------------------

    def _on_chess_mode_toggled(self, on: bool) -> None:
        self.chess_mode_changed.emit(on)
        if self._widget is not None:
            self._widget.set_chess_mode(on)
        self._selected = None
        if on:
            self._highlight()
        elif self._widget is not None:
            self._widget.clear_legal_highlights()

    def on_square_clicked(self, file: int, rank: int) -> None:
        """Handle a board-square click in chess mode: select, re-select, or move."""
        sq = chess.square(file, rank)
        occupant = self._chess.piece_at(sq)

        if self._selected is None:
            if occupant is not None and occupant.color == self._chess.turn:
                self._selected = sq
                self._highlight()
            return

        move = self._legal_move_between(self._selected, sq)
        if move is not None:
            self._play(move)
            self._selected = None
            self._highlight()
        elif occupant is not None and occupant.color == self._chess.turn:
            self._selected = sq
            self._highlight()
        else:
            self._selected = None
            self._highlight()

    def _legal_move_between(self, from_sq: int, to_sq: int) -> Optional[chess.Move]:
        """Return the legal move from->to (auto-queen promotion), or None."""
        promo = None
        piece = self._chess.piece_at(from_sq)
        if (piece is not None and piece.piece_type == chess.PAWN
                and chess.square_rank(to_sq) in (0, 7)):
            promo = chess.QUEEN
        move = chess.Move(from_sq, to_sq, promotion=promo)
        return move if move in self._chess.legal_moves else None

    def _legal_dests(self, from_sq: int) -> List[int]:
        """Destination squares of all legal moves from a square."""
        return [m.to_square for m in self._chess.legal_moves if m.from_square == from_sq]

    def _highlight(self) -> None:
        if self._widget is None:
            return
        if self._selected is None:
            self._widget.clear_legal_highlights()
            return
        dests = self._legal_dests(self._selected)
        squares = {(chess.square_file(d), chess.square_rank(d)) for d in dests}
        sel = (chess.square_file(self._selected), chess.square_rank(self._selected))
        self._widget.set_legal_highlights(squares, sel)

    # ------------------------------------------------------------------
    # Move execution
    # ------------------------------------------------------------------

    def _play(self, move: chess.Move) -> None:
        """Apply a legal move: compute robot targets, dispatch, update state."""
        targets, san, is_white, fullmove = self._apply_move(move)
        self._dispatch(targets)
        self._append_move(san, is_white, fullmove)
        if self._widget is not None:
            self._widget.refresh()
        self._refresh_status()

    def _apply_move(
        self, move: chess.Move
    ) -> Tuple[Dict[int, Tuple[float, float]], str, bool, int]:
        """Compute per-piece mm targets and update the square map for a move.

        Returns (targets, san, is_white, fullmove_number). Uses the pre-push board
        state to resolve captures/castling/en passant, then pushes the move.
        """
        from_sq, to_sq = move.from_square, move.to_square
        mover_color = "white" if self._chess.turn == chess.WHITE else "black"
        is_white = self._chess.turn == chess.WHITE
        fullmove = self._chess.fullmove_number
        is_ep = self._chess.is_en_passant(move)
        is_capture = self._chess.is_capture(move)
        is_castle = self._chess.is_castling(move)
        san = self._chess.san(move)

        targets: Dict[int, Tuple[float, float]] = {}
        mover_id = self._sq_to_id.get(from_sq)

        # Captures (normal or en passant) drive to the graveyard.
        cap_sq: Optional[int] = None
        if is_ep:
            cap_sq = chess.square(chess.square_file(to_sq), chess.square_rank(from_sq))
        elif is_capture:
            cap_sq = to_sq
        cap_id = self._sq_to_id.get(cap_sq) if cap_sq is not None else None
        if cap_id is not None:
            targets[cap_id] = self._assign_graveyard(self._color_of(cap_id))

        # Castling also relocates the rook.
        rook_from = rook_to = rook_id = None
        if is_castle:
            rank = chess.square_rank(from_sq)
            if chess.square_file(to_sq) == 6:      # king-side (g-file king)
                rook_from, rook_to = chess.square(7, rank), chess.square(5, rank)
            else:                                   # queen-side (c-file king)
                rook_from, rook_to = chess.square(0, rank), chess.square(3, rank)
            rook_id = self._sq_to_id.get(rook_from)
            if rook_id is not None:
                targets[rook_id] = self._square_center(rook_to)

        spare_used: Optional[int] = None
        if move.promotion:
            # The pawn retires to the graveyard; a spare queen takes the square.
            if mover_id is not None:
                targets[mover_id] = self._assign_graveyard(mover_color)
            spare_used = self._spare.get(mover_color)
            if spare_used is not None:
                targets[spare_used] = self._square_center(to_sq)
            else:
                self.status_log.emit(
                    f"No spare {mover_color} queen available for promotion."
                )
        elif mover_id is not None:
            targets[mover_id] = self._square_center(to_sq)

        # Commit the logical move, then update the square->id occupancy map.
        self._chess.push(move)
        if cap_sq is not None:
            self._sq_to_id.pop(cap_sq, None)
        self._sq_to_id.pop(from_sq, None)
        if is_castle and rook_id is not None:
            self._sq_to_id.pop(rook_from, None)
            self._sq_to_id[rook_to] = rook_id
        if move.promotion and spare_used is not None:
            self._spare[mover_color] = None
            self._sq_to_id[to_sq] = spare_used
        elif mover_id is not None:
            self._sq_to_id[to_sq] = mover_id

        return targets, san, is_white, fullmove

    def _dispatch(self, targets: Dict[int, Tuple[float, float]]) -> None:
        """Plan the physical moves with the selected planner and emit them."""
        if not targets:
            return
        board_pos = {p.piece_id: (p.x_mm, p.y_mm) for p in self._board.active_pieces()}
        planner = self._current_planner()
        # Exact pieces must move (like a single manual move), so no interchange.
        if hasattr(planner, "interchangeable"):
            planner.interchangeable = False
        # Only pieces still on the board (or moving this turn, including one heading
        # to the graveyard) take part in planning. Pin each to its current square so
        # the planner routes the moving pieces around them (making way where a piece
        # is boxed in) instead of driving through. Already-captured pieces sit
        # off-board and are left out entirely; pinning them made the buffered-Voronoi
        # executor nudge the clustered graveyard pieces a little on every move.
        relevant = (set(self._sq_to_id.values()) | set(targets)) & set(board_pos)
        positions = {pid: board_pos[pid] for pid in relevant}
        full_targets = dict(positions)
        full_targets.update({pid: t for pid, t in targets.items() if pid in positions})
        commands: List[MoveCommand] = planner.plan_moves(positions, full_targets)
        self.send_commands.emit(commands, bool(getattr(planner, "produces_trajectory", False)))

    def _current_planner(self) -> BasePlanner:
        try:
            return load_planner(self._algo_combo.currentText())
        except (KeyError, ImportError, AttributeError):
            from planning.direct_planner import DirectPlanner
            return DirectPlanner()

    # ------------------------------------------------------------------
    # Status / move list
    # ------------------------------------------------------------------

    def _append_move(self, san: str, is_white: bool, fullmove: int) -> None:
        if is_white:
            self._move_list.addItem(f"{fullmove}. {san}")
        elif self._move_list.count():
            item = self._move_list.item(self._move_list.count() - 1)
            item.setText(f"{item.text()}   {san}")
        else:
            self._move_list.addItem(f"{fullmove}... {san}")
        self._move_list.scrollToBottom()

    def _refresh_status(self) -> None:
        side = "White" if self._chess.turn == chess.WHITE else "Black"
        self._turn_label.setText(f"To move: {side}")
        if self._chess.is_checkmate():
            winner = "Black" if self._chess.turn == chess.WHITE else "White"
            self._status_label.setText(f"Checkmate — {winner} wins")
        elif self._chess.is_stalemate():
            self._status_label.setText("Stalemate — draw")
        elif self._chess.is_insufficient_material():
            self._status_label.setText("Draw — insufficient material")
        elif self._chess.is_check():
            self._status_label.setText(f"{side} to move — CHECK")
        else:
            self._status_label.setText("Game in progress")
