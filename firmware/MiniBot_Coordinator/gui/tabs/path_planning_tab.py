"""
gui/tabs/path_planning_tab.py  —  MiniBot Chess Swarm Coordinator

Path Planning tab:
  - Algorithm selector dropdown
  - Piece selector (or "All")
  - Target X / Y inputs
  - "Plan Move" button  — runs the selected algorithm, populates move queue
  - "Return All Home"   — plans home moves for all pieces
  - "Return All to Graveyard" — plans moves to graveyard for all pieces
  - FEN input + "Set FEN to Board" button — set piece positions from FEN string
  - Move queue list
  - "Send Commands" button — dispatches queued moves to serial handler
"""

from __future__ import annotations

import itertools
import logging
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from itertools import permutations, product
from turtle import home
from typing import Dict, List, Optional, Tuple

from PyQt6.QtCore import pyqtSignal, pyqtSlot, Qt

log = logging.getLogger(__name__)
from PyQt6.QtWidgets import (
    QCheckBox, QComboBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QPushButton, QSizePolicy,
    QVBoxLayout, QWidget, QTextEdit
)

from config import PIECES, PLANNING
from models.piece import BoardState
from planning.base_planner import BasePlanner, MoveCommand, load_planner


# ---------------------------------------------------------------------------
# Module-level worker — must be at module scope so ProcessPoolExecutor can
# pickle it on Windows (spawn start method).
# ---------------------------------------------------------------------------

def _assignment_trial_worker(
    args: tuple,
) -> tuple:
    """Run one planner trial in a subprocess.

    Args:
        args: (trial_idx, positions, trial_targets)

    Returns:
        (trial_idx, score, n_commands, elapsed_ms, error_str or None)
        Commands themselves are NOT returned to avoid large pickle payloads;
        the main process re-runs the winner once to obtain the final command list.
    """
    trial_idx, positions, trial_targets = args
    import time as _time
    from planning.enhanced_conflict_planner import EnhancedConflictPlanner
    planner = EnhancedConflictPlanner()
    t0 = _time.perf_counter()
    try:
        commands = planner.plan_moves(positions, trial_targets,
                                      skip_target_optimisation=True)
        elapsed_ms = (_time.perf_counter() - t0) * 1000
        # Compute score: sum of per-wave max durations
        waves: dict = {}
        for cmd in commands:
            waves[cmd.sequence_num] = max(
                waves.get(cmd.sequence_num, 0.0), float(cmd.duration_ms))
        score = sum(waves.values()) if waves else float('inf')
        return (trial_idx, score, len(commands), elapsed_ms, None)
    except Exception as exc:
        elapsed_ms = (_time.perf_counter() - t0) * 1000
        return (trial_idx, float('inf'), 0, elapsed_ms, str(exc))


class PathPlanningTab(QWidget):
    """Control panel tab for path planning.

    Signals:
        send_commands(list[MoveCommand])   — emitted when "Send Commands" is clicked
        plan_visualized(list, dict)        — emitted after planning to update board arrows
    """

    send_commands   = pyqtSignal(list)   # list[MoveCommand]
    plan_visualized = pyqtSignal(list, dict)  # commands, initial_positions
    planning_log    = pyqtSignal(str)    # debug message → debug tab log

    def __init__(self, board_state: BoardState, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._board         = board_state
        self._move_queue:   List[MoveCommand] = []
        self._selected_id:  Optional[int]     = None  # from chessboard click
        self._viz_enabled:  bool              = True
        # Snapshot of positions at the time the last plan was generated
        self._viz_positions: Dict[int, Tuple[float, float]] = {}

        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(8)

        # --- Algorithm ---
        algo_group = QGroupBox('Algorithm')
        algo_layout = QVBoxLayout(algo_group)
        self._algo_combo = QComboBox()
        for name in PLANNING.PLANNERS:
            self._algo_combo.addItem(name)
        default_idx = self._algo_combo.findText(PLANNING.DEFAULT_PLANNER)
        if default_idx >= 0:
            self._algo_combo.setCurrentIndex(default_idx)
        algo_layout.addWidget(self._algo_combo)
        self._chk_optimize_assignment = QCheckBox('Optimize Assignment')
        self._chk_optimize_assignment.setToolTip(
            'Test every permutation of interchangeable-piece target assignments\n'
            'and keep the plan with the shortest total execution time.\n'
            'Results are logged to the Debug tab.  May be slow for many pieces.'
        )
        algo_layout.addWidget(self._chk_optimize_assignment)
        root.addWidget(algo_group)

        # --- Target ---
        target_group = QGroupBox('Target')
        tl = QVBoxLayout(target_group)

        piece_row = QHBoxLayout()
        piece_row.addWidget(QLabel('Piece:'))
        self._piece_combo = QComboBox()
        self._piece_combo.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        self._rebuild_piece_combo()
        piece_row.addWidget(self._piece_combo)
        tl.addLayout(piece_row)

        xy_row = QHBoxLayout()
        xy_row.addWidget(QLabel('X (mm):'))
        self._target_x = QDoubleSpinBox()
        self._target_x.setRange(-PIECES.CIRCLE_RADIUS_MM * 3, 500.0)
        self._target_x.setDecimals(1)
        self._target_x.setValue(0.0)
        xy_row.addWidget(self._target_x)

        xy_row.addWidget(QLabel('Y (mm):'))
        self._target_y = QDoubleSpinBox()
        self._target_y.setRange(-PIECES.CIRCLE_RADIUS_MM * 3, 500.0)
        self._target_y.setDecimals(1)
        self._target_y.setValue(0.0)
        xy_row.addWidget(self._target_y)
        

    
        

        tl.addLayout(xy_row)

        btn_row = QHBoxLayout()

        self._btn_plan = QPushButton('Plan Move')
        self._btn_plan.clicked.connect(self._on_plan_move)
        btn_row.addWidget(self._btn_plan)

        self._btn_home = QPushButton('Return All Home')
        self._btn_home.clicked.connect(self._on_return_home)
        btn_row.addWidget(self._btn_home)
        tl.addLayout(btn_row)

        self._btn_graveyard = QPushButton('Return All to Graveyard')
        self._btn_graveyard.clicked.connect(self._on_return_graveyard)
        btn_row.addWidget(self._btn_graveyard)
        tl.addLayout(btn_row)

        self._fen_input = QTextEdit('rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1')
        self._fen_input.setFixedHeight(30)
        tl.addWidget(self._fen_input)
        
        self._btn_fen_set = QPushButton('Set FEN to Board')
        self._btn_fen_set.clicked.connect(self._on_fen_positions)
        tl.addWidget(self._btn_fen_set)

        root.addWidget(target_group)

        # --- Move queue ---
        queue_group = QGroupBox('Move Queue')
        ql = QVBoxLayout(queue_group)
        self._queue_list = QListWidget()
        self._queue_list.setMaximumHeight(180)
        ql.addWidget(self._queue_list)

        clr_row = QHBoxLayout()
        self._btn_clear_queue = QPushButton('Clear Queue')
        self._btn_clear_queue.clicked.connect(self._on_clear_queue)
        clr_row.addWidget(self._btn_clear_queue)

        self._btn_clear_paths = QPushButton('Clear Paths')
        self._btn_clear_paths.clicked.connect(self._on_clear_paths)
        clr_row.addWidget(self._btn_clear_paths)

        self._chk_show_paths = QCheckBox('Show Paths')
        self._chk_show_paths.setChecked(True)
        self._chk_show_paths.toggled.connect(self._on_show_paths_toggled)
        clr_row.addWidget(self._chk_show_paths)

        clr_row.addStretch()
        ql.addLayout(clr_row)

        root.addWidget(queue_group)

        # --- Send ---
        self._btn_send = QPushButton('Send Commands')
        self._btn_send.setMinimumHeight(40)
        self._btn_send.setStyleSheet('font-weight: bold;')
        self._btn_send.clicked.connect(self._on_send)
        root.addWidget(self._btn_send)

        root.addStretch()

    # ------------------------------------------------------------------
    # Public slots (wired by MainWindow)
    # ------------------------------------------------------------------

    @pyqtSlot(int)
    def on_piece_selected(self, piece_id: int) -> None:
        """Sync piece selector when user clicks a piece on the board."""
        self._selected_id = piece_id
        idx = self._piece_combo.findData(piece_id)
        if idx >= 0:
            self._piece_combo.setCurrentIndex(idx)

    @pyqtSlot(int, float, float)
    def on_target_set(self, piece_id: int, x_mm: float, y_mm: float) -> None:
        """Receive left-click target from chessboard — sync fields only."""
        self._selected_id = piece_id
        idx = self._piece_combo.findData(piece_id)
        if idx >= 0:
            self._piece_combo.setCurrentIndex(idx)
        self._target_x.setValue(x_mm)
        self._target_y.setValue(y_mm)

    @pyqtSlot(int, float, float)
    def enqueue_from_board(self, piece_id: int, x_mm: float, y_mm: float) -> None:
        """Add a move to the queue from a right-click on the board.

        Uses the currently selected algorithm to build the MoveCommand.
        """
        piece = self._board.get_piece(piece_id)
        if piece is None:
            return

        # Sync the UI fields so the user can see what was queued
        idx = self._piece_combo.findData(piece_id)
        if idx >= 0:
            self._piece_combo.setCurrentIndex(idx)
        self._target_x.setValue(x_mm)
        self._target_y.setValue(y_mm)

        from planning.enhanced_conflict_planner import EnhancedConflictPlanner
        planner   = EnhancedConflictPlanner()
        positions = {p.piece_id: (p.x_mm, p.y_mm) for p in self._board.active_pieces()}
        # All bystanders return to their current position; target piece gets its new goal.
        targets   = dict(positions)
        targets[piece_id] = (x_mm, y_mm)

        def _validator(pid: int, tx: float, ty: float) -> bool:
            return self._board.validate_move(pid, tx, ty)

        commands = planner.plan_moves(positions, targets, validator=_validator)
        self._enqueue(commands, snap_positions=dict(positions))

    # ------------------------------------------------------------------
    # Button handlers
    # ------------------------------------------------------------------

    def _on_plan_move(self) -> None:
        piece_id = self._piece_combo.currentData()
        if piece_id is None:
            return

        target_x = self._target_x.value()
        target_y = self._target_y.value()

        piece = self._board.get_piece(piece_id)
        if piece is None:
            return

        from planning.enhanced_conflict_planner import EnhancedConflictPlanner
        planner   = EnhancedConflictPlanner()
        positions = {p.piece_id: (p.x_mm, p.y_mm) for p in self._board.active_pieces()}
        # All bystanders return to their current position; target piece gets its new goal.
        targets   = dict(positions)
        targets[piece_id] = (target_x, target_y)
        validator = self._board.validate_move  # chess engine hook

        def _validator(pid: int, tx: float, ty: float) -> bool:
            return validator(pid, tx, ty)

        self._viz_positions = {}  # fresh snapshot for new plan
        commands = planner.plan_moves(positions, targets, validator=_validator)
        self._enqueue(commands, snap_positions=dict(positions))

    def _on_return_home(self) -> None:
        from config import PIECES as P, PLANNING as PL
        planner   = self._get_planner()
        positions = {}
        targets   = {}

        for piece in self._board.active_pieces():
            home = P.HOME_POSITIONS.get(piece.piece_id)
            if home:
                positions[piece.piece_id] = (piece.x_mm, piece.y_mm)
                targets[piece.piece_id]   = (float(home[0]), float(home[1]))

        if self._chk_optimize_assignment.isChecked():
            commands = self._exhaustive_assignment_plan(planner, positions, targets)
        else:
            commands = planner.plan_moves(positions, targets)

        for cmd in commands:
            cmd.duration_ms = PL.HOME_MOVE_DURATION_MS
        self._viz_positions = {}  # fresh snapshot for new plan
        self._enqueue(commands, snap_positions=dict(positions))

    def _on_return_graveyard(self) -> None:
        from config import PIECES as P, PLANNING as PL
        planner   = self._get_planner()
        positions = {}
        targets   = {}

        for piece in self._board.active_pieces():
            home = P.GRAVEYARD_POSITIONS.get(piece.piece_id)
            if home:
                positions[piece.piece_id] = (piece.x_mm, piece.y_mm)
                targets[piece.piece_id]   = (float(home[0]), float(home[1]))

        if self._chk_optimize_assignment.isChecked():
            commands = self._exhaustive_assignment_plan(planner, positions, targets)
        else:
            commands = planner.plan_moves(positions, targets)

        for cmd in commands:
            cmd.duration_ms = PL.HOME_MOVE_DURATION_MS
        self._viz_positions = {}  # fresh snapshot for new plan
        self._enqueue(commands, snap_positions=dict(positions))

    def _on_fen_positions(self) -> None:
        from config import PIECES as P, PLANNING as PL
        planner   = self._get_planner()
        positions = {}
        targets   = {}

        
        _fen_string = self._fen_input.toPlainText().strip()
        valid_fen, error = self.validate_fen(_fen_string)
        if not valid_fen:
            self.planning_log.emit(error)
            return
        
        fen_positions = _fen_string.split(" ")[0]
        fen_ranks = fen_positions.split("/")
        
        
        piece_cords = []
        
        for index, rank_data in enumerate(fen_ranks):
            rank_number = 8 - index
            file_index = 0

            for character in rank_data:
                if character.isdigit():
                    file_index += int(character)
                    continue

                file_letter = chr(ord("a") + file_index)
                square = f"{file_letter}{rank_number}"

                
                piece_cords.append((character, rank_number, file_index))

                file_index += 1
                
        def rankFileToMM(x):
            return x*PIECES._S - PIECES._S//2
                
        for   piece in (self._board.active_pieces()):
            piece_found = False
            for index, (char, rank, file) in enumerate(piece_cords):
                color = "black" if char.islower() else "white"
                piece_letter = piece.rank[0] if piece.rank != "knight" else "n"
                if piece_letter == char.lower() and piece.color == color:
                    
                    positions[piece.piece_id] = (piece.x_mm, piece.y_mm)
                    targets[piece.piece_id]   = (rankFileToMM(file) + PIECES._S, rankFileToMM(rank))
                    piece_cords.pop(index)
                    piece_found = True
                    break
            if not piece_found:
                positions[piece.piece_id] = (piece.x_mm, piece.y_mm)
                targets[piece.piece_id]   = (PIECES.GRAVEYARD_POSITIONS[piece.piece_id][0], PIECES.GRAVEYARD_POSITIONS[piece.piece_id][1])

        if self._chk_optimize_assignment.isChecked():
            commands = self._exhaustive_assignment_plan(planner, positions, targets)
        else:
            commands = planner.plan_moves(positions, targets)

        for cmd in commands:
            cmd.duration_ms = PL.HOME_MOVE_DURATION_MS
        self._viz_positions = {}  # fresh snapshot for new plan
        self._enqueue(commands, snap_positions=dict(positions))        

                    
               


    def _on_clear_queue(self) -> None:
        self._move_queue.clear()
        self._queue_list.clear()
        self.plan_visualized.emit([], {})

    def _on_clear_paths(self) -> None:
        self.plan_visualized.emit([], {})

    def _on_show_paths_toggled(self, checked: bool) -> None:
        self._viz_enabled = checked
        if not checked:
            self.plan_visualized.emit([], {})
        elif self._move_queue:
            self.plan_visualized.emit(list(self._move_queue), dict(self._viz_positions))

    def _on_send(self) -> None:
        if self._move_queue:
            self.send_commands.emit(list(self._move_queue))
            self._on_clear_queue()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    # Maximum number of planner calls the exhaustive search will make (also in config.py).
    @property
    def _MAX_EXHAUSTIVE_PLANS(self) -> int:
        return int(getattr(PLANNING, 'OPTIMIZE_ASSIGNMENT_MAX_TRIALS', 200))

    @staticmethod
    def _plan_score(commands: List[MoveCommand]) -> float:
        """Estimate total execution time (ms) for a list of commands.

        Waves run sequentially; the duration of each wave is the longest
        command in that wave.  Returns inf for empty/None plans.
        """
        if not commands:
            return float('inf')
        waves: Dict[int, float] = {}
        for cmd in commands:
            waves[cmd.sequence_num] = max(waves.get(cmd.sequence_num, 0.0),
                                          float(cmd.duration_ms))
        return sum(waves.values())

    def _exhaustive_assignment_plan(
        self,
        planner,
        positions: Dict[int, Tuple[float, float]],
        targets:   Dict[int, Tuple[float, float]],
    ) -> List[MoveCommand]:
        """Run the planner for every permutation of interchangeable-piece
        target assignments and return the plan with the lowest total time.

        Interchangeable groups: pieces of the same colour and piece type
        (pawn / bishop / rook / knight) that are currently active.
        Kings and queens are unique and are never reassigned.

        The search is capped at ``_MAX_EXHAUSTIVE_PLANS`` total planner calls.
        Results are emitted via the ``planning_log`` signal so the Debug tab
        can display them.
        """
        from config import PIECES

        # ---- Build interchangeable groups --------------------------------
        groups: Dict[tuple, List[int]] = {}
        for pid in positions:
            rank = PIECES.PIECE_RANKS.get(pid, '')
            if rank not in ('pawn', 'bishop', 'rook', 'knight'):
                continue
            colour = 'white' if pid in PIECES.WHITE_IDS else 'black'
            groups.setdefault((colour, rank), []).append(pid)

        # Keep only groups with ≥2 active pieces
        groups = {k: sorted(v) for k, v in groups.items() if len(v) >= 2}

        if not groups:
            self.planning_log.emit('[AssignOpt] No interchangeable groups — running single plan.')
            return planner.plan_moves(positions, targets)

        # ---- Generate permutations per group --------------------------------
        cap = self._MAX_EXHAUSTIVE_PLANS
        group_keys  = sorted(groups.keys())
        group_pids  = [groups[k] for k in group_keys]
        group_slots = [[targets[pid] for pid in pids] for pids in group_pids]
        group_perms = [list(permutations(slots)) for slots in group_slots]

        total_combos = 1
        for perms in group_perms:
            total_combos *= len(perms)

        group_summary = ', '.join(
            f'{k[0]}_{k[1]}({len(p)} perms)'
            for k, p in zip(group_keys, group_perms)
        )
        run_count = min(total_combos, cap)
        self.planning_log.emit(
            f'[AssignOpt] Groups: {group_summary}  — '
            f'{total_combos} total combinations, running {run_count}.'
        )

        # ---- Build trial argument list -----------------------------------
        n_workers = int(getattr(PLANNING, 'OPTIMIZE_ASSIGNMENT_WORKERS', 16))
        trial_args: List[tuple] = []
        trial_desc: Dict[int, str] = {}   # trial_idx → human-readable description

        for trial_idx, combo in enumerate(
                itertools.islice(product(*group_perms), cap), start=1):
            trial_targets = dict(targets)
            desc_parts: List[str] = []
            for key, pids, perm in zip(group_keys, group_pids, combo):
                for pid, slot in zip(pids, perm):
                    trial_targets[pid] = slot
                mapping = '  '.join(
                    f'0x{pid:02X}->({slot[0]:.0f},{slot[1]:.0f})'
                    for pid, slot in zip(pids, perm)
                )
                desc_parts.append(f'[{key[0]}_{key[1]}: {mapping}]')
            trial_args.append((trial_idx, positions, trial_targets))
            trial_desc[trial_idx] = '  '.join(desc_parts)

        # ---- Parallel exhaustive search ----------------------------------
        best_score:      float           = float('inf')
        best_trial_idx:  int             = -1
        best_trial_targets: Optional[Dict] = None

        wall_t0 = time.perf_counter()
        with ProcessPoolExecutor(max_workers=min(n_workers, len(trial_args))) as pool:
            futures = {pool.submit(_assignment_trial_worker, a): a[0]
                       for a in trial_args}
            for fut in as_completed(futures):
                trial_idx, score, n_cmds, elapsed_ms, err = fut.result()
                desc = trial_desc[trial_idx]
                if err:
                    self.planning_log.emit(
                        f'[AssignOpt] Trial {trial_idx}/{run_count}: '
                        f'EXCEPTION {err} ({elapsed_ms:.0f} ms)  {desc}'
                    )
                    log.warning('[AssignOpt] Trial %d exception: %s', trial_idx, err)
                    continue
                is_best = score < best_score
                tag = ' *** BEST ***' if is_best else ''
                self.planning_log.emit(
                    f'[AssignOpt] Trial {trial_idx}/{run_count}: '
                    f'score={score:.0f} ms  cmds={n_cmds}  '
                    f'plan_time={elapsed_ms:.0f} ms{tag}  {desc}'
                )
                if is_best:
                    best_score      = score
                    best_trial_idx  = trial_idx
                    best_trial_targets = trial_args[trial_idx - 1][2]

        wall_ms = (time.perf_counter() - wall_t0) * 1000

        # Re-run the winner in-process to obtain the actual command list
        if best_trial_targets is not None:
            self.planning_log.emit(
                f'[AssignOpt] Done. Best trial={best_trial_idx}  '
                f'score={best_score:.0f} ms  wall={wall_ms:.0f} ms  '
                f'(re-running winner to build commands)'
            )
            return planner.plan_moves(positions, best_trial_targets,
                                      skip_target_optimisation=True)

        self.planning_log.emit('[AssignOpt] All trials failed — falling back to default plan.')
        return planner.plan_moves(positions, targets)

    def _get_planner(self) -> BasePlanner:
        name = self._algo_combo.currentText()
        try:
            return load_planner(name)
        except (KeyError, ImportError, AttributeError):
            from planning.direct_planner import DirectPlanner
            return DirectPlanner()

    def _enqueue(self, commands: List[MoveCommand], snap_positions: Optional[Dict[int, Tuple[float, float]]] = None) -> None:
        if snap_positions and not self._viz_positions:
            self._viz_positions = dict(snap_positions)
        elif snap_positions:
            # Merge: only add positions for pieces not already in the snapshot
            for pid, pos in snap_positions.items():
                if pid not in self._viz_positions:
                    self._viz_positions[pid] = pos
        self._move_queue.extend(commands)
        for cmd in commands:
            label = (
                f"[seq {cmd.sequence_num:02d}] "
                f"0x{cmd.piece_id:02X}"
                f" → ({cmd.target_x_mm:.0f}, {cmd.target_y_mm:.0f}) mm"
                f"  {cmd.duration_ms} ms"
            )
            if cmd.planner_debug:
                label += f"  | {cmd.planner_debug}"
            self._queue_list.addItem(QListWidgetItem(label))
        if self._viz_enabled and commands:
            self.plan_visualized.emit(list(self._move_queue), dict(self._viz_positions))

    def _rebuild_piece_combo(self) -> None:
        self._piece_combo.clear()
        for piece in sorted(self._board.all_pieces(), key=lambda p: p.piece_id):
            label = f"0x{piece.piece_id:02X}  {piece.color[0].upper()}  {piece.rank_char}"
            self._piece_combo.addItem(label, userData=piece.piece_id)

    def validate_fen(self, fen: str) -> Tuple[bool, str]:
        VALID_PIECES = set("prnbqkPRNBQK")
        """
        Validate the basic syntax of a complete FEN string.

        Returns:
            (True, "") when valid.
            (False, "reason") when invalid.
        """

        fields = fen.strip().split()

        if len(fields) != 6:
            return False, "FEN must contain exactly 6 fields"

        board, active_color, castling, en_passant, halfmove, fullmove = fields

        # Validate board layout.
        ranks = board.split("/")

        if len(ranks) != 8:
            return False, "Board section must contain exactly 8 ranks"

        white_king_count = 0
        black_king_count = 0

        for rank_number, rank_data in zip(range(8, 0, -1), ranks):
            square_count = 0

            for character in rank_data:
                if character.isdigit():
                    empty_squares = int(character)

                    if not 1 <= empty_squares <= 8:
                        return (
                            False,
                            f"Invalid empty-square count on rank {rank_number}",
                        )

                    square_count += empty_squares

                elif character in VALID_PIECES:
                    square_count += 1

                    if character == "K":
                        white_king_count += 1
                    elif character == "k":
                        black_king_count += 1

                else:
                    return (
                        False,
                        f"Invalid character '{character}' on rank {rank_number}",
                    )

            if square_count != 8:
                return (
                    False,
                    f"Rank {rank_number} contains {square_count} squares instead of 8",
                )

        if white_king_count != 1:
            return False, "FEN must contain exactly one white king"

        if black_king_count != 1:
            return False, "FEN must contain exactly one black king"

        # Validate active color.
        if active_color not in ("w", "b"):
            return False, "Active color must be 'w' or 'b'"

        # Validate castling rights.
        if castling != "-":
            valid_castling = set("KQkq")

            if any(character not in valid_castling for character in castling):
                return False, "Invalid castling rights"

            if len(set(castling)) != len(castling):
                return False, "Castling rights cannot contain duplicates"

        # Validate en passant square.
        if en_passant != "-":
            if len(en_passant) != 2:
                return False, "Invalid en passant square"

            file_character = en_passant[0]
            rank_character = en_passant[1]

            if file_character not in "abcdefgh":
                return False, "Invalid en passant file"

            if rank_character not in ("3", "6"):
                return False, "En passant target must be on rank 3 or 6"

        # Validate halfmove clock.
        try:
            halfmove_number = int(halfmove)

            if halfmove_number < 0:
                return False, "Halfmove clock cannot be negative"

        except ValueError:
            return False, "Halfmove clock must be an integer"

        # Validate fullmove number.
        try:
            fullmove_number = int(fullmove)

            if fullmove_number < 1:
                return False, "Fullmove number must be at least 1"

        except ValueError:
            return False, "Fullmove number must be an integer"

        return True, ""
