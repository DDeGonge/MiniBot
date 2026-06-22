"""
gui/main_window.py  —  MiniBot Chess Swarm Coordinator

Top-level QMainWindow.  Assembles: ChessBoardWidget (left) + QTabWidget
(right) and wires all inter-component signals.

Signal routing summary:
  Board.piece_selected      → PathPlanningTab.on_piece_selected
  Board.target_set          → PathPlanningTab.on_target_set
  PathPlanningTab.send_commands → _on_send_move_commands (serial or sim)
  DebugTab.send_raw         → _on_debug_send_raw (serial or sim)
  DebugTab.simulator_mode_changed → _on_sim_mode_changed
  SystemControlTab.send_raw → serial_handler.send
  PositionTrackerTab.send_raw → serial_handler.send
  serial_handler.position_received → _on_position_received → board + tracker
  serial_handler.raw_line_received → DebugTab.on_raw_line_received
  serial_handler.ack_received      → DebugTab.on_ack_received
  serial_handler.error_received    → DebugTab.on_error_received
  serial_handler.serial_error      → status bar
  simulator.position_updated       → _on_position_received → board + tracker
  simulator.log_message            → DebugTab.on_sim_log
"""

from __future__ import annotations

import math
import random
import time
from typing import Dict, List, Optional, Set, Tuple

from PyQt6.QtCore import QTimer, pyqtSlot
from PyQt6.QtWidgets import (
    QComboBox, QGroupBox, QHBoxLayout, QLabel, QMainWindow, QPushButton,
    QStatusBar, QTabWidget, QVBoxLayout, QWidget,
)

from comms.protocol import build_position_request, build_position_command
from comms.serial_handler import SerialHandler
from config import COMM, GUI, PIECES, PLANNING, SIMULATOR
from models.piece import BoardState
from planning.base_planner import MoveCommand
from simulation.simulator import MotionSimulator

from gui.chessboard_widget import ChessBoardWidget
from gui.tabs.path_planning_tab    import PathPlanningTab
from gui.tabs.debug_tab            import DebugTab
from gui.tabs.system_control_tab   import SystemControlTab
from gui.tabs.position_tracker_tab import PositionTrackerTab
from gui.tabs.command_looper_tab   import CommandLooperTab

# ---------------------------------------------------------------------------
# Application-wide dark theme stylesheet
# ---------------------------------------------------------------------------
_STYLESHEET = """
/* ── Base ──────────────────────────────────────────────────────────────── */
QMainWindow, QWidget {
    background-color: #1f1f1f;
    color: #d0d0d0;
    font-family: "Segoe UI", "Helvetica Neue", sans-serif;
    font-size: 13px;
}

/* ── Group boxes ────────────────────────────────────────────────────────── */
QGroupBox {
    background-color: #272727;
    border: 1px solid #3a3a3a;
    border-radius: 6px;
    margin-top: 10px;
    padding: 6px 8px 8px 8px;
    font-weight: 600;
    color: #999999;
}
QGroupBox::title {
    subcontrol-origin: margin;
    subcontrol-position: top left;
    left: 10px;
    padding: 0 4px;
    color: #999999;
}

/* ── Buttons ────────────────────────────────────────────────────────────── */
QPushButton {
    background-color: #303030;
    color: #d0d0d0;
    border: 1px solid #484848;
    border-radius: 5px;
    padding: 4px 12px;
    font-weight: 600;
    min-height: 22px;
}
QPushButton:hover {
    background-color: #3c3c3c;
    border-color: #707070;
    color: #f0f0f0;
}
QPushButton:pressed {
    background-color: #484848;
    border-color: #888888;
}
QPushButton:disabled {
    background-color: #272727;
    color: #505050;
    border-color: #303030;
}

/* ── Tabs ───────────────────────────────────────────────────────────────── */
QTabWidget::pane {
    border: 1px solid #3a3a3a;
    border-radius: 0 4px 4px 4px;
    background-color: #272727;
    top: -1px;
}
QTabBar::tab {
    background-color: #1f1f1f;
    color: #787878;
    border: 1px solid #3a3a3a;
    border-bottom: none;
    border-radius: 4px 4px 0 0;
    padding: 5px 16px;
    margin-right: 3px;
    font-weight: 600;
}
QTabBar::tab:selected {
    background-color: #272727;
    color: #d0d0d0;
    border-bottom: 2px solid #888888;
}
QTabBar::tab:hover:!selected {
    background-color: #2c2c2c;
    color: #b0b0b0;
}

/* ── Combo boxes ────────────────────────────────────────────────────────── */
QComboBox {
    background-color: #303030;
    color: #d0d0d0;
    border: 1px solid #484848;
    border-radius: 4px;
    padding: 3px 8px;
    min-height: 22px;
}
QComboBox:hover { border-color: #707070; }
QComboBox::drop-down { border: none; width: 18px; }
QComboBox::down-arrow { image: none; width: 0; }
QComboBox QAbstractItemView {
    background-color: #272727;
    color: #d0d0d0;
    border: 1px solid #3a3a3a;
    selection-background-color: #3c3c3c;
    selection-color: #f0f0f0;
    outline: none;
}

/* ── Spin boxes ─────────────────────────────────────────────────────────── */
QSpinBox, QDoubleSpinBox {
    background-color: #303030;
    color: #d0d0d0;
    border: 1px solid #484848;
    border-radius: 4px;
    padding: 3px 6px;
    min-height: 22px;
}
QSpinBox:hover, QDoubleSpinBox:hover { border-color: #707070; }
QSpinBox::up-button, QDoubleSpinBox::up-button,
QSpinBox::down-button, QDoubleSpinBox::down-button {
    background-color: #3c3c3c;
    border: none;
    border-radius: 2px;
    width: 16px;
}
QSpinBox::up-button:hover, QDoubleSpinBox::up-button:hover,
QSpinBox::down-button:hover, QDoubleSpinBox::down-button:hover {
    background-color: #484848;
}

/* ── Text areas ─────────────────────────────────────────────────────────── */
QTextEdit, QPlainTextEdit {
    background-color: #141414;
    color: #b8b8b8;
    border: 1px solid #3a3a3a;
    border-radius: 4px;
    font-family: "Consolas", "Courier New", monospace;
    font-size: 12px;
    selection-background-color: #3c3c3c;
}

/* ── Labels ─────────────────────────────────────────────────────────────── */
QLabel {
    color: #c0c0c0;
    background: transparent;
}

/* ── Check boxes ────────────────────────────────────────────────────────── */
QCheckBox {
    color: #c0c0c0;
    spacing: 7px;
}
QCheckBox::indicator {
    width: 15px;
    height: 15px;
    border: 1px solid #484848;
    border-radius: 3px;
    background-color: #303030;
}
QCheckBox::indicator:checked {
    background-color: #787878;
    border-color: #888888;
}
QCheckBox::indicator:hover {
    border-color: #707070;
}

/* ── Tables ─────────────────────────────────────────────────────────────── */
QTableWidget {
    background-color: #272727;
    color: #d0d0d0;
    border: 1px solid #3a3a3a;
    border-radius: 4px;
    gridline-color: #303030;
    outline: none;
}
QTableWidget::item { padding: 2px 6px; }
QTableWidget::item:selected {
    background-color: #c8c4b8;
    color: #1a1a1a;
}
QHeaderView::section {
    background-color: #1f1f1f;
    color: #909090;
    border: none;
    border-bottom: 1px solid #3a3a3a;
    border-right: 1px solid #3a3a3a;
    padding: 4px 8px;
    font-weight: 700;
}

/* ── Scrollbars ─────────────────────────────────────────────────────────── */
QScrollBar:vertical {
    background: #1f1f1f;
    width: 8px;
    margin: 0;
}
QScrollBar::handle:vertical {
    background: #484848;
    border-radius: 4px;
    min-height: 20px;
}
QScrollBar::handle:vertical:hover { background: #606060; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }
QScrollBar:horizontal {
    background: #1f1f1f;
    height: 8px;
    margin: 0;
}
QScrollBar::handle:horizontal {
    background: #484848;
    border-radius: 4px;
    min-width: 20px;
}
QScrollBar::handle:horizontal:hover { background: #606060; }
QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal { width: 0; }

/* ── Status bar ─────────────────────────────────────────────────────────── */
QStatusBar {
    background-color: #141414;
    color: #888888;
    border-top: 1px solid #3a3a3a;
    font-size: 12px;
}
QStatusBar::item { border: none; }

/* ── Splitter ───────────────────────────────────────────────────────────── */
QSplitter::handle { background: #3a3a3a; }
"""


class MainWindow(QMainWindow):
    """Application main window."""

    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle(GUI.WINDOW_TITLE)
        self.setMinimumSize(GUI.WINDOW_MIN_WIDTH, GUI.WINDOW_MIN_HEIGHT)
        self.setStyleSheet(_STYLESHEET)

        # Shared state
        self._board      = BoardState()
        self._board.reset_to_home()
        self._handler    = SerialHandler(self)
        self._simulator  = MotionSimulator(self._board, parent=self)
        self._sim_mode   = False  # True when debug tab simulator is active

        # Sequence-wave dispatcher state.
        self._pending_waves: List[List[MoveCommand]] = []
        self._active_wave_piece_ids: Set[int] = set()
        self._wave_running: bool = False
        self._wave_timer = QTimer(self)
        self._wave_timer.setSingleShot(True)
        self._wave_timer.timeout.connect(self._on_wave_timer_timeout)

        # Auto-poll timer (fires POLL at the configured interval when connected)
        self._poll_timer = QTimer(self)
        self._poll_timer.setInterval(COMM.DEFAULT_POLL_INTERVAL_MS)
        self._poll_timer.timeout.connect(self._on_poll_timer)

        # Stale-piece tracking
        self._last_seen:      Dict[int, float] = {}   # piece_id → monotonic time
        self._hide_stale:     bool = False
        self._stale_check_timer = QTimer(self)

        # Polling config
        self._poll_enabled: bool = True
        self._poll_target:  int  = 0xFF
        self._stale_check_timer.setInterval(2000)
        self._stale_check_timer.timeout.connect(self._on_stale_check_timer)
        self._stale_check_timer.start()

        # Repaint throttle: update the canvas at a fixed rate rather than
        # on every incoming position message.
        self._repaint_timer = QTimer(self)
        self._repaint_timer.setInterval(50)   # ~20 Hz
        self._repaint_timer.start()

        self._build_ui()
        # Connect after _build_ui() so _board_widget exists
        self._repaint_timer.timeout.connect(self._board_widget.refresh)
        self._wire_signals()

        # Status bar
        self._status_bar: QStatusBar = self.statusBar()
        self._status_bar.showMessage('Not connected')

    # ------------------------------------------------------------------
    # UI assembly
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        central = QWidget()
        self.setCentralWidget(central)

        layout = QHBoxLayout(central)
        layout.setSpacing(6)
        layout.setContentsMargins(6, 6, 6, 6)

        # Left: chessboard
        self._board_widget = ChessBoardWidget(self._board, self)
        layout.addWidget(self._board_widget, stretch=1)

        # Right: connection bar + tabbed control panel
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setSpacing(4)
        right_layout.setContentsMargins(0, 0, 0, 0)

        # --- Always-visible connection bar ---
        conn_group = QGroupBox('Serial Connection')
        conn_group_layout = QVBoxLayout(conn_group)
        conn_group_layout.setSpacing(3)
        conn_group_layout.setContentsMargins(6, 6, 6, 6)

        conn_row = QHBoxLayout()
        conn_row.setSpacing(6)
        conn_row.addWidget(QLabel('Port:'))
        self._port_combo = QComboBox()
        self._port_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToContents)
        self._refresh_ports()
        conn_row.addWidget(self._port_combo)

        self._btn_refresh_ports = QPushButton('↺')
        self._btn_refresh_ports.setFixedWidth(35)
        self._btn_refresh_ports.setToolTip('Refresh serial port list')
        self._btn_refresh_ports.clicked.connect(self._refresh_ports)
        conn_row.addWidget(self._btn_refresh_ports)
        conn_row.addStretch()

        conn_row.addWidget(QLabel('Baud:'))
        self._baud_combo = QComboBox()
        for baud in [9600, 57600, 115200, 250000, 460800, 921600]:
            self._baud_combo.addItem(str(baud), userData=baud)
        default_idx = self._baud_combo.findData(COMM.DEFAULT_BAUD_RATE)
        if default_idx >= 0:
            self._baud_combo.setCurrentIndex(default_idx)
        conn_row.addWidget(self._baud_combo)

        self._btn_connect = QPushButton('Connect')
        self._btn_connect.clicked.connect(self._on_connect)
        conn_row.addWidget(self._btn_connect)

        self._btn_disconnect = QPushButton('Disconnect')
        self._btn_disconnect.setEnabled(False)
        self._btn_disconnect.clicked.connect(self._on_disconnect)
        conn_row.addWidget(self._btn_disconnect)

        conn_group_layout.addLayout(conn_row)

        self._conn_status_label = QLabel('Status: Disconnected')
        self._conn_status_label.setStyleSheet('color: #787878;')
        conn_group_layout.addWidget(self._conn_status_label)

        right_layout.addWidget(conn_group)

        # Tabbed control panel
        self._tabs = QTabWidget()
        self._tabs.setMinimumWidth(GUI.CONTROL_PANEL_MIN_WIDTH)

        self._path_tab   = PathPlanningTab(self._board, self)
        self._debug_tab  = DebugTab(self)
        self._sys_tab    = SystemControlTab(self)
        self._track_tab  = PositionTrackerTab(self._board, self)
        self._looper_tab = CommandLooperTab(self)

        self._tabs.addTab(self._path_tab,   'Path Planning')
        self._tabs.addTab(self._debug_tab,  'Debug')
        self._tabs.addTab(self._sys_tab,    'System Control')
        self._tabs.addTab(self._track_tab,  'Position Tracker')
        self._tabs.addTab(self._looper_tab, 'Command Looper')

        right_layout.addWidget(self._tabs, stretch=1)
        layout.addWidget(right_widget, stretch=0)

    def _wire_signals(self) -> None:
        # Board → path planning tab (sync fields + queuing)
        self._board_widget.piece_selected.connect(self._path_tab.on_piece_selected)
        self._board_widget.target_set.connect(self._path_tab.on_target_set)

        # Board left-click → plan + enqueue
        self._board_widget.target_set.connect(self._path_tab.enqueue_from_board)

        # Board right-click → immediate dispatch
        self._board_widget.target_queued.connect(self._on_board_target_set)

        # Path planning → send (serial or sim depending on mode)
        self._path_tab.send_commands.connect(self._on_send_move_commands)

        # Path planning → board visualization
        self._path_tab.plan_visualized.connect(self._on_plan_visualized)

        # Path planning assignment-optimizer → debug log
        self._path_tab.planning_log.connect(self._debug_tab.on_sim_log)

        # Debug tab → send (serial or sim) + simulator toggle
        self._debug_tab.send_raw.connect(self._on_debug_send_raw)
        self._debug_tab.simulator_mode_changed.connect(self._on_sim_mode_changed)
        self._debug_tab.hide_stale_pieces_changed.connect(self._on_hide_stale_changed)
        self._debug_tab.show_electromagnets_changed.connect(self._board_widget.set_show_electromagnets)
        self._debug_tab.randomize_positions.connect(self._on_randomize_positions)
        self._debug_tab.set_fen_postions.connect(self._on_set_fen_positions)
        self._debug_tab.sim_collision_changed.connect(self._on_sim_collision_changed)
        self._debug_tab.clear_pending_moves.connect(self._simulator.stop_all)

        # System control → send (always serial; system commands never simulated)
        self._sys_tab.send_raw.connect(self._handler.send)
        self._sys_tab.poll_interval_changed.connect(self._on_poll_interval_changed)
        self._sys_tab.poll_enabled_changed.connect(self._on_poll_enabled_changed)
        self._sys_tab.poll_target_changed.connect(self._on_poll_target_changed)

        # Position tracker → poll (always serial)
        self._track_tab.send_raw.connect(self._handler.send)

        # Command looper → planned move dispatch (rotate-then-straight, same as right-click)
        self._looper_tab.request_move.connect(self._on_looper_request_move)

        # Serial handler → board updates + UI
        self._handler.position_received.connect(self._on_position_received)
        self._handler.raw_line_received.connect(self._debug_tab.on_raw_line_received)
        self._handler.ack_received.connect(self._debug_tab.on_ack_received)
        self._handler.error_received.connect(self._debug_tab.on_error_received)
        self._handler.position_received.connect(self._track_tab.on_position_received)
        self._handler.connection_changed.connect(self._on_connection_changed)
        self._handler.serial_error.connect(self._on_serial_error)

        # Simulator → board updates + debug log
        self._simulator.position_updated.connect(self._on_position_received)
        self._simulator.position_updated.connect(self._track_tab.on_position_received)
        self._simulator.move_complete.connect(self._on_sim_move_complete)
        self._simulator.log_message.connect(self._debug_tab.on_sim_log)

    # ------------------------------------------------------------------
    # Slots
    # ------------------------------------------------------------------

    @pyqtSlot(int, float, float, float, float)
    def _on_position_received(self, piece_id: int, x_mm: float, y_mm: float, theta_deg: float, battery_v: float) -> None:
        """Update board state; canvas repaint is handled by the repaint timer."""
        self._last_seen[piece_id] = time.monotonic()
        self._board.update_piece_position(piece_id, x_mm, y_mm, theta_deg, battery_v)

    @pyqtSlot(int, float, float)
    def _on_board_target_set(self, piece_id: int, x_mm: float, y_mm: float) -> None:
        """Immediately dispatch a move when the user clicks a target on the board.

        Uses the DirectPlanner to build a single MoveCommand and sends it
        straight to _on_send_move_commands without touching the queue.
        """
        from planning.enhanced_conflict_planner import EnhancedConflictPlanner
        piece = self._board.get_piece(piece_id)
        if piece is None:
            return
        planner   = EnhancedConflictPlanner()
        positions = {piece_id: (piece.x_mm, piece.y_mm)}
        targets   = {piece_id: (x_mm, y_mm)}
        commands  = planner.plan_moves(positions, targets)
        if commands:
            self._on_send_move_commands(commands)

    @pyqtSlot(int, float, float, int)
    def _on_looper_request_move(self, piece_id: int, x_mm: float, y_mm: float, move_time_ms: int) -> None:
        """Dispatch a move from the command looper.

        Mirrors right-click behavior: piece auto-rotates to face the target,
        then drives in a straight line.  Sends directly via _send_serial_move
        to avoid interfering with the wave dispatch timer.
        """
        cmd = MoveCommand(
            piece_id    = piece_id,
            target_x_mm = x_mm,
            target_y_mm = y_mm,
            duration_ms = move_time_ms,
        )
        if self._sim_mode:
            self._simulator.speed_mm_s = self._debug_tab.simulator_speed_mm_s
            self._simulator.queue_moves([cmd])
        else:
            self._send_serial_move(cmd)

    @pyqtSlot(list)
    def _on_send_move_commands(self, commands: List[MoveCommand]) -> None:
        """Dispatch commands in sequence-number waves.

        All commands sharing the same sequence_num are sent in parallel as one
        wave. Later waves are delayed until the active wave is considered done.
        """
        if not commands:
            return

        self._reset_wave_dispatch()
        self._pending_waves = self._group_by_sequence(commands)
        self._dispatch_next_wave()

    @pyqtSlot(bytes)
    def _on_debug_send_raw(self, data: bytes) -> None:
        """Route a raw bytes send from the debug tab to serial or simulator."""
        if self._sim_mode:
            cmd = self._parse_mov_bytes(data)
            if cmd is not None:
                self._simulator.speed_mm_s = self._debug_tab.simulator_speed_mm_s
                self._simulator.queue_moves([cmd])
        else:
            self._handler.send(data)

    @pyqtSlot(list, dict)
    def _on_plan_visualized(self, commands: list, positions: dict) -> None:
        """Forward planned move commands to the board for arrow visualization."""
        self._board_widget.set_plan_visualization(commands or None, positions)

    @pyqtSlot(bool)
    def _on_sim_collision_changed(self, enabled: bool) -> None:
        self._simulator.collision_enabled = enabled

    @pyqtSlot(bool)
    def _on_sim_mode_changed(self, enabled: bool) -> None:
        self._reset_wave_dispatch()
        self._sim_mode = enabled
        if enabled:
            self._status_bar.showMessage('Simulator mode active')
        else:
            self._simulator.stop_all()
            if self._handler.is_connected:
                self._status_bar.showMessage('Connected')
            else:
                self._status_bar.showMessage('Not connected')

    @pyqtSlot()
    def _on_randomize_positions(self) -> None:
        """Scatter all active pieces to random positions with ≥5 mm spacing."""
        r     = float(PIECES.CIRCLE_RADIUS_MM)
        gap   = 5.0          # minimum edge-to-edge gap between any two pieces
        min_d = 2.0 * r + gap

        x_lo = float(SIMULATOR.X_MIN_MM) + r
        x_hi = float(SIMULATOR.X_MAX_MM) - r
        y_lo = float(SIMULATOR.Y_MIN_MM) + r
        y_hi = float(SIMULATOR.Y_MAX_MM) - r

        placed: List[Tuple[float, float]] = []
        max_attempts = 10_000

        for piece in self._board.active_pieces():
            for _ in range(max_attempts):
                cx = random.uniform(x_lo, x_hi)
                cy = random.uniform(y_lo, y_hi)
                if all(math.hypot(cx - px, cy - py) >= min_d for px, py in placed):
                    placed.append((cx, cy))
                    self._board.update_piece_position(piece.piece_id, cx, cy, piece.orientation_deg, 0.0)
                    self._last_seen[piece.piece_id] = time.monotonic()
                    break

        self._board_widget.refresh()
        self._debug_tab.on_sim_log('SIM: positions randomized')
        
    @pyqtSlot(str)
    def _on_set_fen_positions(self, fen: str) -> None:
        fen_positions = fen.split(" ")[0]
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
                
        for current_piece_index,  piece in enumerate(self._board.active_pieces()):
            piece_found = False
            for index, (char, rank, file) in enumerate(piece_cords):
                color = "black" if char.islower() else "white"
                piece_letter = piece.rank[0] if piece.rank != "knight" else "n"
                if piece_letter == char.lower() and piece.color == color:
                    print(rank, file)
                    self._board.update_piece_position(piece.piece_id, rankFileToMM(file) + PIECES._S, rankFileToMM(rank), piece.orientation_deg, 0.0)
                    piece_cords.pop(index)
                    self._last_seen[piece.piece_id] = time.monotonic()
                    piece_found = True
                    break
            if not piece_found:
                self._board.update_piece_position(piece.piece_id, PIECES.GRAVEYARD_POSITIONS[piece.piece_id][0], PIECES.GRAVEYARD_POSITIONS[piece.piece_id][1], piece.orientation_deg, 0.0)
                    
               
                    
            

    @pyqtSlot(bool)
    def _on_connection_changed(self, connected: bool) -> None:
        if connected:
            self._conn_status_label.setText('Status: Connected')
            self._conn_status_label.setStyleSheet('color: #7cbb7c; font-weight: bold;')
            self._status_bar.showMessage('Connected')
            self._poll_timer.start()
        else:
            self._conn_status_label.setText('Status: Disconnected')
            self._conn_status_label.setStyleSheet('color: #787878;')
            self._status_bar.showMessage('Disconnected')
            self._poll_timer.stop()
        self._btn_connect.setEnabled(not connected)
        self._btn_disconnect.setEnabled(connected)

    @pyqtSlot(str)
    def _on_serial_error(self, message: str) -> None:
        self._conn_status_label.setText(f'Error: {message}')
        self._conn_status_label.setStyleSheet('color: #f48771; font-weight: bold;')
        self._btn_connect.setEnabled(True)
        self._btn_disconnect.setEnabled(False)
        self._status_bar.showMessage(f'Serial error: {message}')

    @pyqtSlot(bool)
    def _on_hide_stale_changed(self, hide: bool) -> None:
        self._hide_stale = hide
        self._on_stale_check_timer()  # apply immediately

    @pyqtSlot(int)
    def _on_poll_interval_changed(self, interval_ms: int) -> None:
        self._poll_timer.setInterval(interval_ms)

    @pyqtSlot(bool)
    def _on_poll_enabled_changed(self, enabled: bool) -> None:
        self._poll_enabled = enabled

    @pyqtSlot(int)
    def _on_poll_target_changed(self, target: int) -> None:
        self._poll_target = target

    @pyqtSlot()
    def _on_stale_check_timer(self) -> None:
        """Recompute the stale set and push it to the board widget."""
        # In simulation mode all piece positions are synthetic, so nothing
        # is ever truly "stale" — skip the greying-out logic entirely.
        if self._sim_mode:
            self._board_widget.set_stale_pieces(set(), self._hide_stale)
            return
        now = time.monotonic()
        # A piece is stale if it has never been seen or hasn't been seen in >5 s
        stale: Set[int] = {
            p.piece_id
            for p in self._board.all_pieces()
            if not p.is_captured
            and (
                p.piece_id not in self._last_seen
                or now - self._last_seen[p.piece_id] > 5.0
            )
        }
        self._board_widget.set_stale_pieces(stale, self._hide_stale)

    @pyqtSlot()
    def _on_poll_timer(self) -> None:
        """Periodically request a position update from all bots."""
        if self._handler.is_connected and self._poll_enabled:
            self._handler.send(build_position_request(self._poll_target))

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _refresh_ports(self) -> None:
        current = self._port_combo.currentText()
        self._port_combo.clear()
        ports = SerialHandler.available_ports()
        for p in ports:
            self._port_combo.addItem(p)
        if current in ports:
            self._port_combo.setCurrentText(current)

    def _on_connect(self) -> None:
        port = self._port_combo.currentText()
        baud = self._baud_combo.currentData() or COMM.DEFAULT_BAUD_RATE
        if not port:
            self._conn_status_label.setText('Error: No port selected')
            self._conn_status_label.setStyleSheet('color: red;')
            return
        self._conn_status_label.setText('Status: Connecting…')
        self._conn_status_label.setStyleSheet('color: orange;')
        self._btn_connect.setEnabled(False)
        self._handler.connect_port(port, baud)

    def _on_disconnect(self) -> None:
        self._handler.disconnect_port()

    @staticmethod
    def _parse_mov_bytes(data: bytes) -> Optional[MoveCommand]:
        """Parse a position command byte string back into a MoveCommand for the simulator.

        Format: >1,{id},{x_mm},{y_mm},{theta_rad},{duration_ms}\\n
        Returns None if the data is not a valid position command.
        """
        from config import COMM
        try:
            line = data.decode(COMM.ENCODING).strip()
            if not line.startswith(COMM.MSG_PREFIX):
                return None
            parts = line[len(COMM.MSG_PREFIX):].split(COMM.DELIMITER)
            if int(parts[0]) != COMM.CMD_POSITION or len(parts) != 6:
                return None
            return MoveCommand(
                piece_id     = int(parts[1], 0),
                target_x_mm  = float(parts[2]),
                target_y_mm  = float(parts[3]),
                target_theta = math.degrees(float(parts[4])),
                duration_ms  = int(parts[5]),
            )
        except (ValueError, IndexError, UnicodeDecodeError):
            return None

    # ------------------------------------------------------------------
    # Wave dispatcher
    # ------------------------------------------------------------------

    @staticmethod
    def _group_by_sequence(commands: List[MoveCommand]) -> List[List[MoveCommand]]:
        grouped: Dict[int, List[MoveCommand]] = {}
        for cmd in sorted(commands, key=lambda c: (c.sequence_num, c.piece_id)):
            grouped.setdefault(cmd.sequence_num, []).append(cmd)
        return [grouped[seq] for seq in sorted(grouped.keys())]

    def _reset_wave_dispatch(self) -> None:
        self._wave_timer.stop()
        self._pending_waves = []
        self._active_wave_piece_ids.clear()
        self._wave_running = False

    def _dispatch_next_wave(self) -> None:
        if not self._pending_waves:
            self._wave_running = False
            self._active_wave_piece_ids.clear()
            return

        wave = self._pending_waves.pop(0)
        self._wave_running = True

        if self._sim_mode:
            self._dispatch_wave_sim(wave)
        else:
            self._dispatch_wave_serial(wave)

    def _dispatch_wave_sim(self, wave: List[MoveCommand]) -> None:
        self._simulator.speed_mm_s = self._debug_tab.simulator_speed_mm_s
        self._active_wave_piece_ids = {cmd.piece_id for cmd in wave}
        self._simulator.queue_moves(wave)

        # Empty wave guard.
        if not self._active_wave_piece_ids:
            self._dispatch_next_wave()

    def _dispatch_wave_serial(self, wave: List[MoveCommand]) -> None:
        max_duration_ms = 0
        for cmd in wave:
            est_ms = self._send_serial_move(cmd)
            if est_ms > max_duration_ms:
                max_duration_ms = est_ms

        # Gate the next wave with a conservative timing window.
        slack_ms = int(PLANNING.CONFLICT_SERIAL_WAVE_SLACK_MS)
        wait_ms = max(1, max_duration_ms + slack_ms)
        self._wave_timer.start(wait_ms)

    def _send_serial_move(self, cmd: MoveCommand) -> int:
        piece = self._board.get_piece(cmd.piece_id)
        if piece is None:
            theta = cmd.target_theta if cmd.target_theta is not None else 0.0
            self._handler.send(build_position_command(
                cmd.piece_id,
                cmd.target_x_mm,
                cmd.target_y_mm,
                theta,
                cmd.duration_ms,
            ))
            return max(1, cmd.duration_ms)

        dx = cmd.target_x_mm - piece.x_mm
        dy = cmd.target_y_mm - piece.y_mm
        dist = math.hypot(dx, dy)

        if dist > 1.0:
            fwd = math.degrees(math.atan2(dy, dx)) % 360.0
            bwd = (fwd + 180.0) % 360.0
            diff_fwd = abs((fwd - piece.orientation_deg + 180.0) % 360.0 - 180.0)
            diff_bwd = abs((bwd - piece.orientation_deg + 180.0) % 360.0 - 180.0)
            rotate_theta = fwd if diff_fwd <= diff_bwd else bwd
        else:
            rotate_theta = piece.orientation_deg

        rot_angle_deg = abs((rotate_theta - piece.orientation_deg + 180.0) % 360.0 - 180.0)
        rot_duration_ms = 0
        if rot_angle_deg > 0.5:
            rot_duration_ms = max(1, int(
                math.radians(rot_angle_deg)
                / SIMULATOR.ROTATION_ANGULAR_VEL_RAD_S * 1000
            ))
            self._handler.send(build_position_command(
                cmd.piece_id,
                piece.x_mm,
                piece.y_mm,
                rotate_theta,
                rot_duration_ms,
            ))

        self._handler.send(build_position_command(
            cmd.piece_id,
            cmd.target_x_mm,
            cmd.target_y_mm,
            rotate_theta,
            cmd.duration_ms,
        ))
        return rot_duration_ms + max(1, cmd.duration_ms)

    @pyqtSlot(int)
    def _on_sim_move_complete(self, piece_id: int) -> None:
        if not self._wave_running or not self._sim_mode:
            return
        self._active_wave_piece_ids.discard(piece_id)
        if not self._active_wave_piece_ids:
            self._dispatch_next_wave()

    @pyqtSlot()
    def _on_wave_timer_timeout(self) -> None:
        if self._sim_mode:
            return
        if not self._wave_running:
            return
        self._dispatch_next_wave()
