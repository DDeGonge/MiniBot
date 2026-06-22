"""
gui/tabs/debug_tab.py  —  MiniBot Chess Swarm Coordinator

Debug Messaging tab:
  - Target ID (hex spinbox)
  - X / Y / theta / duration fields
  - "Send MOV" button
  - Response log (read-only text area fed by raw serial lines)
  - "Clear Log" button
"""

from __future__ import annotations

from typing import Optional, Tuple

from PyQt6.QtCore import pyqtSignal, pyqtSlot, Qt
from PyQt6.QtWidgets import (
    QCheckBox, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel, QPushButton,
    QSpinBox, QTextEdit, QVBoxLayout, QWidget,
)

from comms.protocol import build_position_command
from config import COMM, ELECTROMAGNETS, PIECES, PLANNING, SIMULATOR


class DebugTab(QWidget):
    """Debug messaging control panel tab.

    Signals:
        send_raw(bytes)                  — emit bytes to the serial handler
        simulator_mode_changed(bool)     — True when simulator mode is toggled on
        hide_stale_pieces_changed(bool)  — True to hide pieces with no recent position
    """

    send_raw                    = pyqtSignal(bytes)
    simulator_mode_changed      = pyqtSignal(bool)   # True = simulator active
    hide_stale_pieces_changed   = pyqtSignal(bool)   # True = hide pieces unseen >5s
    show_electromagnets_changed = pyqtSignal(bool)   # True = show electromagnet rings
    randomize_positions         = pyqtSignal()        # scatter all pieces randomly
    set_fen_postions            = pyqtSignal(str)
    sim_collision_changed       = pyqtSignal(bool)   # True = collision detection on
    clear_pending_moves         = pyqtSignal()        # cancel all in-flight sim moves

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(8)

        # --- Command fields ---
        cmd_group = QGroupBox('Send MOV Command')
        cl = QVBoxLayout(cmd_group)

        id_row = QHBoxLayout()
        id_row.addWidget(QLabel('Target ID (hex):'))
        self._id_spin = QSpinBox()
        self._id_spin.setRange(PIECES.WHITE_ID_START, PIECES.BLACK_ID_END)
        self._id_spin.setDisplayIntegerBase(16)
        self._id_spin.setPrefix('0x')
        self._id_spin.setValue(PIECES.WHITE_ID_START)
        id_row.addWidget(self._id_spin)
        id_row.addStretch()
        cl.addLayout(id_row)

        xy_row = QHBoxLayout()
        xy_row.addWidget(QLabel('X (mm):'))
        self._x_spin = QDoubleSpinBox()
        self._x_spin.setRange(-200.0, 600.0)
        self._x_spin.setDecimals(1)
        self._x_spin.setValue(0.0)
        xy_row.addWidget(self._x_spin)

        xy_row.addWidget(QLabel('Y (mm):'))
        self._y_spin = QDoubleSpinBox()
        self._y_spin.setRange(-200.0, 600.0)
        self._y_spin.setDecimals(1)
        self._y_spin.setValue(0.0)
        xy_row.addWidget(self._y_spin)
        cl.addLayout(xy_row)

        theta_dur_row = QHBoxLayout()
        theta_dur_row.addWidget(QLabel('θ (°):'))
        self._theta_spin = QDoubleSpinBox()
        self._theta_spin.setRange(0.0, 359.9)
        self._theta_spin.setDecimals(1)
        self._theta_spin.setValue(0.0)
        self._theta_spin.setWrapping(True)
        theta_dur_row.addWidget(self._theta_spin)

        theta_dur_row.addWidget(QLabel('Duration (s):'))
        self._dur_spin = QDoubleSpinBox()
        self._dur_spin.setRange(0.1, 60.0)
        self._dur_spin.setDecimals(2)
        self._dur_spin.setSingleStep(0.5)
        self._dur_spin.setValue(PLANNING.DEFAULT_MOVE_DURATION_MS / 1000.0)
        theta_dur_row.addWidget(self._dur_spin)
        cl.addLayout(theta_dur_row)

        self._btn_send = QPushButton('Send MOV')
        self._btn_send.setMinimumHeight(36)
        self._btn_send.clicked.connect(self._on_send)
        cl.addWidget(self._btn_send)

        root.addWidget(cmd_group)

        # --- Simulator ---
        sim_group = QGroupBox('Simulator')
        sl = QVBoxLayout(sim_group)

        self._sim_check = QCheckBox('Enable Simulator Mode')
        self._sim_check.setToolTip(
            'When enabled, move commands are simulated locally instead of\n'
            'being sent to the serial port.  Position is updated in real time\n'
            'at the configured speed with boundary and collision enforcement.'
        )
        self._sim_check.toggled.connect(self._on_sim_toggled)
        sl.addWidget(self._sim_check)

        speed_row = QHBoxLayout()
        speed_row.addWidget(QLabel('Sim speed (mm/s):'))
        self._sim_speed = QDoubleSpinBox()
        self._sim_speed.setRange(1.0, 500.0)
        self._sim_speed.setDecimals(1)
        self._sim_speed.setSingleStep(10.0)
        self._sim_speed.setValue(SIMULATOR.DEFAULT_SPEED_MM_S)
        self._sim_speed.setToolTip('Nominal robot speed used by the simulator')
        speed_row.addWidget(self._sim_speed)
        speed_row.addStretch()
        sl.addLayout(speed_row)

        self._btn_randomize = QPushButton('Randomize Positions')
        self._btn_randomize.setToolTip(
            'Scatter all active pieces to random positions, ensuring at least\n'
            '5 mm of spacing between any two pieces.'
        )
        self._btn_randomize.setEnabled(False)
        self._btn_randomize.clicked.connect(self.randomize_positions)
        sl.addWidget(self._btn_randomize)
        
        self._fen_input = QTextEdit('rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR w KQkq - 0 1')
        self._fen_input.setFixedHeight(30)
        sl.addWidget(self._fen_input)
        
        self._btn_fen_set = QPushButton('Set FEN to Board')
        self._btn_fen_set.clicked.connect(self._on_fen_set)
        sl.addWidget(self._btn_fen_set)

        self._collision_check = QCheckBox('Enable collision detection')
        self._collision_check.setChecked(True)
        self._collision_check.setEnabled(False)
        self._collision_check.setToolTip(
            'When unchecked, pieces pass through each other in the simulator.\n'
            'Useful for testing planners without false collision blocks.'
        )
        self._collision_check.toggled.connect(self.sim_collision_changed)
        sl.addWidget(self._collision_check)

        self._btn_clear_moves = QPushButton('Clear Pending Moves')
        self._btn_clear_moves.setToolTip(
            'Cancel all in-flight simulator moves immediately.\n'
            'Pieces stay at their current positions.'
        )
        self._btn_clear_moves.setEnabled(False)
        self._btn_clear_moves.clicked.connect(self.clear_pending_moves)
        sl.addWidget(self._btn_clear_moves)

        root.addWidget(sim_group)

        # --- Display options ---
        disp_group = QGroupBox('Display')
        dl = QVBoxLayout(disp_group)
        self._hide_stale_check = QCheckBox('Hide pieces with no recent position (>5 s)')
        self._hide_stale_check.setChecked(False)
        self._hide_stale_check.toggled.connect(self.hide_stale_pieces_changed)
        dl.addWidget(self._hide_stale_check)
        self._show_em_check = QCheckBox(
            f'Show electromagnet locations'
            f'  (OD {ELECTROMAGNETS.OD_MM:.0f} mm / ID {ELECTROMAGNETS.ID_MM:.0f} mm,'
            f'  {len(ELECTROMAGNETS.POSITIONS)} magnet{"s" if len(ELECTROMAGNETS.POSITIONS) != 1 else ""})'
        )
        self._show_em_check.setChecked(False)
        self._show_em_check.toggled.connect(self.show_electromagnets_changed)
        dl.addWidget(self._show_em_check)
        root.addWidget(disp_group)

        # --- Response log ---
        log_group = QGroupBox('Response Log')
        ll = QVBoxLayout(log_group)
        self._log = QTextEdit()
        self._log.setReadOnly(True)
        self._log.setPlaceholderText('Serial responses will appear here…')
        self._log.setMinimumHeight(200)
        ll.addWidget(self._log)

        btn_row = QHBoxLayout()
        self._btn_clear = QPushButton('Clear Log')
        self._btn_clear.clicked.connect(self._log.clear)
        btn_row.addWidget(self._btn_clear)
        btn_row.addStretch()
        ll.addLayout(btn_row)
        root.addWidget(log_group)

        root.addStretch()

    # ------------------------------------------------------------------
    # Public slots
    # ------------------------------------------------------------------

    @pyqtSlot(str)
    def on_raw_line_received(self, line: str) -> None:
        """Append a raw serial line to the response log."""
        self._log.append(f'← {line}')

    @pyqtSlot(int, str)
    def on_error_received(self, piece_id: int, reason: str) -> None:
        self._log.append(
            f'<span style="color:red">ERR 0x{piece_id:02X}: {reason}</span>'
        )

    @pyqtSlot(int)
    def on_ack_received(self, piece_id: int) -> None:
        self._log.append(
            f'<span style="color:green">ACK 0x{piece_id:02X}</span>'
        )

    # ------------------------------------------------------------------
    # Button handler
    # ------------------------------------------------------------------

    def _on_send(self) -> None:
        piece_id    = self._id_spin.value()
        x_mm        = self._x_spin.value()
        y_mm        = self._y_spin.value()
        theta_deg   = self._theta_spin.value()
        duration_ms = int(self._dur_spin.value() * 1000)

        data = build_position_command(piece_id, x_mm, y_mm, theta_deg, duration_ms)
        self._log.append(f'→ {data.decode(COMM.ENCODING).strip()}')
        self.send_raw.emit(data)
        
    def _on_fen_set(self) -> None:
        _fen_string = self._fen_input.toPlainText().strip()
        valid_fen, error = self.validate_fen(_fen_string)
        if valid_fen:
            self.set_fen_postions.emit(_fen_string)
        else:
            self._log.append(error)

    def _on_sim_toggled(self, enabled: bool) -> None:
        mode_str = 'ON' if enabled else 'OFF'
        self._log.append(f'[SIM] Simulator mode {mode_str}')
        self._btn_randomize.setEnabled(enabled)
        self._collision_check.setEnabled(enabled)
        self._btn_clear_moves.setEnabled(enabled)
        self.simulator_mode_changed.emit(enabled)

    # ------------------------------------------------------------------
    # Simulator log relay
    # ------------------------------------------------------------------

    @pyqtSlot(str)
    def on_sim_log(self, message: str) -> None:
        """Show simulator events in the response log."""
        self._log.append(f'<span style="color:#888">{message}</span>')

    # ------------------------------------------------------------------
    # Properties (read by MainWindow)
    # ------------------------------------------------------------------

    @property
    def is_simulator_enabled(self) -> bool:
        return self._sim_check.isChecked()

    @property
    def simulator_speed_mm_s(self) -> float:
        return self._sim_speed.value()


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