"""
gui/tabs/position_tracker_tab.py  —  MiniBot Chess Swarm Coordinator

Position Tracker tab:
  - QTableWidget showing all 34 robots with live-updating position data
  - Columns: ID | Color | Rank | X (mm) | Y (mm) | θ (°) | Last Update
  - "Poll Now" button to trigger an immediate position dump
  - Auto-updates when position_received signals arrive
"""

from __future__ import annotations

import datetime
from typing import Optional

from PyQt6.QtCore import pyqtSignal, pyqtSlot, Qt
from PyQt6.QtWidgets import (
    QHBoxLayout,
    QPushButton,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from comms.protocol import build_position_request
from config import GUI, PIECES
from models.piece import BoardState


class PositionTrackerTab(QWidget):
    """Live position and orientation tracker for all bots.

    Signals:
        send_raw(bytes)   — emit bytes to the serial handler (POLL command)
    """

    send_raw = pyqtSignal(bytes)

    _COL_ID = 0
    _COL_COLOR = 1
    _COL_RANK = 2
    _COL_X = 3
    _COL_Y = 4
    _COL_THETA = 5
    _COL_BATT = 6
    _COL_UPDATED = 7

    def __init__(
        self, board_state: BoardState, parent: Optional[QWidget] = None
    ) -> None:
        super().__init__(parent)
        self._board = board_state
        self._build_ui()
        self._populate_initial()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)

        # Toolbar
        btn_row = QHBoxLayout()
        self._btn_poll = QPushButton("Poll Now")
        self._btn_poll.setFixedWidth(90)
        self._btn_poll.clicked.connect(self._on_poll)
        btn_row.addWidget(self._btn_poll)
        btn_row.addStretch()
        root.addLayout(btn_row)

        # Table
        columns = GUI.TRACKER_COLUMNS
        self._table = QTableWidget(0, len(columns))
        self._table.setHorizontalHeaderLabels(columns)
        self._table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self._table.setSelectionBehavior(QTableWidget.SelectionBehavior.SelectRows)
        self._table.verticalHeader().setVisible(False)
        self._table.horizontalHeader().setStretchLastSection(True)
        self._table.horizontalHeader().setDefaultSectionSize(60)
        self._table.setSortingEnabled(True)
        self._table.sortByColumn(self._COL_ID, Qt.SortOrder.AscendingOrder)
        root.addWidget(self._table)

    # ------------------------------------------------------------------
    # Initial population
    # ------------------------------------------------------------------

    def _populate_initial(self) -> None:
        pieces = sorted(self._board.all_pieces(), key=lambda p: p.piece_id)
        self._table.setRowCount(len(pieces))
        for row, piece in enumerate(pieces):
            self._set_row(row, piece.piece_id)

    def _set_row(self, row: int, piece_id: int) -> None:
        piece = self._board.get_piece(piece_id)
        if piece is None:
            return

        updated = datetime.datetime.fromtimestamp(piece.last_updated).strftime(
            "%H:%M:%S.%f"
        )[:-3]

        values = [
            f"0x{piece.piece_id:02X}",
            piece.color.capitalize(),
            piece.rank.capitalize(),
            f"{piece.x_mm:.1f}",
            f"{piece.y_mm:.1f}",
            f"{piece.orientation_deg:.1f}",
            f"{piece.battery_v:.2f}" if piece.battery_v else "",
            updated,
        ]

        for col, val in enumerate(values):
            item = QTableWidgetItem(val)
            item.setTextAlignment(
                int(Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter)
            )
            if col == self._COL_ID:
                item.setData(Qt.ItemDataRole.UserRole, piece_id)
            self._table.setItem(row, col, item)

        # Color-code by side
        from PyQt6.QtGui import QColor, QBrush

        bg = QColor("#2a2a2a") if piece.color == "white" else QColor("#323232")
        text = QColor("#d0d0d0")
        for col in range(self._table.columnCount()):
            item = self._table.item(row, col)
            if item:
                item.setBackground(QBrush(bg))
                item.setForeground(QBrush(text))

    # ------------------------------------------------------------------
    # Public slots
    # ------------------------------------------------------------------

    @pyqtSlot(int, float, float, float, float)
    def on_position_received(
        self,
        piece_id: int,
        x_mm: float,
        y_mm: float,
        theta_deg: float,
        battery_v: float,
    ) -> None:
        """Update a row when a POS message arrives."""
        row = self._find_row(piece_id)
        if row is not None:
            self._set_row(row, piece_id)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _find_row(self, piece_id: int) -> Optional[int]:
        for row in range(self._table.rowCount()):
            item = self._table.item(row, self._COL_ID)
            if item and item.data(Qt.ItemDataRole.UserRole) == piece_id:
                return row
        return None

    def _on_poll(self) -> None:
        self.send_raw.emit(build_position_request(0xFF))
