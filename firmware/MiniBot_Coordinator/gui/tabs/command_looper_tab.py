"""
gui/tabs/command_looper_tab.py  —  MiniBot Chess Swarm Coordinator

Command Looper tab:
  - Form to add position commands (x, y, orientation, move time, command delay)
  - List view of queued commands with remove capability
  - Enable/disable toggle that continuously loops through the command list
  - Command delay: how long (ms) to wait after the previous command before
    sending the next one
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional

from PyQt6.QtCore import QTimer, pyqtSignal, pyqtSlot
from PyQt6.QtWidgets import (
    QAbstractItemView, QDoubleSpinBox, QGroupBox, QHBoxLayout, QLabel,
    QListWidget, QListWidgetItem, QPushButton, QSpinBox, QSizePolicy,
    QSplitter, QVBoxLayout, QWidget, QCheckBox,
)
from PyQt6.QtCore import Qt

from comms.protocol import build_position_command


@dataclass
class LoopCommand:
    """One entry in the command loop list."""
    x_mm:        float
    y_mm:        float
    theta_deg:   float
    move_time_ms: int
    delay_ms:    int  # wait after previous command before sending this one

    def display_text(self) -> str:
        return (
            f"X={self.x_mm:.1f}  Y={self.y_mm:.1f}  "
            f"θ={self.theta_deg:.1f}°  "
            f"Move={self.move_time_ms} ms  "
            f"Delay={self.delay_ms} ms"
        )


class CommandLooperTab(QWidget):
    """Repeatedly loop through a list of position commands.

    Signals:
        send_raw(bytes) — emit bytes to the serial handler
    """

    send_raw = pyqtSignal(bytes)

    _BROADCAST_ID = 0xFF

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._commands: List[LoopCommand] = []
        self._loop_index: int = 0
        self._looping: bool = False

        # Timer fires after each command delay; single-shot so we control
        # the interval dynamically.
        self._loop_timer = QTimer(self)
        self._loop_timer.setSingleShot(True)
        self._loop_timer.timeout.connect(self._on_loop_tick)

        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(8)

        # ── Command entry form ────────────────────────────────────────
        entry_group = QGroupBox('Add Command')
        form_layout = QVBoxLayout(entry_group)
        form_layout.setSpacing(6)

        # Row 1: X / Y
        row1 = QHBoxLayout()
        row1.addWidget(QLabel('X (mm):'))
        self._spin_x = QDoubleSpinBox()
        self._spin_x.setRange(-9999.0, 9999.0)
        self._spin_x.setDecimals(1)
        self._spin_x.setSingleStep(10.0)
        self._spin_x.setValue(0.0)
        row1.addWidget(self._spin_x)
        row1.addSpacing(12)
        row1.addWidget(QLabel('Y (mm):'))
        self._spin_y = QDoubleSpinBox()
        self._spin_y.setRange(-9999.0, 9999.0)
        self._spin_y.setDecimals(1)
        self._spin_y.setSingleStep(10.0)
        self._spin_y.setValue(0.0)
        row1.addWidget(self._spin_y)
        row1.addStretch()
        form_layout.addLayout(row1)

        # Row 2: Orientation
        row2 = QHBoxLayout()
        row2.addWidget(QLabel('Orientation (°):'))
        self._spin_theta = QDoubleSpinBox()
        self._spin_theta.setRange(-360.0, 360.0)
        self._spin_theta.setDecimals(1)
        self._spin_theta.setSingleStep(15.0)
        self._spin_theta.setValue(0.0)
        row2.addWidget(self._spin_theta)
        row2.addStretch()
        form_layout.addLayout(row2)

        # Row 3: Move time / Command delay
        row3 = QHBoxLayout()
        row3.addWidget(QLabel('Move Time (ms):'))
        self._spin_move_time = QSpinBox()
        self._spin_move_time.setRange(1, 60000)
        self._spin_move_time.setSingleStep(100)
        self._spin_move_time.setValue(1000)
        row3.addWidget(self._spin_move_time)
        row3.addSpacing(12)
        row3.addWidget(QLabel('Command Delay (ms):'))
        self._spin_delay = QSpinBox()
        self._spin_delay.setRange(0, 60000)
        self._spin_delay.setSingleStep(100)
        self._spin_delay.setValue(500)
        row3.addWidget(self._spin_delay)
        row3.addStretch()
        form_layout.addLayout(row3)

        # Add button
        btn_add_row = QHBoxLayout()
        self._btn_add = QPushButton('Add Command')
        self._btn_add.clicked.connect(self._on_add_command)
        btn_add_row.addWidget(self._btn_add)
        btn_add_row.addStretch()
        form_layout.addLayout(btn_add_row)

        root.addWidget(entry_group)

        # ── Command list ──────────────────────────────────────────────
        list_group = QGroupBox('Command List')
        list_layout = QVBoxLayout(list_group)

        self._list_widget = QListWidget()
        self._list_widget.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self._list_widget.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        list_layout.addWidget(self._list_widget)

        btn_list_row = QHBoxLayout()
        self._btn_remove = QPushButton('Remove Selected')
        self._btn_remove.clicked.connect(self._on_remove_command)
        btn_list_row.addWidget(self._btn_remove)
        self._btn_clear = QPushButton('Clear All')
        self._btn_clear.clicked.connect(self._on_clear_commands)
        btn_list_row.addWidget(self._btn_clear)
        btn_list_row.addStretch()
        list_layout.addLayout(btn_list_row)

        root.addWidget(list_group, stretch=1)

        # ── Loop control ──────────────────────────────────────────────
        loop_group = QGroupBox('Loop Control')
        loop_layout = QVBoxLayout(loop_group)

        # Target ID row
        target_row = QHBoxLayout()
        target_row.addWidget(QLabel('Target ID (hex):'))
        self._spin_target_id = QSpinBox()
        self._spin_target_id.setRange(0x00, 0xFF)
        self._spin_target_id.setDisplayIntegerBase(16)
        self._spin_target_id.setPrefix('0x')
        self._spin_target_id.setValue(self._BROADCAST_ID)
        target_row.addWidget(self._spin_target_id)
        target_row.addStretch()
        loop_layout.addLayout(target_row)

        # Enable + status row
        enable_row = QHBoxLayout()
        self._loop_check = QCheckBox('Enable Looping')
        self._loop_check.setChecked(False)
        self._loop_check.toggled.connect(self._on_loop_toggled)
        enable_row.addWidget(self._loop_check)

        self._loop_status_label = QLabel('Stopped')
        self._loop_status_label.setStyleSheet('color: #787878;')
        enable_row.addSpacing(16)
        enable_row.addWidget(self._loop_status_label)
        enable_row.addStretch()
        loop_layout.addLayout(enable_row)

        root.addWidget(loop_group)

    # ------------------------------------------------------------------
    # Slots — command list management
    # ------------------------------------------------------------------

    @pyqtSlot()
    def _on_add_command(self) -> None:
        cmd = LoopCommand(
            x_mm=self._spin_x.value(),
            y_mm=self._spin_y.value(),
            theta_deg=self._spin_theta.value(),
            move_time_ms=self._spin_move_time.value(),
            delay_ms=self._spin_delay.value(),
        )
        self._commands.append(cmd)
        item = QListWidgetItem(f'[{len(self._commands)}]  {cmd.display_text()}')
        self._list_widget.addItem(item)

    @pyqtSlot()
    def _on_remove_command(self) -> None:
        row = self._list_widget.currentRow()
        if row < 0:
            return
        self._commands.pop(row)
        self._list_widget.takeItem(row)
        self._refresh_list_labels()

        # Keep loop index in bounds
        if self._loop_index >= len(self._commands):
            self._loop_index = 0

    @pyqtSlot()
    def _on_clear_commands(self) -> None:
        self._commands.clear()
        self._list_widget.clear()
        self._loop_index = 0
        if self._looping:
            self._stop_loop()
            self._loop_check.setChecked(False)

    # ------------------------------------------------------------------
    # Slots — loop control
    # ------------------------------------------------------------------

    @pyqtSlot(bool)
    def _on_loop_toggled(self, enabled: bool) -> None:
        if enabled:
            if not self._commands:
                # Nothing to loop — uncheck silently
                self._loop_check.blockSignals(True)
                self._loop_check.setChecked(False)
                self._loop_check.blockSignals(False)
                return
            self._start_loop()
        else:
            self._stop_loop()

    # ------------------------------------------------------------------
    # Loop machinery
    # ------------------------------------------------------------------

    def _start_loop(self) -> None:
        self._looping = True
        self._loop_index = 0
        self._loop_status_label.setText('Running…')
        self._loop_status_label.setStyleSheet('color: #7cbb7c; font-weight: bold;')
        # Fire immediately for the first command (delay before first = its own delay_ms)
        self._schedule_next(self._commands[self._loop_index].delay_ms)

    def _stop_loop(self) -> None:
        self._looping = False
        self._loop_timer.stop()
        self._loop_status_label.setText('Stopped')
        self._loop_status_label.setStyleSheet('color: #787878;')
        self._clear_highlight()

    def _schedule_next(self, delay_ms: int) -> None:
        """Start the single-shot timer for the given delay."""
        self._loop_timer.start(max(0, delay_ms))

    @pyqtSlot()
    def _on_loop_tick(self) -> None:
        """Send the current command, advance the index, schedule the next."""
        if not self._looping or not self._commands:
            return

        cmd = self._commands[self._loop_index]
        self.send_raw.emit(
            build_position_command(
                self._spin_target_id.value(),
                cmd.x_mm,
                cmd.y_mm,
                cmd.theta_deg,
                cmd.move_time_ms,
            )
        )
        self._highlight_row(self._loop_index)

        # Advance index (wrap around)
        self._loop_index = (self._loop_index + 1) % len(self._commands)

        # Next delay is the delay of the upcoming command
        next_delay = self._commands[self._loop_index].delay_ms
        self._schedule_next(next_delay)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _refresh_list_labels(self) -> None:
        for i, cmd in enumerate(self._commands):
            item = self._list_widget.item(i)
            if item:
                item.setText(f'[{i + 1}]  {cmd.display_text()}')

    def _highlight_row(self, row: int) -> None:
        from PyQt6.QtGui import QColor, QBrush
        for i in range(self._list_widget.count()):
            item = self._list_widget.item(i)
            if item:
                if i == row:
                    item.setBackground(QBrush(QColor('#3a4a3a')))
                    item.setForeground(QBrush(QColor('#c8e6c8')))
                else:
                    item.setBackground(QBrush(QColor('#272727')))
                    item.setForeground(QBrush(QColor('#d0d0d0')))

    def _clear_highlight(self) -> None:
        from PyQt6.QtGui import QColor, QBrush
        for i in range(self._list_widget.count()):
            item = self._list_widget.item(i)
            if item:
                item.setBackground(QBrush(QColor('#272727')))
                item.setForeground(QBrush(QColor('#d0d0d0')))
