"""
gui/tabs/system_control_tab.py  —  MiniBot Chess Swarm Coordinator

System Control tab:
  - COM port selector + baud rate + Connect / Disconnect buttons
  - Position update polling frequency (spinbox, ms)
  - Electromagnet mode slider (Off / On / Sync)
  - "Send Electromagnet Sync" button
  - Connection status label
"""

from __future__ import annotations

from typing import Optional

from PyQt6.QtCore import pyqtSignal, pyqtSlot, Qt, QTimer
from PyQt6.QtWidgets import (
    QCheckBox,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from comms.protocol import build_electromagnet, build_sync
from config import COMM, GUI


class SystemControlTab(QWidget):
    """System control panel tab.

    Signals:
        send_raw(bytes)            — emit bytes to the serial handler
        poll_interval_changed(int) — new poll interval in ms
        poll_enabled_changed(bool) — enable/disable auto-polling
        poll_target_changed(int)   — new poll target piece ID

    Note: serial connection controls live in the main window connection bar.
    """

    send_raw = pyqtSignal(bytes)
    poll_interval_changed = pyqtSignal(int)
    poll_enabled_changed = pyqtSignal(bool)
    poll_target_changed = pyqtSignal(int)

    def __init__(self, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self._sync_timer = QTimer(self)
        self._sync_timer.timeout.connect(self._on_mag_sync)
        self._build_ui()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(10)

        # --- Polling ---
        poll_group = QGroupBox("Position Polling")
        pl = QVBoxLayout(poll_group)

        self._poll_enable_check = QCheckBox("Enable polling")
        self._poll_enable_check.setChecked(True)
        self._poll_enable_check.toggled.connect(self.poll_enabled_changed)
        pl.addWidget(self._poll_enable_check)

        poll_row = QHBoxLayout()
        poll_row.addWidget(QLabel("Interval (ms):"))
        self._poll_spin = QSpinBox()
        self._poll_spin.setRange(100, 60000)
        self._poll_spin.setSingleStep(100)
        self._poll_spin.setValue(COMM.DEFAULT_POLL_INTERVAL_MS)
        self._poll_spin.valueChanged.connect(self._on_set_rate)
        poll_row.addWidget(self._poll_spin)
        poll_row.addStretch()
        pl.addLayout(poll_row)

        target_row = QHBoxLayout()
        target_row.addWidget(QLabel("Target ID (hex):"))
        self._poll_target_spin = QSpinBox()
        self._poll_target_spin.setRange(0x00, 0xFF)
        self._poll_target_spin.setDisplayIntegerBase(16)
        self._poll_target_spin.setPrefix("0x")
        self._poll_target_spin.setValue(0xFF)
        self._poll_target_spin.valueChanged.connect(self.poll_target_changed)
        target_row.addWidget(self._poll_target_spin)
        target_row.addStretch()
        pl.addLayout(target_row)

        root.addWidget(poll_group)

        # --- Electromagnets ---
        mag_group = QGroupBox("Electromagnets")
        ml = QVBoxLayout(mag_group)

        self._mag_check = QCheckBox("Enable Electromagnet")
        self._mag_check.setChecked(False)
        self._mag_check.toggled.connect(self._on_mag_check)
        ml.addWidget(self._mag_check)

        root.addWidget(mag_group)

        # --- Electromagnet Sync ---
        sync_group = QGroupBox("Electromagnet Sync")
        sl = QVBoxLayout(sync_group)

        self._btn_mag_sync = QPushButton("Send Sync")
        self._btn_mag_sync.setMinimumHeight(36)
        self._btn_mag_sync.clicked.connect(self._on_mag_sync)
        sl.addWidget(self._btn_mag_sync)

        auto_row = QHBoxLayout()
        self._auto_sync_check = QCheckBox("Auto-sync every")
        self._auto_sync_check.setChecked(False)
        self._auto_sync_check.toggled.connect(self._on_auto_sync_toggled)
        auto_row.addWidget(self._auto_sync_check)
        self._auto_sync_spin = QSpinBox()
        self._auto_sync_spin.setRange(1, 3600)
        self._auto_sync_spin.setSingleStep(1)
        self._auto_sync_spin.setValue(5)
        self._auto_sync_spin.setSuffix(" s")
        self._auto_sync_spin.valueChanged.connect(self._on_auto_sync_interval_changed)
        auto_row.addWidget(self._auto_sync_spin)
        auto_row.addStretch()
        sl.addLayout(auto_row)

        root.addWidget(sync_group)
        root.addStretch()

    # ------------------------------------------------------------------
    # Button / widget handlers
    # ------------------------------------------------------------------

    def _on_set_rate(self) -> None:
        interval_ms = self._poll_spin.value()
        self.poll_interval_changed.emit(interval_ms)

    def _on_mag_check(self, checked: bool) -> None:
        self.send_raw.emit(
            build_electromagnet(COMM.MAG_ON if checked else COMM.MAG_OFF)
        )

    def _on_mag_sync(self) -> None:
        self.send_raw.emit(build_sync())

    def _on_auto_sync_toggled(self, checked: bool) -> None:
        if checked:
            self._sync_timer.start(self._auto_sync_spin.value() * 1000)
        else:
            self._sync_timer.stop()

    def _on_auto_sync_interval_changed(self, value: int) -> None:
        if self._sync_timer.isActive():
            self._sync_timer.start(value * 1000)
