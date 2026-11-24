"""
gui/tabs/ota_update_tab.py  —  MiniBot Chess Swarm Coordinator

OTA Update tab:
  - Firmware file picker (native file-open dialog)
  - Tile grid of all 34 bots: checkbox + color-coded OTA status dot
  - Begin OTA / Cancel OTA buttons

Status dot colors: grey=none, yellow=pending, orange=active, green=success,
red=failed. Cancelling a rollout reverts pending/active dots back to grey;
bots that already succeeded or failed keep their color. Cancelling while the
firmware image is still streaming aborts the upload and reverts pending dots
so the cancelled rollout cannot resurrect itself via a late
``upload_finished(True, ...)`` callback. A serial disconnect likewise aborts an
in-flight upload/rollout: ``OtaManager`` owns the ``SerialHandler``, so the tab
listens to the re-emitted ``serial_connection_changed`` instead of receiving the
handler itself — ``SerialHandler.send()``/``send_chunked()`` are silent no-ops
when the port is down, which used to make a dead link look healthy.
"""

from __future__ import annotations

import os
from typing import Callable, Dict, List, Optional, Set

from PyQt6.QtWidgets import (
    QCheckBox,
    QFileDialog,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QVBoxLayout,
    QWidget,
)

from comms.ota_manager import OtaManager
from config import OTA, PIECES

_STATUS_COLORS = {
    "none": "#555555",
    "pending": "#d4b106",
    "active": "#ff8c00",
    "success": "#4caf50",
    "failed": "#e05656",
}

_TILE_COLS = 9  # back rank + extra queen per side


class _BotTile(QWidget):
    """One bot's checkbox + OTA status dot, used in the tile grid."""

    def __init__(self, piece_id: int, rank: str, parent: Optional[QWidget] = None) -> None:
        super().__init__(parent)
        self.piece_id = piece_id
        self.status = "none"

        layout = QHBoxLayout(self)
        layout.setContentsMargins(4, 2, 4, 2)
        layout.setSpacing(5)

        self._dot = QLabel()
        self._dot.setFixedSize(11, 11)
        layout.addWidget(self._dot)

        self.checkbox = QCheckBox(f"0x{piece_id:02X}")
        self.checkbox.setToolTip(rank.capitalize())
        layout.addWidget(self.checkbox)
        layout.addStretch()

        self.set_status("none")

    def set_status(self, status: str) -> None:
        self.status = status
        color = _STATUS_COLORS.get(status, _STATUS_COLORS["none"])
        self._dot.setStyleSheet(
            f"background-color: {color}; border-radius: 5px; border: 1px solid #1a1a1a;"
        )

    def is_checked(self) -> bool:
        return self.checkbox.isChecked()

    def set_checked(self, checked: bool) -> None:
        self.checkbox.setChecked(checked)


class OtaUpdateTab(QWidget):
    """Fleet firmware update panel.

    Args:
        ota_manager:    shared OtaManager instance (owns the serial protocol)
        get_active_ids: callable returning piece IDs currently reporting
                        fresh (non-stale) positions, used to pre-check bots
                        the first time this tab is shown
    """

    def __init__(
        self,
        ota_manager: OtaManager,
        get_active_ids: Callable[[], Set[int]],
        parent: Optional[QWidget] = None,
    ) -> None:
        super().__init__(parent)
        self._manager = ota_manager
        self._get_active_ids = get_active_ids
        self._firmware_path: Optional[str] = None
        self._auto_checked_once = False
        self._pending_targets: List[int] = []
        self._rollout_total = 0  # snapshot of target count taken at rollout start
        self._completed_count = 0
        # Reason an in-flight upload/rollout was killed by the Coordinator
        # itself (serial disconnect). Surfaced on the summary line so the user
        # can tell "the link went down" apart from a bot/server failure.
        self._abort_reason: Optional[str] = None
        self._tiles: Dict[int, _BotTile] = {}

        self._build_ui()
        self._wire_manager_signals()

    # ------------------------------------------------------------------
    # UI construction
    # ------------------------------------------------------------------

    def _build_ui(self) -> None:
        root = QVBoxLayout(self)
        root.setSpacing(10)

        # --- Firmware file picker ---
        file_group = QGroupBox("Firmware Image")
        fl = QVBoxLayout(file_group)
        row = QHBoxLayout()
        self._btn_browse = QPushButton("Browse...")
        self._btn_browse.clicked.connect(self._on_browse)
        row.addWidget(self._btn_browse)
        self._file_label = QLabel("No file selected")
        self._file_label.setStyleSheet("color: #888888;")
        row.addWidget(self._file_label, stretch=1)
        fl.addLayout(row)
        root.addWidget(file_group)

        # --- Bot tile grid ---
        bots_group = QGroupBox("Bots")
        bl = QVBoxLayout(bots_group)
        bl.addWidget(self._build_color_grid("White", PIECES.WHITE_IDS))
        bl.addWidget(self._build_color_grid("Black", PIECES.BLACK_IDS))
        root.addWidget(bots_group, stretch=1)

        # --- Action buttons ---
        btn_row = QHBoxLayout()
        self._btn_begin = QPushButton("Begin OTA")
        self._btn_begin.setMinimumHeight(36)
        self._btn_begin.clicked.connect(self._on_begin)
        btn_row.addWidget(self._btn_begin)

        self._btn_cancel = QPushButton("Cancel OTA")
        self._btn_cancel.setMinimumHeight(36)
        self._btn_cancel.setEnabled(False)
        self._btn_cancel.clicked.connect(self._on_cancel)
        btn_row.addWidget(self._btn_cancel)
        root.addLayout(btn_row)

        self._status_label = QLabel("")
        self._status_label.setStyleSheet("color: #888888;")
        root.addWidget(self._status_label)

    def _build_color_grid(self, title: str, ids) -> QGroupBox:
        group = QGroupBox(title)
        grid = QGridLayout(group)
        grid.setSpacing(4)
        for index, piece_id in enumerate(sorted(ids)):
            rank = PIECES.PIECE_RANKS.get(piece_id, "")
            tile = _BotTile(piece_id, rank)
            self._tiles[piece_id] = tile
            grid.addWidget(tile, index // _TILE_COLS, index % _TILE_COLS)
        return group

    # ------------------------------------------------------------------
    # Qt overrides
    # ------------------------------------------------------------------

    def showEvent(self, event) -> None:
        super().showEvent(event)
        # Only auto-checkmark the first time the panel is shown, so flipping
        # back to this tab later doesn't clobber a manual selection.
        if not self._auto_checked_once:
            self._auto_checked_once = True
            self._check_active_bots()

    # ------------------------------------------------------------------
    # Firmware selection
    # ------------------------------------------------------------------

    def _on_browse(self) -> None:
        path, _ = QFileDialog.getOpenFileName(
            self, "Select firmware image", "", "Firmware image (*.bin);;All files (*)"
        )
        if not path:
            return
        self._firmware_path = path
        try:
            size = os.path.getsize(path)
            self._file_label.setText(f"{os.path.basename(path)} ({size:,} bytes)")
        except OSError:
            self._file_label.setText(os.path.basename(path))
        self._file_label.setStyleSheet("color: #d0d0d0;")

    def _check_active_bots(self) -> None:
        active_ids = self._get_active_ids()
        for piece_id, tile in self._tiles.items():
            tile.set_checked(piece_id in active_ids)

    # ------------------------------------------------------------------
    # Rollout control
    # ------------------------------------------------------------------

    def _on_begin(self) -> None:
        if not self._firmware_path:
            self._status_label.setText("Select a firmware file first.")
            self._status_label.setStyleSheet("color: #e05656;")
            return

        checked_ids = sorted(pid for pid, tile in self._tiles.items() if tile.is_checked())
        if not checked_ids:
            self._status_label.setText("Select at least one bot.")
            self._status_label.setStyleSheet("color: #e05656;")
            return

        # Refuse before touching any state: SerialHandler.send()/send_chunked()
        # are silent no-ops on a dead link, so without this check the upload
        # looks healthy for OTA.UPLOAD_READY_TIMEOUT_S and then blames the
        # Server ("no READY received") for what is really an unplugged port.
        if not self._manager.serial_connected():
            self._status_label.setText(
                "Serial port not connected to the Server - connect it before "
                "starting an OTA."
            )
            self._status_label.setStyleSheet("color: #e05656;")
            return

        # Read the image here before flipping Begin/Cancel or entering the
        # pending state so:
        #   * an unreadable file (deleted / moved / locked / a directory, or
        #     an SMB path that went away) surfaces as an explicit error
        #     instead of raising inside the Qt ``clicked`` callback and
        #     leaving every dot pending with Cancel as the only escape;
        #   * the label shown at Begin-time matches the payload we will
        #     actually stream (the Browse-time ``os.path.getsize`` and a
        #     second read inside ``upload_firmware`` used to be able to
        #     disagree if the file changed since Browse).
        # ``upload_firmware`` now takes bytes, so this single read is the
        # sole source of both the displayed size and the CRC.
        try:
            with open(self._firmware_path, "rb") as f:
                firmware_bytes = f.read()
        except OSError as exc:
            self._status_label.setText(
                f"Could not read firmware file: {exc}"
            )
            self._status_label.setStyleSheet("color: #e05656;")
            return

        # Refresh the Browse-time label against the actual bytes read.
        self._file_label.setText(
            f"{os.path.basename(self._firmware_path)} "
            f"({len(firmware_bytes):,} bytes)"
        )
        self._file_label.setStyleSheet("color: #d0d0d0;")

        self._pending_targets = checked_ids
        self._completed_count = 0
        self._abort_reason = None
        for piece_id, tile in self._tiles.items():
            tile.set_status("pending" if piece_id in checked_ids else "none")

        self._btn_begin.setEnabled(False)
        self._btn_cancel.setEnabled(True)
        self._status_label.setStyleSheet("color: #888888;")
        self._status_label.setText("Uploading firmware to server...")
        self._manager.upload_firmware(firmware_bytes)

    def _on_cancel(self) -> None:
        # Abort an in-flight upload first. When an upload is in flight,
        # ``abort_upload`` clears the pending bytes/timers and emits
        # ``upload_finished(False, "cancelled by user", 0)``, which drives
        # ``_on_upload_finished`` and reverts the pending tiles. It is a
        # no-op when no upload is running.
        self._manager.abort_upload()
        # Then cancel any rollout in progress (also a no-op if none is active).
        outstanding = self._manager.cancel_rollout()
        for piece_id in outstanding:
            tile = self._tiles.get(piece_id)
            if tile is not None:
                tile.set_status("none")
        self._pending_targets = []
        self._abort_reason = None
        self._btn_begin.setEnabled(True)
        self._btn_cancel.setEnabled(False)
        self._status_label.setText("OTA cancelled.")
        self._status_label.setStyleSheet("color: #888888;")

    # ------------------------------------------------------------------
    # OtaManager signal handlers
    # ------------------------------------------------------------------

    def _wire_manager_signals(self) -> None:
        self._manager.upload_progress.connect(self._on_upload_progress)
        self._manager.upload_bytes_progress.connect(self._on_upload_bytes_progress)
        self._manager.upload_finished.connect(self._on_upload_finished)
        self._manager.bot_dispatched.connect(self._on_bot_dispatched)
        self._manager.bot_result.connect(self._on_bot_result)
        self._manager.rollout_finished.connect(self._on_rollout_finished)
        # Connection state: OtaManager re-emits SerialHandler.connection_changed
        # and already fails whatever is in flight; the tab only needs to say
        # why the dots changed.
        self._manager.serial_connection_changed.connect(
            self._on_serial_connection
        )

    def _on_serial_connection(self, connected: bool) -> None:
        if connected:
            if self._abort_reason:
                # Link came back; keep the last outcome text but stop labelling
                # it as a disconnect-induced abort.
                self._abort_reason = None
            return

        # Only complain about in-flight work when something actually is:
        # a plain disconnect with no upload/rollout running must not look like
        # a failure, and must not overwrite a previous run's summary.
        in_flight = any(
            tile.status in ("pending", "active") for tile in self._tiles.values()
        )
        if not in_flight:
            return
        self._abort_reason = "serial port disconnected"
        self._status_label.setText(
            "Serial port disconnected - any in-flight upload/rollout is being "
            "failed."
        )
        self._status_label.setStyleSheet("color: #e05656;")
        self._btn_begin.setEnabled(True)
        self._btn_cancel.setEnabled(False)

    def _on_upload_progress(self, message: str) -> None:
        self._status_label.setText(message)

    def _on_upload_bytes_progress(self, sent: int, total: int) -> None:
        if total <= 0:
            return
        pct = (sent / total) * 100.0
        self._status_label.setText(
            f"Uploading firmware: {sent:,} / {total:,} bytes ({pct:.0f}%)"
        )

    def _on_upload_finished(self, success: bool, message: str, crc32: int) -> None:
        if not success:
            self._status_label.setText(f"Upload failed: {message}")
            self._status_label.setStyleSheet("color: #e05656;")
            for piece_id in self._pending_targets:
                self._tiles[piece_id].set_status("none")
            self._pending_targets = []
            self._btn_begin.setEnabled(True)
            self._btn_cancel.setEnabled(False)
            return

        # Snapshot the target list so a stale reference cannot be replayed
        # if upload_finished ever fires again (e.g. a leaked timeout timer on
        # a subsequent upload). ``_pending_targets`` is cleared right after
        # the rollout is kicked off.
        targets = list(self._pending_targets)
        self._pending_targets = []
        self._rollout_total = len(targets)
        self._status_label.setText(f"Rolling out to {self._rollout_total} bot(s)...")
        self._manager.start_rollout(targets, version=crc32)

    def _on_bot_dispatched(self, piece_id: int) -> None:
        # Reached only on the Server's accept ack (>203,{id}), so an orange dot
        # now means "this bot's OTA command is queued on the Server" rather
        # than "the Coordinator wrote some bytes". Bots that are still queued
        # but not yet accepted stay pending (grey) until the ack, a per-bot
        # NACK, or the per-bot timeout resolves them.
        tile = self._tiles.get(piece_id)
        if tile is not None:
            tile.set_status("active")
        self._status_label.setStyleSheet("color: #888888;")
        self._status_label.setText(
            f"Server accepted bot 0x{piece_id:02X} - waiting for its result..."
        )

    def _on_bot_result(self, piece_id: int, success: bool, err_code: int) -> None:
        tile = self._tiles.get(piece_id)
        if tile is not None:
            tile.set_status("success" if success else "failed")

        self._completed_count += 1
        # Use the snapshot taken at rollout start: ``_pending_targets`` is
        # cleared in ``_on_upload_finished`` once the rollout begins so a
        # stale reference cannot resurrect the previous run.
        total = self._rollout_total
        if success:
            outcome = "succeeded"
        elif err_code == OTA.ERR_CODE_TIMEOUT:
            outcome = "timed out waiting for a result"
        elif err_code == OTA.ERR_CODE_NO_CONNECTION:
            outcome = "aborted - serial port disconnected"
        else:
            outcome = f"failed (code {err_code})"
        self._status_label.setStyleSheet("color: #888888;")
        self._status_label.setText(
            f"Bot 0x{piece_id:02X}: {outcome}  \u2014  {self._completed_count}/{total} complete"
        )

    def _on_rollout_finished(self, results: dict) -> None:
        succeeded = sum(1 for ok in results.values() if ok)
        total = len(results)
        # Prefix a Coordinator-side abort (serial disconnect) so the summary
        # does not read like every bot refused the update on its own.
        prefix = f"{self._abort_reason} - " if self._abort_reason else ""
        self._abort_reason = None
        self._status_label.setText(
            f"{prefix}OTA complete: {succeeded}/{total} succeeded."
        )
        self._status_label.setStyleSheet(
            "color: #7cbb7c;" if total and succeeded == total else "color: #e0a456;"
        )
        self._btn_begin.setEnabled(True)
        self._btn_cancel.setEnabled(False)
