"""
comms/ota_manager.py  —  MiniBot Chess Swarm Coordinator

Drives fleet firmware updates on top of SerialHandler:
  1. Stage a firmware image on the Server (upload over USB serial, staged
     to LittleFS, served to bots over its WiFi AP).
  2. Roll it out to a list of bot IDs, dispatching up to `max_parallel`
     OTA_START commands at a time and waiting for each bot's result.

Usage:
    mgr = OtaManager(serial_handler)
    mgr.upload_finished.connect(...)
    mgr.rollout_progress.connect(...)
    mgr.rollout_finished.connect(...)
    mgr.upload_firmware("firmware.bin")
    # once upload_finished(True, "", crc32) fires:
    mgr.start_rollout(piece_ids=[0x01, 0x02, ...], version=crc32, max_parallel=1)
"""

from __future__ import annotations

import time
import zlib
from typing import Dict, List, Optional

from PyQt6.QtCore import QObject, QTimer, pyqtSignal

from comms.protocol import build_ota_start, build_ota_upload_begin
from comms.serial_handler import SerialHandler
from config import OTA


class OtaManager(QObject):
    """Coordinates firmware staging and fleet-wide rollout over serial."""

    # Upload (staging firmware on the Server)
    upload_progress = pyqtSignal(str)  # human-readable status text
    upload_bytes_progress = pyqtSignal(int, int)  # bytes sent, total bytes
    upload_finished = pyqtSignal(bool, str, int)  # success, error message, crc32

    # Rollout (pushing the staged firmware out to bots)
    bot_dispatched = pyqtSignal(int)  # piece_id — OTA_START actually sent
    bot_result = pyqtSignal(int, bool, int)  # piece_id, success, err_code
    rollout_progress = pyqtSignal(int, int)  # completed, total
    rollout_finished = pyqtSignal(dict)  # piece_id -> success

    def __init__(self, serial: SerialHandler, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._serial = serial
        self._serial.ota_upload_status.connect(self._on_upload_status)
        self._serial.ota_result_received.connect(self._on_bot_result)
        self._serial.chunk_sent.connect(self._on_chunk_sent)

        self._pending_upload: Optional[bytes] = None
        self._pending_crc32: int = 0
        self._awaiting_ready = False
        self._upload_sent_bytes = 0
        self._upload_total_bytes = 0

        self._upload_progress_timer = QTimer(self)
        self._upload_progress_timer.setInterval(OTA.UPLOAD_PROGRESS_INTERVAL_MS)
        self._upload_progress_timer.timeout.connect(self._on_upload_progress_tick)

        # Diagnostic timeouts so a broken handshake surfaces as an explicit
        # failure instead of silently sitting at "waiting..." forever.
        self._ready_timeout_timer = QTimer(self)
        self._ready_timeout_timer.setSingleShot(True)
        self._ready_timeout_timer.timeout.connect(self._on_ready_timeout)

        self._complete_timeout_timer = QTimer(self)
        self._complete_timeout_timer.setSingleShot(True)
        self._complete_timeout_timer.timeout.connect(self._on_complete_timeout)

        self.max_parallel = OTA.MAX_PARALLEL_UPDATES
        self._version = 0
        self._queue: List[int] = []
        self._inflight: Dict[int, float] = {}
        self._results: Dict[int, bool] = {}
        self._total = 0

        self._timeout_timer = QTimer(self)
        self._timeout_timer.setInterval(1000)
        self._timeout_timer.timeout.connect(self._check_timeouts)

    # ------------------------------------------------------------------
    # Upload
    # ------------------------------------------------------------------

    def upload_firmware(self, path: str) -> None:
        """Read a .bin file and stage it on the Server over serial."""
        with open(path, "rb") as f:
            data = f.read()
        crc32 = zlib.crc32(data) & 0xFFFFFFFF
        self._pending_upload = data
        self._pending_crc32 = crc32
        self._awaiting_ready = True
        self.upload_progress.emit(
            f"Sent upload request ({len(data)} bytes, crc32=0x{crc32:08X}); "
            "waiting for server READY..."
        )
        self._serial.send(build_ota_upload_begin(len(data), crc32))
        self._ready_timeout_timer.start(int(OTA.UPLOAD_READY_TIMEOUT_S * 1000))

    def _on_ready_timeout(self) -> None:
        if not self._awaiting_ready:
            return
        self._awaiting_ready = False
        self._pending_upload = None
        message = (
            f"Timed out after {OTA.UPLOAD_READY_TIMEOUT_S:.0f}s waiting for the "
            "server to acknowledge the upload request (no READY received)."
        )
        self.upload_progress.emit(message)
        self.upload_finished.emit(False, message, 0)

    def _on_upload_status(self, status: str, detail: str) -> None:
        if status == "READY":
            self._ready_timeout_timer.stop()
            if not self._awaiting_ready or self._pending_upload is None:
                return
            self._awaiting_ready = False
            data = self._pending_upload
            self._upload_sent_bytes = 0
            self._upload_total_bytes = len(data)
            self.upload_progress.emit(
                f"Server READY; streaming {len(data)} bytes..."
            )
            self.upload_bytes_progress.emit(0, self._upload_total_bytes)
            self._upload_progress_timer.start()
            self._complete_timeout_timer.start(
                int(OTA.UPLOAD_COMPLETE_TIMEOUT_S * 1000)
            )
            self._serial.send_chunked(data, OTA.UPLOAD_CHUNK_SIZE)
        elif status == "OK":
            self._complete_timeout_timer.stop()
            self._upload_progress_timer.stop()
            self.upload_bytes_progress.emit(
                self._upload_total_bytes, self._upload_total_bytes
            )
            self._pending_upload = None
            self.upload_progress.emit("Server confirmed firmware staged (CRC verified).")
            self.upload_finished.emit(True, "", self._pending_crc32)
        elif status == "ERR":
            self._complete_timeout_timer.stop()
            self._upload_progress_timer.stop()
            self._pending_upload = None
            self._awaiting_ready = False
            self.upload_progress.emit(f"Server rejected upload: {detail}")
            self.upload_finished.emit(False, detail, 0)

    def _on_complete_timeout(self) -> None:
        self._upload_progress_timer.stop()
        self._pending_upload = None
        message = (
            f"Timed out after {OTA.UPLOAD_COMPLETE_TIMEOUT_S:.0f}s waiting for "
            "the server to confirm the upload (all bytes were sent, but no "
            "OK/ERR reply arrived)."
        )
        self.upload_progress.emit(message)
        self.upload_finished.emit(False, message, 0)

    def _on_chunk_sent(self, num_bytes: int) -> None:
        self._upload_sent_bytes += num_bytes
        if (
            self._upload_total_bytes
            and self._upload_sent_bytes >= self._upload_total_bytes
            and self._upload_progress_timer.isActive()
        ):
            # All bytes handed off; stop ticking at a static "100%" and make
            # clear we're now waiting on the server, not stuck sending.
            self._upload_progress_timer.stop()
            self.upload_bytes_progress.emit(
                self._upload_total_bytes, self._upload_total_bytes
            )
            self.upload_progress.emit(
                "All bytes sent; waiting for server to verify checksum and "
                "finish staging..."
            )

    def _on_upload_progress_tick(self) -> None:
        sent = min(self._upload_sent_bytes, self._upload_total_bytes)
        self.upload_bytes_progress.emit(sent, self._upload_total_bytes)

    # ------------------------------------------------------------------
    # Rollout
    # ------------------------------------------------------------------

    def start_rollout(
        self,
        piece_ids: List[int],
        version: int,
        max_parallel: Optional[int] = None,
    ) -> None:
        """Begin pushing the currently staged firmware to `piece_ids`.

        Dispatches up to `max_parallel` (default OTA.MAX_PARALLEL_UPDATES)
        OTA_START commands at once; as each bot's result arrives the next
        queued bot is dispatched, until the whole list has been processed.
        """
        self.max_parallel = max(1, max_parallel if max_parallel is not None else OTA.MAX_PARALLEL_UPDATES)
        self._version = version
        self._queue = list(piece_ids)
        self._inflight = {}
        self._results = {}
        self._total = len(piece_ids)
        self._timeout_timer.start()
        self._dispatch_more()

    def _dispatch_more(self) -> None:
        while self._queue and len(self._inflight) < self.max_parallel:
            piece_id = self._queue.pop(0)
            self._inflight[piece_id] = time.monotonic()
            self._serial.send(build_ota_start(piece_id, self._version))
            self.bot_dispatched.emit(piece_id)

    def cancel_rollout(self) -> List[int]:
        """Abort a rollout in progress.

        Bots already dispatched cannot be recalled over the air, but the
        Coordinator stops waiting for them. Returns the piece IDs that were
        still queued or in-flight (i.e. never resolved to success/failure) so
        the caller can reset their UI state.
        """
        outstanding = list(self._queue) + list(self._inflight.keys())
        self._queue = []
        self._inflight = {}
        self._timeout_timer.stop()
        return outstanding

    def _on_bot_result(self, piece_id: int, success: bool, err_code: int, version: int) -> None:
        if piece_id not in self._inflight:
            return  # not part of an active rollout (or already timed out)
        del self._inflight[piece_id]
        self._results[piece_id] = success
        self.bot_result.emit(piece_id, success, err_code)
        self.rollout_progress.emit(len(self._results), self._total)
        self._advance()

    def _check_timeouts(self) -> None:
        now = time.monotonic()
        for piece_id, started in list(self._inflight.items()):
            if now - started > OTA.RESULT_TIMEOUT_S:
                del self._inflight[piece_id]
                self._results[piece_id] = False
                self.bot_result.emit(piece_id, False, -1)
                self.rollout_progress.emit(len(self._results), self._total)
        self._advance()

    def _advance(self) -> None:
        if not self._queue and not self._inflight:
            self._timeout_timer.stop()
            self.rollout_finished.emit(dict(self._results))
        else:
            self._dispatch_more()
