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
    mgr.serial_connection_changed.connect(...)   # fail OTA work on disconnect
    mgr.upload_firmware(data_bytes)   # caller reads the file so its size
                                      # label and the staged payload come
                                      # from the same read
    # once upload_finished(True, "", crc32) fires:
    mgr.start_rollout(piece_ids=[0x01, 0x02, ...], version=crc32, max_parallel=1)
"""

from __future__ import annotations

import time
import zlib
from typing import Dict, List, Optional, Set

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
    # Emitted only when the Server answers a >201 with its accept ack (>203,{id}),
    # i.e. once the command is known to be queued on the Server — not merely
    # written to the UART. A lost/corrupted serial line therefore no longer
    # paints an orange dot that nothing backs up; the slot stays pending until
    # either >203, a per-bot NACK (>4) or the per-bot timeout resolves it.
    bot_dispatched = pyqtSignal(int)  # piece_id — OTA_START accepted by Server
    bot_result = pyqtSignal(int, bool, int)  # piece_id, success, err_code
    rollout_progress = pyqtSignal(int, int)  # completed, total
    rollout_finished = pyqtSignal(dict)  # piece_id -> success

    # Serial link state (re-emitted from SerialHandler). SerialHandler.send()
    # and send_chunked() are silent no-ops unless the worker is up AND
    # connected, so an OTA attempt on a dead port used to look healthy for
    # OTA.UPLOAD_READY_TIMEOUT_S before reporting a misleading "no READY
    # received" timeout, and a disconnect mid-flight left pending/orange dots
    # with no way to fail them. Everything in flight is failed explicitly.
    serial_connection_changed = pyqtSignal(bool)  # True=connected

    def __init__(self, serial: SerialHandler, parent: Optional[QObject] = None) -> None:
        super().__init__(parent)
        self._serial = serial
        self._serial.ota_upload_status.connect(self._on_upload_status)
        self._serial.ota_result_received.connect(self._on_bot_result)
        self._serial.chunk_sent.connect(self._on_chunk_sent)
        # Server rejections of >201 now arrive as per-bot NACKs
        # (>4,{id},{err_type},{timestamp_ms}); resolve the affected rollout
        # slot immediately instead of waiting out RESULT_TIMEOUT_S.
        self._serial.error_received.connect(self._on_dispatch_error)
        # Serial link state (bug 4): the tab and the rollout must not rely on
        # send()/send_chunked() silently doing nothing.
        self._serial.connection_changed.connect(self._on_connection_changed)
        # Server accept ack for >201 (>203,{id}): only a command the Server
        # actually queued may be shown as an active (orange) tile.
        self._serial.ota_command_accepted.connect(self._on_bot_accepted)

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
        # Piece IDs whose >201 the Server confirmed with an accept ack (>203).
        # Guards against a duplicate ack re-emitting bot_dispatched.
        self._accepted: Set[int] = set()

        self._timeout_timer = QTimer(self)
        self._timeout_timer.setInterval(1000)
        self._timeout_timer.timeout.connect(self._check_timeouts)

    # ------------------------------------------------------------------
    # Upload
    # ------------------------------------------------------------------

    def upload_firmware(self, data: bytes) -> None:
        """Stage a firmware image on the Server over serial.

        The caller (typically ``OtaUpdateTab._on_begin``) owns the file
        read. Previously this function did ``open(path, "rb").read()``
        itself, which caused two problems when invoked from a Qt signal
        callback: (a) a deleted / moved / locked file raised inside the
        ``clicked`` handler and wedged the tab permanently (no
        ``upload_finished`` fired, every dot stuck pending with Cancel as
        the only escape), and (b) the file's size could change between the
        label's ``os.path.getsize`` at Browse-time and the payload read
        here at Begin-time, so the two could silently disagree. Reading
        the bytes at the tab and handing them in removes both failure
        modes; any file error is surfaced at the tab before this
        function runs.

        A dead serial link is rejected up-front: ``SerialHandler.send()``
        would otherwise be a silent no-op and the only clue would be the
        misleading ``OTA.UPLOAD_READY_TIMEOUT_S`` "no READY received" message
        ten seconds later.
        """
        if not self._serial.is_connected:
            message = (
                "Serial port is not connected to the Server, so the image "
                "cannot be staged."
            )
            self.upload_progress.emit(message)
            self.upload_finished.emit(False, message, 0)
            return

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

    def abort_upload(self, reason: str = "cancelled by user") -> bool:
        """Cancel an in-flight upload.

        Clears pending upload state and stops every upload timer so no late
        READY / OK / ERR / timeout callback can drive ``upload_finished``
        after the user has already cancelled. Without this, a cancel issued
        during the streaming phase is silently undone when the upload
        finishes: ``_on_upload_finished(True, ...)`` still fires and
        ``start_rollout(self._pending_targets)`` pushes the firmware to the
        cancelled target list.

        No-op when no upload is in flight (already finished, aborted, or
        never started).

        Returns True if an upload was aborted, False otherwise.
        """
        if not self._awaiting_ready and self._pending_upload is None:
            return False
        self._awaiting_ready = False
        self._pending_upload = None
        self._ready_timeout_timer.stop()
        self._complete_timeout_timer.stop()
        self._upload_progress_timer.stop()
        self.upload_progress.emit(f"Upload aborted: {reason}.")
        self.upload_finished.emit(False, reason, 0)
        return True

    # ------------------------------------------------------------------
    # Serial link state
    # ------------------------------------------------------------------

    def serial_connected(self) -> bool:
        """True when the serial worker is running and attached to the Server.

        ``SerialHandler.send()``/``send_chunked()`` are silent no-ops when
        this is False, so every OTA entry point must check it rather than
        assume the bytes went out.
        """
        return bool(self._serial.is_connected)

    def _on_connection_changed(self, connected: bool) -> None:
        self.serial_connection_changed.emit(connected)
        if connected:
            return
        self._fail_outstanding(OTA.ERR_CODE_NO_CONNECTION)
        self.abort_upload("serial port disconnected")

    def _fail_outstanding(self, err_code: int) -> int:
        """Resolve every queued/in-flight rollout slot as failed.

        Used when the serial link goes down: nothing more can be sent, and
        nothing already sent can be acknowledged, so waiting for results is
        pointless. ``cancel_rollout`` deliberately leaves those bots
        unresolved (the user took an explicit action); here the rollout ends
        on its own, so every slot gets an outcome and the UI never sits on
        orange dots with Cancel as the only escape.

        Returns the number of bots resolved.
        """
        outstanding = list(self._queue) + list(self._inflight)
        if not outstanding:
            return 0
        self._queue = []
        self._inflight = {}
        self._timeout_timer.stop()
        for piece_id in outstanding:
            self._results[piece_id] = False
            self.bot_result.emit(piece_id, False, err_code)
            self.rollout_progress.emit(len(self._results), self._total)
        self.rollout_finished.emit(dict(self._results))
        return len(outstanding)

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

        ``bot_dispatched`` lags the actual write by one round trip: it is
        emitted when the Server answers a ``>201`` with its accept ack
        (``>203,{id}``), so a UI tile only turns orange once the command is
        known to be queued on the Server.
        """
        self.max_parallel = max(1, max_parallel if max_parallel is not None else OTA.MAX_PARALLEL_UPDATES)
        self._version = version
        self._queue = list(piece_ids)
        self._inflight = {}
        self._accepted = set()
        self._results = {}
        self._total = len(piece_ids)
        self._timeout_timer.start()
        self._dispatch_more()

    def _dispatch_more(self) -> None:
        if not self._serial.is_connected:
            # send() is a silent no-op on a dead link: nothing can be
            # dispatched and nothing already sent can be acknowledged, so
            # fail the whole remainder instead of letting the per-bot
            # OTA.RESULT_TIMEOUT_S timers burn out one bot at a time.
            self._fail_outstanding(OTA.ERR_CODE_NO_CONNECTION)
            return
        while self._queue and len(self._inflight) < self.max_parallel:
            piece_id = self._queue.pop(0)
            self._inflight[piece_id] = time.monotonic()
            self._serial.send(build_ota_start(piece_id, self._version))
            # bot_dispatched is deliberately NOT emitted here: at this point
            # the >201 bytes have only been queued onto the serial worker, the
            # Server has not parsed the line, and ESPNowTask has not broadcast
            # the command yet. The Server's accept ack (>203,{id}) raises the
            # signal in _on_bot_accepted instead, so an orange tile always
            # means "the Server queued this bot's OTA command".

    def _on_bot_accepted(self, piece_id: int) -> None:
        """Handle the Server's accept ack (>203,{id}) for one bot's >201.

        Only a command the Server actually queued may be shown as active:
        the ack is ignored for IDs that are not part of the live rollout, and
        a repeated ack for the same ID is ignored so the tile is not
        re-published every time.
        """
        if piece_id not in self._inflight or piece_id in self._accepted:
            return
        self._accepted.add(piece_id)
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
                self.bot_result.emit(piece_id, False, OTA.ERR_CODE_TIMEOUT)
                self.rollout_progress.emit(len(self._results), self._total)
        self._advance()

    def _advance(self) -> None:
        if not self._queue and not self._inflight:
            self._timeout_timer.stop()
            self.rollout_finished.emit(dict(self._results))
        else:
            self._dispatch_more()

    def _on_dispatch_error(self, piece_id: int, reason: str) -> None:
        """Resolve an in-flight bot when the Server rejects its >201.

        The Server's ``case 201`` used to print bare ``>ERR,OTA: ...`` lines
        for preconditions (no staged firmware) and queue overflow. Those
        landed in ``RESP_PARSE_ERROR`` on the Coordinator and were invisible
        to OtaManager, so every rejection silently burned
        ``OTA.RESULT_TIMEOUT_S`` (90 s) before surfacing as a timeout. The
        Server now emits per-bot NACKs ``>4,{id},{err_type},{timestamp_ms}``
        which SerialHandler forwards via ``error_received``. Fail the
        affected rollout slot immediately so the tile flips to red in
        milliseconds and the next queued bot is dispatched without waiting.

        Ignores NACKs for piece IDs that are not currently dispatched —
        unrelated position/motion NACKs, or late replies after a cancel —
        so this handler is safe to leave wired in even when no rollout is
        running.

        ``reason`` is the numeric NACK err_type stringified by
        ``protocol.parse_line`` (see the ``>4,{id},{err_type},{timestamp_ms}``
        branch). Known Server-side codes for OTA rejections are
        ``ERR_ROBOT_UNAVAILABLE=2`` ("no firmware staged") and
        ``ERR_QUEUE_FULL=1`` ("command queue full"); other values are
        surfaced as-is so the UI can display them via
        ``OtaUpdateTab._on_bot_result``'s ``failed (code N)`` label.
        """
        if piece_id not in self._inflight:
            return
        try:
            err_code = int(reason)
        except (TypeError, ValueError):
            err_code = 0
        del self._inflight[piece_id]
        self._results[piece_id] = False
        self.bot_result.emit(piece_id, False, err_code)
        self.rollout_progress.emit(len(self._results), self._total)
        self._advance()
