"""
gui/chessboard_widget.py  —  MiniBot Chess Swarm Coordinator

Custom QWidget that renders the physical chessboard environment:
  - 125mm left/right borders, 25mm top/bottom borders (all drawn to scale)
  - Hard outer outline rectangle
  - 8×8 checkerboard (50mm squares)
  - 34 robot pieces as labeled circles with orientation markers
  - Selected-piece highlight and target-position indicator

Coordinate space:
  The widget uses a logical mm coordinate system matching the physical robot
  table.  Origin (0,0) is the bottom-left corner of the PLAYING AREA.
  Screen Y is inverted so +Y points upward (standard math convention).

  painter.setWindow / setViewport provides the mm→px mapping.

Signals:
  piece_selected(int)               — emitted when user clicks near a piece
  target_set(int, float, float)     — left-click target: plan + add to move queue
  target_queued(int, float, float)  — right-click target: immediate dispatch
"""

from __future__ import annotations

import math
from typing import Dict, FrozenSet, List, Optional, Set, Tuple

from PyQt6.QtCore import Qt, QPoint, QPointF, QRect, QRectF, pyqtSignal
from PyQt6.QtGui import (
    QBrush,
    QColor,
    QFont,
    QMouseEvent,
    QPainter,
    QPaintEvent,
    QPen,
)
from PyQt6.QtWidgets import QWidget

from config import BOARD, ELECTROMAGNETS, GUI, PIECES
from models.piece import BoardState, Piece
from planning.base_planner import MoveCommand


class ChessBoardWidget(QWidget):
    """Scaled chessboard canvas widget.

    The logical coordinate space is (0,0) bottom-left of the playing area,
    650 mm wide × 450 mm tall.  The canvas scales to fill the widget size
    while preserving the aspect ratio.
    """

    piece_selected = pyqtSignal(int)  # piece_id
    target_set = pyqtSignal(
        int, float, float
    )  # piece_id, x_mm, y_mm  (left-click → plan + queue)
    target_queued = pyqtSignal(
        int, float, float
    )  # piece_id, x_mm, y_mm  (right-click → immediate dispatch)

    # ------------------------------------------------------------------
    # Logical canvas constants (mm)
    # ------------------------------------------------------------------
    _LW = BOARD.CANVAS_WIDTH_MM  # 650
    _LH = BOARD.CANVAS_HEIGHT_MM  # 450
    _OX = BOARD.BORDER_LEFT_MM  # 125  offset of playing area origin
    _OY = BOARD.BORDER_BOTTOM_MM  # 25

    def __init__(
        self, board_state: BoardState, parent: Optional[QWidget] = None
    ) -> None:
        super().__init__(parent)
        self._board = board_state
        self._selected_id: Optional[int] = None
        self._target: Optional[Tuple[float, float]] = None  # mm in playing area
        self._hide_stale: bool = False
        self._stale_ids: Set[int] = set()
        self._show_electromagnets: bool = False
        # Plan visualization: list of (x0, y0, x1, y1, wave_idx, total_waves)
        self._plan_arrows: List[Tuple[float, float, float, float, int, int]] = []

        self.setMinimumSize(
            int(BOARD.CANVAS_WIDTH_MM * GUI.SCALE_FACTOR),
            int(BOARD.CANVAS_HEIGHT_MM * GUI.SCALE_FACTOR),
        )
        self.setMouseTracking(False)
        self.setAttribute(Qt.WidgetAttribute.WA_OpaquePaintEvent)
        # Accept keyboard focus so Escape key can deselect
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)

    # ------------------------------------------------------------------
    # Public slots
    # ------------------------------------------------------------------

    def refresh(self) -> None:
        """Trigger a repaint (call after updating BoardState externally)."""
        self.update()

    def set_stale_pieces(self, stale_ids: Set[int], hide: bool) -> None:
        """Update the set of piece IDs considered stale and whether to hide them."""
        self._stale_ids = stale_ids
        self._hide_stale = hide
        self.update()

    def set_show_electromagnets(self, show: bool) -> None:
        """Toggle display of electromagnet location rings."""
        self._show_electromagnets = show
        self.update()

    def set_plan_visualization(
        self,
        commands: Optional[List[MoveCommand]],
        initial_positions: Dict[int, Tuple[float, float]],
    ) -> None:
        """Compute and store colored arrow draw records for planned moves.

        Call with commands=None or empty list to clear the visualization.
        Arrow color encodes wave index: green (wave 0) → orange → red (last wave).
        """
        self._plan_arrows = []
        if not commands:
            self.update()
            return

        # Group by sequence_num
        waves: Dict[int, List[MoveCommand]] = {}
        for cmd in commands:
            waves.setdefault(cmd.sequence_num, []).append(cmd)

        if not waves:
            self.update()
            return

        total_waves = max(waves.keys()) + 1
        # Simulate piece positions wave by wave so arrows start from correct positions
        cur_pos: Dict[int, Tuple[float, float]] = dict(initial_positions)

        for wave_idx in sorted(waves.keys()):
            wave_cmds = waves[wave_idx]
            for cmd in wave_cmds:
                pid = cmd.piece_id
                if pid not in cur_pos:
                    continue
                x0, y0 = cur_pos[pid]
                x1, y1 = cmd.target_x_mm, cmd.target_y_mm
                self._plan_arrows.append((x0, y0, x1, y1, wave_idx, total_waves))
            # Advance positions
            for cmd in wave_cmds:
                cur_pos[cmd.piece_id] = (cmd.target_x_mm, cmd.target_y_mm)

        self.update()

    def clear_selection(self) -> None:
        self._selected_id = None
        self._target = None
        self.update()

    def set_target(self, piece_id: int, x_mm: float, y_mm: float) -> None:
        """Programmatically set a target indicator for a piece."""
        self._selected_id = piece_id
        self._target = (x_mm, y_mm)
        self.update()

    # ------------------------------------------------------------------
    # Mouse interaction
    # ------------------------------------------------------------------

    def mousePressEvent(self, event: QMouseEvent) -> None:
        self.setFocus()
        board_x, board_y = self._widget_to_mm(event.position())
        clicked_pid = self._piece_at(board_x, board_y)
        btn = event.button()

        # ── Right-click ──────────────────────────────────────────────────
        if btn == Qt.MouseButton.RightButton:
            if self._selected_id is not None and clicked_pid != self._selected_id:
                # Piece selected + right-clicking a target → queue the move
                self._target = (board_x, board_y)
                self.update()
                self.target_queued.emit(self._selected_id, board_x, board_y)
            # Right-clicking with no selection (or on the selected piece) is ignored
            return

        if btn != Qt.MouseButton.LeftButton:
            return

        # ── Left-click ───────────────────────────────────────────────────
        if self._selected_id is None:
            # No selection: try to select a piece
            if clicked_pid is not None:
                self._selected_id = clicked_pid
                self._target = None
                self.update()
                self.piece_selected.emit(clicked_pid)
        else:
            if clicked_pid is not None and clicked_pid != self._selected_id:
                # Clicked a different piece: re-select it
                self._selected_id = clicked_pid
                self._target = None
                self.update()
                self.piece_selected.emit(clicked_pid)
            elif clicked_pid == self._selected_id:
                # Clicked the already-selected piece: deselect
                self.clear_selection()
            else:
                # Clicked empty area: immediate dispatch
                self._target = (board_x, board_y)
                self.update()
                self.target_set.emit(self._selected_id, board_x, board_y)

    def keyPressEvent(self, event) -> None:
        if event.key() == Qt.Key.Key_Escape:
            self.clear_selection()
        else:
            super().keyPressEvent(event)

    # ------------------------------------------------------------------
    # Painting
    # ------------------------------------------------------------------

    def paintEvent(self, event: QPaintEvent) -> None:
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w = self.width()
        h = self.height()

        # --- Set up mm→pixel mapping ---
        # Logical window: (0, 0) top-left = (-OX, +OY+PA) in playing-area mm
        # We define the logical window so that (0,0) maps to the bottom-left
        # of the playing area, Y increases upward.
        # QPainter.setWindow uses top-left origin with Y downward, so we flip.
        # Strategy: manually scale & translate inside paintEvent.

        # Scale to preserve aspect ratio, centred in widget
        scale_x = w / self._LW
        scale_y = h / self._LH
        scale = min(scale_x, scale_y)

        canvas_w_px = int(self._LW * scale)
        canvas_h_px = int(self._LH * scale)
        offset_x = (w - canvas_w_px) // 2
        offset_y = (h - canvas_h_px) // 2

        painter.translate(offset_x, offset_y)
        painter.scale(scale, scale)

        # Now 1 logical unit = 1 mm.  Y flipped below per item.
        # Helper: mm (playing-area coords) → canvas pixels (Y-down)
        # Note: canvas Y-down coords have origin at top-left of canvas.
        # playing-area (0,0) is at canvas (_OX, _LH - _OY) in Y-down mm.

        self._scale = scale
        self._offset_x = offset_x
        self._offset_y = offset_y

        self._draw_background(painter)
        self._draw_outer_outline(painter)
        self._draw_grid(painter)
        self._draw_plan_paths(painter)
        self._draw_electromagnets(painter)
        self._draw_target_indicator(painter)
        self._draw_pieces(painter)

        painter.end()

    # ------------------------------------------------------------------
    # Drawing sub-routines  (all coords in canvas-mm, Y-down)
    # ------------------------------------------------------------------

    def _pa_to_canvas(self, x_mm: float, y_mm: float) -> Tuple[float, float]:
        """Convert playing-area mm (Y-up) to canvas mm (Y-down)."""
        cx = x_mm + self._OX
        cy = self._LH - (y_mm + self._OY)
        return cx, cy

    def _draw_background(self, p: QPainter) -> None:
        p.fillRect(QRectF(0, 0, self._LW, self._LH), QColor(GUI.BOARD_BACKGROUND_COLOR))

    def _draw_outer_outline(self, p: QPainter) -> None:
        pen = QPen(QColor(GUI.BOARD_OUTLINE_COLOR))
        pen.setWidthF(BOARD.OUTLINE_THICKNESS_MM)
        pen.setJoinStyle(Qt.PenJoinStyle.MiterJoin)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        # Draw exactly at canvas boundary (inset by half line width)
        hw = BOARD.OUTLINE_THICKNESS_MM / 2
        p.drawRect(QRectF(hw, hw, self._LW - 2 * hw, self._LH - 2 * hw))

    def _draw_grid(self, p: QPainter) -> None:
        s = BOARD.SQUARE_SIZE_MM
        n = BOARD.NUM_SQUARES

        for row in range(n):
            for col in range(n):
                # Square center in playing-area coords (Y-up)
                pa_x = col * s
                pa_y = row * s
                cx, cy = self._pa_to_canvas(pa_x, pa_y)

                # Checkerboard: light if (row+col) even
                light = (row + col) % 2 == 0
                color = QColor(
                    GUI.LIGHT_SQUARE_COLOR if light else GUI.DARK_SQUARE_COLOR
                )
                p.fillRect(QRectF(cx, cy - s, s, s), color)

        # Grid lines
        pen = QPen(QColor(0, 0, 0, 60))
        pen.setWidthF(BOARD.GRID_LINE_THICKNESS_MM)
        p.setPen(pen)
        pa_x0, pa_y0 = 0.0, 0.0
        pa_x1, pa_y1 = float(n * s), float(n * s)
        cx0, cy_top = self._pa_to_canvas(pa_x0, pa_y1)
        cx1, cy_bot = self._pa_to_canvas(pa_x1, pa_y0)

        # Vertical lines
        for col in range(n + 1):
            cx, _ = self._pa_to_canvas(float(col * s), 0)
            p.drawLine(QPointF(cx, cy_top), QPointF(cx, cy_bot))
        # Horizontal lines
        for row in range(n + 1):
            _, cy = self._pa_to_canvas(0, float(row * s))
            p.drawLine(QPointF(cx0, cy), QPointF(cx1, cy))

    def _draw_plan_paths(self, p: QPainter) -> None:
        """Draw colored wave arrows for planned moves."""
        if not self._plan_arrows:
            return

        def _wave_color(wave_idx: int, total: int) -> QColor:
            """Green → orange → red gradient by wave index."""
            if total <= 1:
                t = 0.0
            else:
                t = wave_idx / (total - 1)
            # Green (#00C060) → orange (#FFA020) → red (#FF3030)
            if t <= 0.5:
                s = t * 2.0
                r = int(0x00 + (0xFF - 0x00) * s)
                g = int(0xC0 + (0xA0 - 0xC0) * s)
                b = int(0x60 + (0x20 - 0x60) * s)
            else:
                s = (t - 0.5) * 2.0
                r = 0xFF
                g = int(0xA0 + (0x30 - 0xA0) * s)
                b = int(0x20 + (0x30 - 0x20) * s)
            return QColor(r, g, b, 200)

        ARROW_HEAD_MM = 8.0
        ARROW_HALF_ANG = math.radians(25)
        LINE_W = 2.0

        for x0, y0, x1, y1, wave_idx, total_waves in self._plan_arrows:
            cx0, cy0 = self._pa_to_canvas(x0, y0)
            cx1, cy1 = self._pa_to_canvas(x1, y1)

            color = _wave_color(wave_idx, total_waves)
            pen = QPen(color)
            pen.setWidthF(LINE_W)
            p.setPen(pen)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawLine(QPointF(cx0, cy0), QPointF(cx1, cy1))

            # Arrowhead
            dx = cx1 - cx0
            dy = cy1 - cy0
            mag = math.hypot(dx, dy)
            if mag < 1e-6:
                continue
            ux, uy = dx / mag, dy / mag
            # Arrowhead base point back along the line
            bx = cx1 - ux * ARROW_HEAD_MM
            by = cy1 - uy * ARROW_HEAD_MM
            cos_a = math.cos(ARROW_HALF_ANG)
            sin_a = math.sin(ARROW_HALF_ANG)
            # Two wing points
            w1x = bx + (-uy * sin_a + ux * (cos_a - 1)) * ARROW_HEAD_MM
            w1y = by + (ux * sin_a + uy * (cos_a - 1)) * ARROW_HEAD_MM
            w2x = bx + (uy * sin_a + ux * (cos_a - 1)) * ARROW_HEAD_MM
            w2y = by + (-ux * sin_a + uy * (cos_a - 1)) * ARROW_HEAD_MM

            from PyQt6.QtGui import QPolygonF

            tip = QPointF(cx1, cy1)
            wing1 = QPointF(w1x, w1y)
            wing2 = QPointF(w2x, w2y)
            poly = QPolygonF([tip, wing1, wing2])

            p.setPen(Qt.PenStyle.NoPen)
            p.setBrush(QBrush(color))
            p.drawPolygon(poly)

    def _draw_electromagnets(self, p: QPainter) -> None:
        """Draw semi-transparent light-grey rings at each electromagnet location."""
        if not self._show_electromagnets:
            return

        outer_r = ELECTROMAGNETS.OD_MM / 2.0
        inner_r = ELECTROMAGNETS.ID_MM / 2.0
        ring_w = outer_r - inner_r

        ring_color = QColor(200, 200, 200, 90)  # light grey, semi-transparent
        pen = QPen(ring_color)
        pen.setWidthF(ring_w)
        pen.setCapStyle(Qt.PenCapStyle.FlatCap)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)

        mid_r = (outer_r + inner_r) / 2.0
        for x_mm, y_mm in ELECTROMAGNETS.POSITIONS:
            cx, cy = self._pa_to_canvas(x_mm, y_mm)
            p.drawEllipse(QPointF(cx, cy), mid_r, mid_r)

    def _draw_target_indicator(self, p: QPainter) -> None:
        if self._target is None or self._selected_id is None:
            return
        tx, ty = self._pa_to_canvas(self._target[0], self._target[1])
        r = PIECES.CIRCLE_RADIUS_MM
        pen = QPen(QColor(GUI.TARGET_INDICATOR_COLOR))
        pen.setWidthF(2.0)
        pen.setStyle(Qt.PenStyle.DashLine)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QPointF(tx, ty), r, r)
        # Cross-hair
        p.drawLine(QPointF(tx - r * 0.5, ty), QPointF(tx + r * 0.5, ty))
        p.drawLine(QPointF(tx, ty - r * 0.5), QPointF(tx, ty + r * 0.5))

    def _draw_pieces(self, p: QPainter) -> None:
        for piece in self._board.all_pieces():
            if piece.is_captured:
                continue
            if self._hide_stale and piece.piece_id in self._stale_ids:
                continue
            self._draw_piece(p, piece, stale=piece.piece_id in self._stale_ids)

    def _draw_piece(self, p: QPainter, piece: Piece, stale: bool = False) -> None:
        cx, cy = self._pa_to_canvas(piece.x_mm, piece.y_mm)
        r = float(PIECES.CIRCLE_RADIUS_MM)
        is_sel = piece.piece_id == self._selected_id

        white = piece.color == "white"
        fill = QColor(GUI.WHITE_PIECE_FILL if white else GUI.BLACK_PIECE_FILL)
        outline = QColor(GUI.WHITE_PIECE_OUTLINE if white else GUI.BLACK_PIECE_OUTLINE)

        # Dim stale pieces
        if stale:
            fill.setAlpha(60)
            outline.setAlpha(60)

        # --- Selection highlight ---
        if is_sel:
            halo_pen = QPen(QColor(GUI.SELECTED_HIGHLIGHT_COLOR))
            halo_pen.setWidthF(3.0)
            p.setPen(halo_pen)
            p.setBrush(Qt.BrushStyle.NoBrush)
            p.drawEllipse(QPointF(cx, cy), r + 3.5, r + 3.5)

        # --- Circle body ---
        body_pen = QPen(outline)
        body_pen.setWidthF(1.5)
        p.setPen(body_pen)
        p.setBrush(QBrush(fill))
        p.drawEllipse(QPointF(cx, cy), r, r)

        # --- Orientation line (from center outward in facing direction) ---
        ol = float(PIECES.ORIENTATION_LINE_MM)
        # theta=0 means facing +X; canvas uses Y-down so negate the sin term
        theta_canvas = math.radians(piece.orientation_deg)
        ex = cx + ol * math.cos(theta_canvas)
        ey = cy - ol * math.sin(theta_canvas)

        ori_color = QColor(
            GUI.ORIENTATION_LINE_COLOR_WHITE
            if white
            else GUI.ORIENTATION_LINE_COLOR_BLACK
        )
        ori_pen = QPen(ori_color)
        ori_pen.setWidthF(PIECES.ORIENTATION_LINE_WIDTH_MM)
        ori_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        p.setPen(ori_pen)
        p.drawLine(QPointF(cx, cy), QPointF(ex, ey))

        # --- Rank label ---
        p.setPen(QPen(outline))
        rank_font = QFont("Arial", PIECES.LABEL_FONT_SIZE_PT, QFont.Weight.Bold)
        rank_font.setPixelSize(int(r * 0.75))
        p.setFont(rank_font)
        p.drawText(
            QRectF(cx - r, cy - r * 0.6, 2 * r, r),
            Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter,
            piece.rank_char,
        )

        # --- ID label (small, below rank) ---
        id_font = QFont("Courier", PIECES.ID_FONT_SIZE_PT)
        id_font.setPixelSize(int(r * 0.45))
        p.setFont(id_font)
        p.drawText(
            QRectF(cx - r, cy + r * 0.05, 2 * r, r * 0.55),
            Qt.AlignmentFlag.AlignHCenter | Qt.AlignmentFlag.AlignVCenter,
            piece.id_hex,
        )

    # ------------------------------------------------------------------
    # Coordinate utilities
    # ------------------------------------------------------------------

    def _widget_to_mm(self, pos: QPointF) -> Tuple[float, float]:
        """Convert widget pixel position to playing-area mm (Y-up)."""
        px = pos.x() - self._offset_x
        py = pos.y() - self._offset_y
        # To canvas mm
        if self._scale == 0:
            return 0.0, 0.0
        cmm_x = px / self._scale
        cmm_y = py / self._scale
        # Canvas mm (Y-down) → playing area mm (Y-up)
        pa_x = cmm_x - self._OX
        pa_y = (self._LH - cmm_y) - self._OY
        return pa_x, pa_y

    def _piece_at(self, pa_x: float, pa_y: float) -> Optional[int]:
        """Return the piece_id of the piece closest to (pa_x, pa_y), if within radius."""
        r = float(PIECES.CIRCLE_RADIUS_MM)
        closest_id = None
        closest_dist = r + 1.0

        for piece in self._board.all_pieces():
            if piece.is_captured:
                continue
            dist = math.hypot(pa_x - piece.x_mm, pa_y - piece.y_mm)
            if dist <= r and dist < closest_dist:
                closest_dist = dist
                closest_id = piece.piece_id

        return closest_id

    # ------------------------------------------------------------------
    # Initialise scale fields so _widget_to_mm works before first paint
    # ------------------------------------------------------------------

    def showEvent(self, event) -> None:
        super().showEvent(event)
        self._scale = GUI.SCALE_FACTOR
        self._offset_x = 0
        self._offset_y = 0

    # Provide defaults so attribute access before first paint() is safe
    _scale = GUI.SCALE_FACTOR
    _offset_x = 0
    _offset_y = 0
