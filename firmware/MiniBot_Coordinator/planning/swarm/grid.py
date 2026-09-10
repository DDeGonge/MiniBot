"""Coarse cell grid for choreography planning (one node per board square center).

PIBT plans which square each piece occupies at each step; this grid gives it the
squares and their 8-connected adjacency. Motion between squares is straight and
continuous (executed by the BVC layer), so adjacency is center-to-center: a piece
travels through tile centers, never along the lines between them. Squares sit far
enough apart that two distinct cells never conflict, so occupancy is simply "one
piece per cell".
"""
import math

_NEIGHBORS_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1),
                (0, 1), (1, -1), (1, 0), (1, 1)]


class CellGrid:
    """Square-center cells with 8-connected adjacency and cell/world conversions."""

    def __init__(self, cell_specs, pitch_mm: float):
        """Build the grid from (col, row, x_mm, y_mm) specs at the given pitch.

        cell_specs lists every square as its integer (col, row) plus its world
        center; pitch_mm is the square side length, used to snap world points back
        to cells. Neighbors are the 8 grid-adjacent squares that are present.
        """
        self._pitch = pitch_mm
        self._cells = [(c, r) for (c, r, _x, _y) in cell_specs]
        self._xy = [(x, y) for (_c, _r, x, y) in cell_specs]
        self._index = {cr: i for i, cr in enumerate(self._cells)}
        present = set(self._cells)
        self._neighbors = [
            [self._index[(c + dc, r + dr)] for dc, dr in _NEIGHBORS_8
             if (c + dc, r + dr) in present]
            for (c, r) in self._cells
        ]

    def _cell_of_world(self, x_mm: float, y_mm: float) -> tuple:
        """Integer (col, row) of the square center covering a world point."""
        return (round(x_mm / self._pitch - 0.5), round(y_mm / self._pitch - 0.5))

    @property
    def count(self) -> int:
        """Number of cells in the grid."""
        return len(self._cells)

    def neighbors(self, cell_id: int) -> list:
        """8-connected neighbor cell ids of a cell."""
        return self._neighbors[cell_id]

    def xy(self, cell_id: int) -> tuple:
        """World (x, y) center of a cell."""
        return self._xy[cell_id]

    def nearest(self, x_mm: float, y_mm: float) -> int:
        """Cell id whose center is nearest the given world point."""
        cell = self._cell_of_world(x_mm, y_mm)
        if cell in self._index:
            return self._index[cell]
        best_id, best_d = 0, math.inf
        for i, (cx, cy) in enumerate(self._xy):
            d = (cx - x_mm) ** 2 + (cy - y_mm) ** 2
            if d < best_d:
                best_d, best_id = d, i
        return best_id
