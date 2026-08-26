"""Headless validation of the Play Chess tab logic.

Run from the coordinator root:  python gui/tabs/_selftest_chess.py
Exercises normal move, capture, castling, en passant, and promotion without
opening the GUI, checking the computed robot targets, graveyard/spare handling,
and square->id consistency with python-chess.
"""
import math
import os
import sys

sys.path.insert(0, os.getcwd())

import chess  # noqa: E402
from PyQt6.QtWidgets import QApplication  # noqa: E402

from config import BOARD, PIECES  # noqa: E402
from models.piece import BoardState  # noqa: E402
from gui.tabs.play_chess_tab import PlayChessTab  # noqa: E402

S = float(BOARD.SQUARE_SIZE_MM)


def center(sq):
    return ((chess.square_file(sq) + 0.5) * S, (chess.square_rank(sq) + 0.5) * S)


def close(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1]) < 0.01


def fresh():
    board = BoardState()
    tab = PlayChessTab(board, None)
    tab.new_game()
    return board, tab


def play(tab, uci):
    """Apply a uci move via the tab, returning (targets, mover_id)."""
    move = chess.Move.from_uci(uci)
    assert move in tab._chess.legal_moves, f"{uci} not legal"
    mover_id = tab._sq_to_id[move.from_square]
    targets, _san, _w, _n = tab._apply_move(move)
    return targets, mover_id


def assert_consistent(tab):
    """Every occupied chess square has exactly one physical id, and vice versa."""
    assert set(tab._sq_to_id) == set(tab._chess.piece_map()), \
        "square->id map diverged from python-chess"


def test_normal_and_capture():
    _board, tab = fresh()
    t, mv = play(tab, "e2e4")          # normal
    assert set(t) == {mv} and close(t[mv], center(chess.E4))
    assert_consistent(tab)
    play(tab, "d7d5")
    cap_id = tab._sq_to_id[chess.D5]   # black d-pawn about to be captured
    t, mv = play(tab, "e4d5")          # capture
    assert close(t[mv], center(chess.D5))
    # Interchangeable graveyard: the captured piece goes to SOME valid black
    # graveyard slot (assigned farthest-first), not one fixed per-id square.
    black_slots = [(float(x), float(y)) for pid, (x, y, _t)
                   in PIECES.GRAVEYARD_POSITIONS.items() if pid >= 0x12]
    assert any(close(t[cap_id], s) for s in black_slots), \
        "captured piece not sent to a graveyard slot"
    assert_consistent(tab)
    print("  normal + capture: OK")


def test_castling():
    _board, tab = fresh()
    for uci in ("e2e4", "e7e5", "g1f3", "b8c6", "f1c4", "g8f6"):
        play(tab, uci)
    rook_id = tab._sq_to_id[chess.H1]
    t, king_id = play(tab, "e1g1")     # king-side castle
    assert close(t[king_id], center(chess.G1)), "king target wrong"
    assert close(t[rook_id], center(chess.F1)), "rook not moved to f1"
    assert_consistent(tab)
    print("  castling (king + rook): OK")


def test_en_passant():
    _board, tab = fresh()
    for uci in ("e2e4", "e7e6", "e4e5", "d7d5"):
        play(tab, uci)
    ep_pawn = tab._sq_to_id[chess.D5]  # black pawn that will be captured e.p.
    move = chess.Move.from_uci("e5d6")
    assert tab._chess.is_en_passant(move)
    mover = tab._sq_to_id[chess.E5]
    t, _san, _w, _n = tab._apply_move(move)
    assert close(t[mover], center(chess.D6)), "e.p. mover target wrong"
    black_slots = [(float(x), float(y)) for pid, (x, y, _t)
                   in PIECES.GRAVEYARD_POSITIONS.items() if pid >= 0x12]
    assert any(close(t[ep_pawn], s) for s in black_slots), \
        "e.p. captured pawn not graveyarded"
    assert chess.D5 not in tab._sq_to_id
    assert_consistent(tab)
    print("  en passant: OK")


def test_promotion():
    board = BoardState()
    tab = PlayChessTab(board, None)
    tab._chess = chess.Board("4k3/P7/8/8/8/8/8/4K3 w - - 0 1")
    tab._spare = {"white": 0x11, "black": 0x22}
    tab._sq_to_id = {chess.A7: 0x01, chess.E1: 0x0D, chess.E8: 0x1E}
    for sq, pid in tab._sq_to_id.items():
        board.get_piece(pid).position_mm = tab._square_center(sq)
    move = chess.Move.from_uci("a7a8q")
    assert move in tab._chess.legal_moves
    t, _san, _w, _n = tab._apply_move(move)
    white_slots = [(float(x), float(y)) for pid, (x, y, _t)
                   in PIECES.GRAVEYARD_POSITIONS.items() if pid < 0x12]
    assert any(close(t[0x01], s) for s in white_slots), "promoted pawn not graveyarded"
    assert close(t[0x11], center(chess.A8)), "spare queen not sent to a8"
    assert tab._sq_to_id[chess.A8] == 0x11 and tab._spare["white"] is None
    assert_consistent(tab)
    print("  promotion (auto-queen via spare): OK")


def test_dispatch_emits_commands():
    _board, tab = fresh()
    got = {}
    tab.send_commands.connect(lambda cmds, traj: got.update(n=len(cmds), traj=traj))
    move = chess.Move.from_uci("e2e4")
    targets, _san, _w, _n = tab._apply_move(move)
    tab._dispatch(targets)
    assert got.get("n", 0) > 0, "no commands emitted"
    print(f"  dispatch via selected planner: {got['n']} commands, trajectory={got['traj']}")


def main():
    app = QApplication.instance() or QApplication([])
    test_normal_and_capture()
    test_castling()
    test_en_passant()
    test_promotion()
    test_dispatch_emits_commands()
    from gui.main_window import MainWindow  # noqa: F401
    print("PASS: all chess-mode checks green; MainWindow + PlayChessTab import clean.")
    del app


if __name__ == "__main__":
    main()
