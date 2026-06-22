"""
main.py  —  MiniBot Chess Swarm Coordinator

Entry point.  Creates the QApplication and shows the main window.

Run with:
    python main.py
"""

import sys

from PyQt6.QtWidgets import QApplication, QStyleFactory

from gui.main_window import MainWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("MiniBot Coordinator")
    app.setOrganizationName("MiniBot")
    app.setStyle(QStyleFactory.create("Fusion"))

    window = MainWindow()
    window.show()

    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
