"""Simple PyQt6 image annotation tool with adjustable brush size and smoothing.

This module provides a tiny raster editor to quickly label grid-based maps,
for example cost maps used in path‑planning demos.

New in this version
-------------------
* Adjustable brush size via a QSpinBox (1‑50 px).
* “Smooth” button that removes tiny colour speckles using connected‑component
  analysis.

Requires
--------
* Python ≥ 3.9
* PyQt6
"""

from __future__ import annotations

import sys
from collections import deque
from functools import partial

from PyQt6.QtCore import Qt, QPoint
from PyQt6.QtGui import QImage, QPainter, QColor
from PyQt6.QtWidgets import (
    QApplication,
    QFileDialog,
    QWidget,
    QPushButton,
    QVBoxLayout,
    QHBoxLayout,
    QSpinBox,
    QLabel,
)

from config import ROOT

# Global var
DEFAULT_SAVE_DIR = ROOT / "res" / "maps"

# Colour lookup table (index → QColor). Color scheme: RGB
# NOTE: use symmetric colors to avoid confusion between BGR abd RBG across different libraries
CELL_COLOURS: dict[str, QColor] = {
    "Unknown": QColor(0, 0, 0),
    "Free": QColor(64, 64, 64),
    "Road": QColor(128, 128, 128),
    "Vehicle": QColor(255, 0, 255),
    "Obstacle": QColor(0, 255, 0),
    "Reserved": QColor(255, 192, 255)
}


class Canvas(QWidget):
    """Widget that implements a very small raster‑based paint surface."""

    def __init__(self, width: int = 500, height: int = 500) -> None:
        """
        :param width:  Width of the canvas in pixels.
        :param height: Height of the canvas in pixels.
        """
        super().__init__()
        self.img = QImage(width, height, QImage.Format.Format_RGB888)
        self.img.fill(CELL_COLOURS["Unknown"])
        self.current_color = "Unknown"  # currently selected colour (index into CELL_COLOURS)
        self.brush_size = 20 # default brush size in pixels
        self.setFixedSize(width, height)

    # ---------------------------------------------------------------- events
    def mouseMoveEvent(self, event) -> None:  # noqa: N802 (PyQt naming)
        """Paint while the left mouse button is held down."""
        if event.buttons() & Qt.MouseButton.LeftButton:
            pos = QPoint(event.position().toPoint())
            self._paint_at(pos)

    def mousePressEvent(self, event) -> None:  # noqa: N802 (PyQt naming)
        """Start painting on left button press."""
        if event.button() == Qt.MouseButton.LeftButton:
            self._paint_at(QPoint(event.position().toPoint()))

    # --------------------------------------------------------------- helpers
    @property
    def colour(self) -> QColor:
        """Currently selected brush colour."""
        return CELL_COLOURS[self.current_color]

    def set_brush_size(self, size: int) -> None:
        """
        Update brush size used when painting.

        :param size: New brush size in pixels.
        :returns: None
        """
        self.brush_size = max(1, size)

    def _paint_at(self, p: QPoint) -> None:
        """
        Paint a filled rectangle at the given position.

        :param p: QPoint (canvas‑local coordinates) where the user painted.
        :returns: None
        """
        if 0 <= p.x() < self.img.width() and 0 <= p.y() < self.img.height():
            painter = QPainter(self.img)
            painter.setPen(self.colour)
            painter.setBrush(self.colour)
            painter.drawRect(
                p.x() - self.brush_size // 2,
                p.y() - self.brush_size // 2,
                self.brush_size,
                self.brush_size,
            )
            painter.end()
            self.update()

    # -------------------------------------------------------- smoothing algo
    def smooth(self, min_size: int = 20) -> None:
        """
        Remove tiny speckles by merging components smaller than *min_size*.

        A 4‑neighbourhood flood‑fill is used to label connected components.
        Components with fewer than *min_size* pixels are recoloured to the
        colour of the nearest large neighbour.

        :param min_size: Minimum pixel count a component must have to be kept.
        :returns: None
        """
        width, height = self.img.width(), self.img.height()
        comp_map = [[-1] * width for _ in range(height)]
        comp_sizes: list[int] = []
        comp_colours: list[QColor] = []

        def neighbours(x: int, y: int):
            """Yield 4‑neighbour coordinates within bounds."""
            if x > 0:
                yield x - 1, y
            if x + 1 < width:
                yield x + 1, y
            if y > 0:
                yield x, y - 1
            if y + 1 < height:
                yield x, y + 1

        # ---- first pass: label components and measure their sizes ----------
        comp_id = 0
        for y in range(height):
            for x in range(width):
                if comp_map[y][x] >= 0:
                    continue  # already visited

                target_colour = self.img.pixelColor(x, y)
                queue = deque([(x, y)])
                comp_map[y][x] = comp_id
                size = 0

                while queue:
                    cx, cy = queue.popleft()
                    size += 1
                    for nx, ny in neighbours(cx, cy):
                        if comp_map[ny][nx] == -1 and self.img.pixelColor(
                            nx, ny
                        ) == target_colour:
                            comp_map[ny][nx] = comp_id
                            queue.append((nx, ny))

                comp_sizes.append(size)
                comp_colours.append(target_colour)
                comp_id += 1

        # ---- second pass: recolour small components ------------------------
        for y in range(height):
            for x in range(width):
                cid = comp_map[y][x]
                if comp_sizes[cid] >= min_size:
                    continue  # large enough

                # find colour of the nearest large neighbour
                replacement: QColor | None = None
                for nx, ny in neighbours(x, y):
                    nid = comp_map[ny][nx]
                    if comp_sizes[nid] >= min_size:
                        replacement = comp_colours[nid]
                        break

                if replacement is None:
                    replacement = comp_colours[cid]  # fallback

                self.img.setPixelColor(x, y, replacement)

        self.update()

    # ---------------------------------------------------------------- widget
    def paintEvent(self, _) -> None:  # noqa: N802 (PyQt naming)
        qp = QPainter(self)
        qp.drawImage(0, 0, self.img)


class Editor(QWidget):
    """Main window that hosts the canvas and the control widgets."""

    def __init__(self) -> None:
        super().__init__()
        self.canvas = Canvas()

        # ----------------------- save button
        self.save_btn = QPushButton("Save BMP")
        self.save_btn.clicked.connect(self._save)

        # ----------------------- smooth button
        self.smooth_btn = QPushButton("Smooth")
        self.smooth_btn.clicked.connect(self.canvas.smooth)

        # ----------------------- brush size control
        self.brush_spin = QSpinBox()
        self.brush_spin.setRange(1, 100)
        self.brush_spin.setValue(self.canvas.brush_size)
        self.brush_spin.setSuffix(" px")
        self.brush_spin.valueChanged.connect(self.canvas.set_brush_size)

        # ----------------------- palette buttons
        self.palette_btns = [QPushButton(str(i)) for i in range(len(CELL_COLOURS))]
        for btn, color in zip(self.palette_btns, CELL_COLOURS):
            btn.setStyleSheet(f"background:{CELL_COLOURS[color].name()}")
            btn.setText(color)
            btn.clicked.connect(partial(self._set_colour, color))

        # ----------------------- layouts
        root = QVBoxLayout(self)
        root.addWidget(self.canvas)

        controls = QHBoxLayout()
        root.addLayout(controls)

        # colour palette
        for btn in self.palette_btns:
            controls.addWidget(btn)

        # brush size + label
        controls.addWidget(QLabel("Brush:"))
        controls.addWidget(self.brush_spin)

        # smooth + save
        controls.addWidget(self.smooth_btn)
        controls.addWidget(self.save_btn)

        self.setWindowTitle("Grid Editor")

    def _set_colour(self, color: str) -> None:
        """
        Select colour *idx* for subsequent painting.

        :param color: Colour name
        """
        self.canvas.current_color = color

    def _save(self) -> None:
        """
        Open a file‑save dialog and write the current canvas as a BMP file.

        :returns: None
        :raises ValueError: If file extension is invalid
        """
        name, _ = QFileDialog.getSaveFileName(
            self, "Save as BMP", str(DEFAULT_SAVE_DIR / "default_map.bmp"), "Bitmap (*.bmp)"
        )
        if name:
            if not name.lower().endswith('.bmp'):
                if '.' in name:
                    raise ValueError("Only .bmp extension is supported")
                name = name + ".bmp"

            if not self.canvas.img.save(name, "BMP"):
                print(f"Could not save image to {name}", file=sys.stderr)


def main() -> None:
    """Entry‑point helper so running the module works as a script."""
    app = QApplication(sys.argv)
    Editor().show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

