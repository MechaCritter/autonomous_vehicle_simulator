import json
from PyQt6.QtCore import Qt, QRectF
from PyQt6.QtGui import QBrush, QPen, QPainter, QColor, QImage
from PyQt6.QtWidgets import QApplication, QGraphicsScene, QGraphicsView, QGraphicsRectItem
from PyQt6.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QInputDialog, QFileDialog

from python import config

class MapEditor(QWidget):
    def __init__(self):
        super().__init__()
        # Prompt user fors map dimensions (in pixels)
        width, ok = QInputDialog.getInt(self, "Map Width", "Enter map width (pixels):", 500, config.MIN_MAP_SIZE, config.MAX_MAP_SIZE)
        if not ok: width = config.MIN_MAP_SIZE
        height, ok = QInputDialog.getInt(self, "Map Height", "Enter map height (pixels):", config.MIN_MAP_SIZE, 1, config.MAX_MAP_SIZE)
        if not ok: height = config.MIN_MAP_SIZE
        self.map_width = width
        self.map_height = height
        self.object_sizes: dict = {}
        self.cell_colors: dict = {}

        with open(config.COLOR_JSON) as f:
            color_data = json.load(f)
            self.cell_colors = {name: QColor(*rgb) for name, rgb in color_data.items()}

        # Convert color lists to QColor (assuming [R,G,B] format in JSON)


        # Load object sizes (meters) from config and compute pixel sizes
        with open(config.OBJECTS_SIZE_JSON) as f:
            self.object_sizes = json.load(f)

        self.resolution = 0.1  # meters per pixel (default)
        for t, vals in self.object_sizes.items():
            vals["length_px"] = int(vals["length"] / self.resolution)
            vals["width_px"]  = int(vals["width"]  / self.resolution)

        # Set up the graphics scene and view
        self.scene = QGraphicsScene(0, 0, width, height)
        self.scene.setSceneRect(0, 0, width, height)
        self.view = MapView(self.scene, self)
        self.view.setBackgroundBrush(QBrush(self.cell_colors["Unknown"]))  # Unknown background
        self.view.setFixedSize(width, height)
        self.view.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.view.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)

        # Layout and controls
        self.current_type = "Obstacle"
        btn_layout = QHBoxLayout()
        for obs, col in self.cell_colors.items():
            if obs in self.object_sizes and obs not in ('Unknown', 'Free'):
                btn = QPushButton(obs)
                btn.setStyleSheet(f"background-color: {col.name()}")
                btn.clicked.connect(lambda _, typ=obs: self.set_current_type(typ))
                btn_layout.addWidget(btn)
        save_btn = QPushButton("Save Map")
        save_btn.clicked.connect(self.save_map)
        btn_layout.addWidget(save_btn)

        layout = QVBoxLayout(self)
        layout.addWidget(self.view)
        layout.addLayout(btn_layout)
        self.setWindowTitle("Map Editor")

    def set_current_type(self, typ: str):
        """Change the type of object to place next."""
        self.current_type = typ

    def add_object_at(self, x: float, y: float):
        """Place a new object of current_type centered at scene coordinates (x, y)."""
        t = self.current_type
        # Determine object dimensions in pixels
        length_px = self.object_sizes[t]["length_px"]
        width_px  = self.object_sizes[t]["width_px"]
        half_len = length_px / 2.0
        half_wid = width_px / 2.0
        # Clamp position so object stays fully within bounds
        if x - half_len < 0:           x = half_len
        if y - half_wid < 0:           y = half_wid
        if x + half_len > self.map_width:   x = self.map_width - half_len
        if y + half_wid > self.map_height:  y = self.map_height - half_wid
        # Create a rectangle item for the object
        rect = QGraphicsRectItem(-half_len, -half_wid, length_px, width_px)
        rect.setTransformOriginPoint(0, 0)
        rect.setBrush(QBrush(self.cell_colors[t]))
        # rect.setPen(QPen(Qt.GlobalColor.white, 1, Qt.PenStyle.DotLine) if t == "Vehicle" else QPen(Qt.GlobalColor.white, 0))
        rect.setPen(QPen(Qt.GlobalColor.white, 1, Qt.PenStyle.DotLine) if t == "Vehicle" else QPen(Qt.PenStyle.NoPen))
        rect.setFlags(QGraphicsRectItem.GraphicsItemFlag.ItemIsMovable |
                      QGraphicsRectItem.GraphicsItemFlag.ItemIsSelectable |
                      QGraphicsRectItem.GraphicsItemFlag.ItemIsFocusable)
        rect.setData(0, t)             # store object type
        rect.setRotation(0.0)          # default orientation
        rect.setPos(x, y)
        self.scene.addItem(rect)
        # Remove any object that overlaps the new one (overwrite spot)
        for item in rect.collidingItems():
            if item.data(0) is not None:
                self.scene.removeItem(item)

    def save_map(self):
        """Save the map image and metadata to files."""
        # Prompt for save location
        filename, _ = QFileDialog.getSaveFileName(self, "Save Map As", "map.bmp", "Bitmap (*.bmp)")
        if not filename:
            return
        if not filename.lower().endswith(".bmp"):
            filename += ".bmp"
        # Render the scene (background + objects) to an image
        image = QImage(self.map_width, self.map_height, QImage.Format.Format_RGB888)
        image.fill(self.cell_colors["Unknown"])  # fill background with Unknown color
        painter = QPainter(image)
        self.scene.render(painter, QRectF(0, 0, self.map_width, self.map_height))
        painter.end()
        image.save(filename, "BMP")
        # Prepare metadata about objects
        objects_list = []
        for item in self.scene.items():
            obj_type = item.data(0)
            if obj_type is None:
                continue  # skip any auxiliary items
            cx, cy   = item.x(), item.y()
            angle    = item.rotation()
            objects_list.append({
                "type": obj_type,
                "x": int(round(cx)),   # center position (pixels)
                "y": int(round(cy)),
                "rotation": angle      # rotation in degrees
            })
        meta = { "objects": objects_list }
        meta_file = filename.rsplit('.', 1)[0] + ".json"
        with open(meta_file, 'w') as jf:
            json.dump(meta, jf, indent=4)
        print(f"Saved map image to {filename} and metadata to {meta_file}")

    def keyPressEvent(self, event):
        """Intercept Delete and Arrow keys for object manipulation."""
        if event.key() == Qt.Key.Key_Delete:
            # Delete all selected items
            for item in self.scene.selectedItems():
                self.scene.removeItem(item)
        elif event.key() in (Qt.Key.Key_Left, Qt.Key.Key_Right):
            # Rotate selected items by ±22.5 degrees
            for item in self.scene.selectedItems():
                current_angle = item.rotation()
                delta = -22.5 if event.key() == Qt.Key.Key_Left else 22.5
                item.setRotation(current_angle + delta)

class MapView(QGraphicsView):
    """GraphicsView that places new objects on left-click and handles rotation."""
    def __init__(self, scene: QGraphicsScene, editor: MapEditor):
        super().__init__(scene)
        self.editor = editor
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)  # <- important: accept key events

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            pos = self.mapToScene(event.position().toPoint())
            if self.scene().itemAt(pos, self.transform()) is None:
                self.editor.add_object_at(pos.x(), pos.y())
        super().mousePressEvent(event)

    def keyPressEvent(self, event):
        """Rotate/delete selected items. Shift = fine rotation (1°)."""
        key = event.key()
        if key in (Qt.Key.Key_Left, Qt.Key.Key_Right):
            items = self.scene().selectedItems()
            if items:
                step = 1.0 if (event.modifiers() & Qt.KeyboardModifier.ShiftModifier) else 22.5
                delta = -step if key == Qt.Key.Key_Left else step
                for it in items:
                    it.setRotation(it.rotation() + delta)
                event.accept()
                return
        elif key in (Qt.Key.Key_Delete, Qt.Key.Key_Backspace):
            for it in self.scene().selectedItems():
                self.scene().removeItem(it)
            event.accept()
            return
        super().keyPressEvent(event)

    def wheelEvent(self, event):
        """Ctrl + wheel to rotate selected items by 5° per notch; wheel alone behaves normally."""
        if event.modifiers() & Qt.KeyboardModifier.ControlModifier:
            items = self.scene().selectedItems()
            if items:
                # angleDelta().y() > 0 -> wheel up; < 0 -> wheel down
                sign = 1 if event.angleDelta().y() > 0 else -1
                for it in items:
                    it.setRotation(it.rotation() + 5.0 * sign)
                event.accept()
                return
        super().wheelEvent(event)

# Run the application (if using as a script)
if __name__ == "__main__":
    app = QApplication([])
    editor = MapEditor()
    editor.show()
    app.exec()
