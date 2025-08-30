import os
import pathlib

ROOT = pathlib.Path(__file__).parent.parent.resolve()
COLOR_JSON = os.path.join(ROOT, "res", "maps", "cell_colors.json")
OBJECTS_SIZE_JSON = os.path.join(ROOT, "res", "objects_size.json")
MIN_MAP_SIZE = 500 # [pixels]
MAX_MAP_SIZE = 2000 # [pixels]