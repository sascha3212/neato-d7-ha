import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "neato_mapper" / "app"))
sys.path.insert(0, str(ROOT))
