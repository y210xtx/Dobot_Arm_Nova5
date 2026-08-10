"""Qt robot-control interface entry point."""

import sys
from pathlib import Path

# The Dobot SDK and alarm tables remain in the parent project directory.
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from ui_qt import run


if __name__ == "__main__":
    raise SystemExit(run())
