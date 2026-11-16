#!/usr/bin/env python3
"""Compatibility import for the original standalone conversion helper."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from dracomancer_mkii_ros.xm430 import CurrentScale, CurrentSession
