#!/usr/bin/env python3
"""Compatibility import for the original standalone conversion helper."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from dracomancer.xm430 import CurrentScale, CurrentSession
