
"""
Launcher script for the TUE USB Binary Protocol PyQt6 GUI.
"""
#!/usr/bin/env python3
import sys
import os

# Ensure the python folder is in the python path
current_dir = os.path.dirname(os.path.abspath(__file__))
if current_dir not in sys.path:
    sys.path.insert(0, current_dir)

from gui.app import run_app

if __name__ == "__main__":
    run_app()
