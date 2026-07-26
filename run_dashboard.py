#!/usr/bin/env python3
"""
Launcher for the 2026 Interactive Web Dashboard for Autonomous Choke Control System.
Serves modern HTML/CSS/JS glassmorphism web app on http://localhost:8050.

Keeps the original Matplotlib dashboard available via:
    python dashboard/dashboard_mpl.py
    or
    python run_dashboard.py --mpl
"""

import sys
import os

if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--mpl":
        print("Launching Matplotlib backup dashboard...")
        os.system(f"{sys.executable} dashboard/dashboard_mpl.py")
    else:
        from dashboard.server import run_server
        port = 8050
        if len(sys.argv) > 1 and sys.argv[1].isdigit():
            port = int(sys.argv[1])
        run_server(port=port, open_browser=True)