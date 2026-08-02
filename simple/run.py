#!/usr/bin/env python3
"""The only command on every machine: python3 run.py."""
from __future__ import annotations
import json, os, runpy, subprocess, sys
from pathlib import Path

root = Path(__file__).resolve().parent
cfg = json.loads((root / "device.json").read_text())
role = cfg["role"]
target = root / role / "run.py"
if not target.exists():
    raise SystemExit(f"missing agent for role {role}: {target}")
# The VLM packages are built in the drone's existing sverk_ws overlay.  A
# normal `python3 run.py` must see that overlay too; users should not have to
# remember a separate `source .../setup.bash` command.
if role == "drone" and os.path.exists("/home/sverk/sverk_ws/install/setup.bash"):
    quoted_target = str(target).replace("'", "'\\\"'\\\"'")
    command = "source /opt/ros/humble/setup.bash; source /home/sverk/sverk_ws/install/setup.bash; exec " + repr(sys.executable) + " '" + quoted_target + "'"
    raise SystemExit(subprocess.call(["bash", "-lc", command]))
if role == "rover" and os.path.exists("/home/pi/install/setup.bash"):
    quoted_target = str(target).replace("'", "'\\\"'\\\"'")
    domain = str(cfg.get("ros_domain_id", "")).strip()
    domain_export = f"export ROS_DOMAIN_ID={domain}; " if domain else ""
    command = "source /opt/ros/jazzy/setup.bash; source /home/pi/install/setup.bash; " + domain_export + "exec " + repr(sys.executable) + " '" + quoted_target + "'"
    raise SystemExit(subprocess.call(["bash", "-lc", command]))
runpy.run_path(str(target), run_name="__main__")
