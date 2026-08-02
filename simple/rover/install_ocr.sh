#!/usr/bin/env bash
# Run once on the rover.  The Air Watch agent imports RapidOCR only on demand.
set -euo pipefail

# The ROS Jazzy agent uses the system interpreter, so a venv would not be
# visible to it.  Debian's PEP 668 guard requires this explicit opt-in.
python3 -m pip install --break-system-packages --upgrade rapidocr_onnxruntime
python3 - <<'PY'
import cv2
from rapidocr_onnxruntime import RapidOCR
print("RapidOCR is ready:", RapidOCR.__name__)
PY
