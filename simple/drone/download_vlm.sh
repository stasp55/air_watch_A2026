#!/usr/bin/env bash
# One-time, resumable VLM model download for the drone.
set -euo pipefail
mkdir -p /home/sverk/vlm_models
wget -c -O /home/sverk/vlm_models/SmolVLM-256M-Instruct-Q4_K_M.gguf \
  https://huggingface.co/pierretokns/SmolVLM-256M-Instruct-GGUF/resolve/main/SmolVLM-256M-Instruct-Q4_K_M.gguf
wget -c -O /home/sverk/vlm_models/mmproj-SmolVLM-256M-Instruct-f16.gguf \
  https://huggingface.co/pierretokns/SmolVLM-256M-Instruct-GGUF/resolve/main/mmproj-SmolVLM-256M-Instruct-f16.gguf
