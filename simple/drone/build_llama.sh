#!/usr/bin/env bash
# One-time, resumable llama.cpp build for the A733 drone.
set -euo pipefail
cd /home/sverk/llama.cpp
cmake -S /home/sverk/llama.cpp -B /home/sverk/llama.cpp/build \
  -DGGML_NATIVE=OFF \
  -DGGML_CPU_ARM_ARCH=armv8.2-a \
  -DGGML_OPENMP=OFF \
  -DLLAMA_BUILD_TESTS=OFF
cmake --build /home/sverk/llama.cpp/build --target llama-cli -- -j4
