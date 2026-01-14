#!/bin/bash
# Setup script for F1Tenth Final Project
# Fixes PyTorch OpenMP TLS allocation issue on ARM64

source /home/f1tenth/Downloads/f1tenth_final_project-main/install/setup.bash
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1

echo "✅ Environment setup complete!"
echo "   LD_PRELOAD set for PyTorch OpenMP compatibility"
