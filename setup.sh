#!/bin/bash
set -e

# Install dependencies for packages currently present in src/.
if [ -d "src" ]; then
  sudo apt-get update
  rosdep update
  rosdep install --from-paths src --ignore-src -y
fi