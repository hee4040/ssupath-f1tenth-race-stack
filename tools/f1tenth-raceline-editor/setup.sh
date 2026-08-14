#!/usr/bin/env bash
set -euo pipefail

if ! command -v uv >/dev/null 2>&1; then
  echo "ERROR: uv is not installed."
  echo "Install uv first: https://docs.astral.sh/uv/getting-started/installation/"
  exit 1
fi

if ! command -v git >/dev/null 2>&1; then
  echo "ERROR: git is not installed or not on PATH."
  exit 1
fi

uv sync
uv run raceline bootstrap
uv run raceline doctor
