$ErrorActionPreference = "Stop"

if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    Write-Error "uv is not installed. See https://docs.astral.sh/uv/getting-started/installation/"
}

if (-not (Get-Command git -ErrorAction SilentlyContinue)) {
    Write-Error "git is not installed or not on PATH."
}

uv sync
uv run raceline bootstrap
uv run raceline doctor
