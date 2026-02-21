#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

# Default to team lookup for competition use.
DASH_ARGS="--team 3318"
if [[ $# -gt 0 ]]; then
  DASH_ARGS="$*"
fi

exec ./gradlew runDashboard --args="$DASH_ARGS"
