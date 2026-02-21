#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT_DIR"

java_works() {
  command -v java >/dev/null 2>&1 && java -version >/dev/null 2>&1
}

discover_wpilib_java_home() {
  local base
  local candidate

  for base in "${HOME}/wpilib" "/Users/Shared/wpilib"; do
    [[ -d "${base}" ]] || continue
    while IFS= read -r candidate; do
      if [[ -x "${candidate}/bin/java" ]]; then
        echo "${candidate}"
        return 0
      fi
    done < <(find "${base}" -maxdepth 2 -type d \( -name 'jdk' -o -name 'jdk-*' \) 2>/dev/null | sort -r)
  done

  return 1
}

# Highest priority override for unusual setups.
if [[ -n "${DASHBOARD_JAVA_HOME:-}" && -x "${DASHBOARD_JAVA_HOME}/bin/java" ]]; then
  export JAVA_HOME="${DASHBOARD_JAVA_HOME}"
  export PATH="${JAVA_HOME}/bin:${PATH}"
fi

# If JAVA_HOME is set and valid, ensure it's on PATH.
if [[ -n "${JAVA_HOME:-}" && -x "${JAVA_HOME}/bin/java" ]]; then
  export PATH="${JAVA_HOME}/bin:${PATH}"
fi

# If java is missing or unusable (macOS stub), discover WPILib JDK automatically.
if ! java_works; then
  if JAVA_DISCOVERED="$(discover_wpilib_java_home)"; then
    export JAVA_HOME="${JAVA_DISCOVERED}"
    export PATH="${JAVA_HOME}/bin:${PATH}"
  fi
fi

if ! java_works; then
  echo "No Java runtime found."
  echo "Install WPILib (which includes Java), or set DASHBOARD_JAVA_HOME/JAVA_HOME."
  echo "Searched under: ${HOME}/wpilib and /Users/Shared/wpilib"
  exit 1
fi

# Default to team lookup for competition use.
DASH_ARGS="--team 3318"
if [[ $# -gt 0 ]]; then
  DASH_ARGS="$*"
fi

exec ./gradlew runDashboard --args="$DASH_ARGS"
