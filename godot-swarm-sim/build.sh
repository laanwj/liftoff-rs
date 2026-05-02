#!/usr/bin/env bash
# Build the gdext cdylib and stage it into the Godot addons dir so
# Godot's res:// path resolution finds the library.
#
# Usage: ./build.sh [debug|release]   (default: debug)

set -euo pipefail

PROFILE="${1:-debug}"
ROOT="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "$ROOT/.." && pwd)"
ADDON_DIR="$ROOT/godot/addons/godot-swarm-sim"

case "$PROFILE" in
    debug)   CARGO_PROFILE_FLAG="" ; TARGET_SUBDIR="debug" ;;
    release) CARGO_PROFILE_FLAG="--release" ; TARGET_SUBDIR="release" ;;
    *)       echo "unknown profile: $PROFILE" >&2 ; exit 2 ;;
esac

echo "==> cargo build -p godot-swarm-sim ($PROFILE)"
cargo build $CARGO_PROFILE_FLAG -p godot-swarm-sim --manifest-path "$WORKSPACE_ROOT/Cargo.toml"

UNAME="$(uname -s)"
case "$UNAME" in
    Linux*)  LIB_NAME="libgodot_swarm_sim.so" ;;
    Darwin*) LIB_NAME="libgodot_swarm_sim.dylib" ;;
    MINGW*|MSYS*|CYGWIN*) LIB_NAME="godot_swarm_sim.dll" ;;
    *) echo "unknown OS: $UNAME" >&2 ; exit 3 ;;
esac

SRC="$WORKSPACE_ROOT/target/$TARGET_SUBDIR/$LIB_NAME"
DST="$ADDON_DIR/$LIB_NAME"

echo "==> staging $SRC -> $DST"
mkdir -p "$ADDON_DIR"
cp "$SRC" "$DST"

echo "==> done."
