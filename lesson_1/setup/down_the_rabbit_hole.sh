#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MAZE_ROOT="$SCRIPT_DIR/.maze"
LEVELS="${LEVELS:-15}"
SIBLINGS_PER_LEVEL="${SIBLINGS_PER_LEVEL:-8}"
BASHRC="$HOME/.bashrc"
TAG_START="# >>> Treasure Hunt Alias Challenge START >>>"
TAG_END="# <<< Treasure Hunt Alias Challenge END <<<"

echo "$MAZE_ROOT"
mkdir -p "$MAZE_ROOT"

current="$MAZE_ROOT"
for i in $(seq -w 1 "$LEVELS"); do
  level_dir="room_${i}"
  current="$current/$level_dir"
  mkdir -p "$current"
  for s in $(seq 1 "$SIBLINGS_PER_LEVEL"); do
    mkdir -p "$MAZE_ROOT/$level_dir/noise_${s}" 2>/dev/null || true
  done
done

KEY_DIR="$current/key_chamber"
mkdir -p "$KEY_DIR"
echo "ALIAS_UNLOCKED" > "$KEY_DIR/key.txt"

#remove inserted alias if rerun
if grep -q "$TAG_START" "$BASHRC"; then
  awk -v start="$TAG_START" -v end="$TAG_END" '
    BEGIN{skip=0}
    $0==start{skip=1; next}
    $0==end{skip=0; next}
    skip==0{print}
  ' "$BASHRC" > "$BASHRC.tmp" && mv "$BASHRC.tmp" "$BASHRC"
fi
path=""
for i in $(seq -w 1 "$LEVELS"); do
  path="${path}room_${i}/"
done
cat >> "$BASHRC" << EOF
$TAG_START
# Broken on purpose: single quotes stop \$HOME from expanding.
# Fix by changing to double quotes, or replace \$HOME with ~
#   goto_key   -> jumps straight to the key chamber

alias goto_key='cd \$HOME/git/ros2_student_tutorial/lesson_1/.maze/${path}key_chamber'
alias go_home= "cd ~/git/ros2_student_tutorial/lesson_1"
$TAG_END
EOF
