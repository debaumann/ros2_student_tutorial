#!/usr/bin/env bash

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source $SCRIPT_DIR/generate_treasure_map.sh
source $SCRIPT_DIR/generate_puzzle.sh
source $SCRIPT_DIR/down_the_rabbit_hole.sh
