#!/usr/bin/env bash

set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KEYWORD="telnet starwarstel.net"
MAZE_ROOT="$SCRIPT_DIR/../.maze"
KEY_DIR="$MAZE_ROOT/room_01/room_02/room_03/room_04/room_05/room_06/room_07/room_08/room_09/room_10/room_11/room_12/room_13/room_14/room_15/key_chamber/data"
TARGET="$HOME/git/ros2_student_tutorial/lesson_1/key_data"

mkdir -p "$KEY_DIR"

i=1
for (( idx=0; idx<${#KEYWORD}; idx++ )); do
  letter="${KEYWORD:$idx:1}"
  echo "$letter" > "$KEY_DIR/${i}.txt"
  i=$((i+1))
done

mkdir -p "$TARGET"
sudo chown root:root "$TARGET"
sudo chmod 755 "$TARGET"

cat > "$TARGET/../reconstruct_treasure.sh" << 'EOF'
#!/usr/bin/env bash

FOLDER="key_data"
KEYWORD=""
for f in $(ls "$FOLDER"/*.txt | sort -V); do
        KEYWORD="${KEYWORD}$(cat "$f" | tr -d '\n')"
done

echo "$KEYWORD"
echo
read -p "Press ENTER to collect your treasure!!!" _

eval "$KEYWORD"
EOF
