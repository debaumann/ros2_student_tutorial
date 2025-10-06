#!/usr/bin/env bash

set -euo pipefail

# Example: always place treasure_map.txt in the repo root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUTFILE="$SCRIPT_DIR/../treasure_map.txt"

# --- Config ---
MIN_GAP="${MIN_GAP:-10000}"
MAX_GAP="${MAX_GAP:-14999}"   # strictly less than 15000
SEED="${SEED:-42}"   # set SEED=42 for reproducible layout
KEYWORDS=("NEBULA" "QUASAR" "HYPERDRIVE")
# ---------------

# Seed bash $RANDOM for reproducible gap choices when SEED is set
RANDOM=$((SEED & 0x7fff))

rand_range() {
  local min="$1" max="$2"
  echo $(( min + (RANDOM % (max - min + 1)) ))
}

emit_noise() {
  local count="$1" seed="$2"
  awk -v n="$count" -v s="$seed" 'BEGIN{
    srand(s);
    for(i=0;i<n;i++){
      line="";
      for(j=0;j<64;j++){
        c=int(rand()*62);
        if(c<10)       line=line sprintf("%d", c);         # 0-9
        else if(c<36)  line=line sprintf("%c", 55+c);      # A-Z
        else           line=line sprintf("%c", c+61);      # a-z
      }
      print line
    }
  }'
}

: > "$OUTFILE"

# Pick gaps (≥10000 and <15000)
gap1="$(rand_range "$MIN_GAP" "$MAX_GAP")"
gap2="$(rand_range "$MIN_GAP" "$MAX_GAP")"
gap3="$(rand_range "$MIN_GAP" "$MAX_GAP")"
tail_gap="$(rand_range "$MIN_GAP" "$MAX_GAP")"

# Emit: [gap1 noise] -> clue1 -> [gap2 noise] -> clue2 -> [gap3 noise] -> clue3 -> [tail noise optional]
emit_noise "$gap1" "$((SEED+1000))" >> "$OUTFILE"
echo "(\"${KEYWORDS[0]}\")FoIoXoTjHjEjAtLtItAtStIgNgToHoEoBpApSpHpRpCpUqSqEqDyOyUyByLyEyQzUzOzTzEzS " >> "$OUTFILE"

emit_noise "$gap2" "$((SEED+2000))" >> "$OUTFILE"
echo "(\"${KEYWORDS[1]}\")CpHpApNpGpEpOoWoNoEoRoSoHoIoPoOtFtTyHyEyKqEqYqDuAuTuAuFvOvLvDvEvRv" >> "$OUTFILE"

emit_noise "$gap3" "$((SEED+3000))" >> "$OUTFILE"
echo "(\"${KEYWORDS[2]}\")CpHpMpOpDp+pXpTjHjEjSgCgRgIgPgTgAsNsDsFyIyXyTzHzEzLzOzOzPz" >> "$OUTFILE"

emit_noise "$tail_gap" "$((SEED+4000))" >> "$OUTFILE"

echo "Wrote $(wc -l < "$OUTFILE") lines to $OUTFILE"
echo "   Gaps: $gap1, $gap2, $gap3  (each in [$MIN_GAP, $MAX_GAP])"

