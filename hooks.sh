#!/usr/bin/env bash
set -euo pipefail

HOOKS=".git/hooks"
mkdir -p "$HOOKS"

# -------- prepare-commit-msg --------
cat > "$HOOKS/prepare-commit-msg" <<'H1'
#!/usr/bin/env bash
set -euo pipefail
FILE="$1"; SOURCE="${2:-}"

# Skip merges/rebases or if already has a message
if [ -n "$SOURCE" ] || [ -s "$FILE" ]; then
  exit 0
fi

cat > "$FILE" <<'EOM'
# <type>: <summary>
# Allowed types:
# Feat   new feature
# Fix    bug fix
# Perf   performance improvement
# Chore  build/tooling
# Docs   documentation
# Test   tests
# Merge  merge commit
#
# Example:
# Feat: add user login endpoint
EOM
H1
chmod +x "$HOOKS/prepare-commit-msg"

# -------- commit-msg (enforce types) --------
cat > "$HOOKS/commit-msg" <<'H2'
#!/usr/bin/env bash
set -euo pipefail
FILE="$1"
ALLOWED="Feat|Fix|Perf|Chore|Docs|Test|Merge"

# First line (trim leading spaces)
FIRST_LINE="$(sed -n '1{s/^[[:space:]]*//;p;q;}' "$FILE")"

# Allow git's default merge messages like "Merge branch ..."
if echo "$FIRST_LINE" | grep -Eq "^Merge(\b|:)"; then
  exit 0
fi

# Must start with <Type>: summary
if ! echo "$FIRST_LINE" | grep -Eq "^(${ALLOWED}):[[:space:]]+.+$"; then
  echo "❌ Invalid commit message!"
  echo "👉 Must start with one of: ${ALLOWED}"
  echo "👉 Example: Feat: add user login endpoint"
  echo "Your message: '$FIRST_LINE'"
  exit 1
fi
H2
chmod +x "$HOOKS/commit-msg"

echo "✅ Hooks installed: prepare-commit-msg & commit-msg"