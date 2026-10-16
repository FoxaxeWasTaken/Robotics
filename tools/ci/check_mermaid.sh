#!/usr/bin/env bash
# Checks that every ```mermaid block of the Markdown files renders without error (mermaid-cli in Docker).
# Usage: bash tools/ci/check_mermaid.sh
set -uo pipefail
cd "$(git rev-parse --show-toplevel)" || exit 1
IMAGE=${MERMAID_CLI_IMAGE:-minlag/mermaid-cli:12.0.1}
tmp=$(mktemp -d); trap 'rm -rf "$tmp"' EXIT
chmod 777 "$tmp"
python3 - "$tmp" <<'PY'
import pathlib, re, sys
out, n = pathlib.Path(sys.argv[1]), 0
skip = {"core", "sim", "build", "install", "log", "node_modules", ".git"}
for md in sorted(pathlib.Path(".").rglob("*.md")):
    if skip & set(md.parts):
        continue
    for i, blk in enumerate(re.findall(r"```mermaid\n(.*?)```", md.read_text(encoding="utf-8"), re.S)):
        (out / f"{str(md).replace('/', '__')}__{i}.mmd").write_text(blk, encoding="utf-8"); n += 1
print(f"{n} diagrams extracted")
PY
status=0
for f in "$tmp"/*.mmd; do
  [ -e "$f" ] || continue
  name=$(basename "$f")
  if ! docker run --rm -v "$tmp":/data "$IMAGE" -q -i "/data/$name" -o "/data/$name.svg" >"$tmp/$name.log" 2>&1; then
    echo "❌ $name"; sed 's/^/   /' "$tmp/$name.log" | head -20; status=1
  fi
done
[ $status -eq 0 ] && echo "✅ every Mermaid diagram renders"
exit $status
