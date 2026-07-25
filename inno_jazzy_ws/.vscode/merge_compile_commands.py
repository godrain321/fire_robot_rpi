#!/usr/bin/env python3
"""Merge package-level CMake compilation databases for editor tooling."""

import json
from pathlib import Path


workspace = Path(__file__).resolve().parent.parent
entries = []

for database in sorted((workspace / "build").glob("*/compile_commands.json")):
    with database.open(encoding="utf-8") as stream:
        entries.extend(json.load(stream))

output = workspace / "compile_commands.json"
output.write_text(json.dumps(entries, indent=2) + "\n", encoding="utf-8")
print(f"Wrote {len(entries)} entries to {output}")
