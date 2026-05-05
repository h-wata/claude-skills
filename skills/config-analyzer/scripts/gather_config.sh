#!/bin/bash
# Gather Claude Code configuration file information
# Usage: bash gather_config.sh [project_dir]

PROJECT_DIR="${1:-.}"

echo "=== Claude Code Configuration Summary ==="
echo ""

# CLAUDE.md files
echo "## CLAUDE.md Files"
for f in ~/.claude/CLAUDE.md "$PROJECT_DIR/CLAUDE.md" "$PROJECT_DIR/.claude/CLAUDE.md"; do
  if [ -f "$f" ]; then
    lines=$(wc -l < "$f")
    size=$(wc -c < "$f")
    imports=$(grep -c '^@\|@[^ ]*\.md' "$f" 2>/dev/null || echo 0)
    echo "  $f: ${lines} lines, ${size} bytes, ${imports} @imports"
  fi
done
echo ""

# Settings files
echo "## Settings Files"
for f in ~/.claude/settings.json "$PROJECT_DIR/.claude/settings.json" "$PROJECT_DIR/.claude/settings.local.json"; do
  if [ -f "$f" ]; then
    size=$(wc -c < "$f")
    echo "  $f: ${size} bytes"
  fi
done
echo ""

# MCP config
echo "## MCP Config"
for f in ~/.claude.json "$PROJECT_DIR/.mcp.json"; do
  if [ -f "$f" ]; then
    size=$(wc -c < "$f")
    echo "  $f: ${size} bytes"
  fi
done
echo ""

# Skills
echo "## Skills"
for d in ~/.claude/skills "$PROJECT_DIR/.claude/skills"; do
  if [ -d "$d" ]; then
    echo "  $d:"
    for s in "$d"/*/; do
      [ -d "$s" ] && echo "    - $(basename "$s")"
    done
    # Check symlinks
    for s in "$d"/*; do
      [ -L "$s" ] && echo "    - $(basename "$s") -> $(readlink "$s")"
    done
  fi
done
echo ""

# Agents
echo "## Agents"
for d in ~/.claude/agents "$PROJECT_DIR/.claude/agents"; do
  if [ -d "$d" ]; then
    echo "  $d:"
    for a in "$d"/*.md; do
      [ -f "$a" ] && echo "    - $(basename "$a")"
    done
  fi
done
echo ""

echo "=== End ==="
