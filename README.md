# Claude Skills Collection

A collection of custom skills for [Claude Code](https://docs.anthropic.com/en/docs/claude-code) that extend Claude's capabilities with specialized workflows.

[日本語版 README はこちら](README_ja.md)

## Overview

This repository manages all personal Claude Code skills in one place. Each skill is a self-contained directory under `skills/` with a `SKILL.md` that provides Claude with domain-specific instructions and workflows.

```
~/work/claude-skills/              <- this repo
├── .claude-plugin/
│   └── marketplace.json           <- plugin marketplace definition
├── config.json                    <- shared config (e.g. Obsidian paths)
├── skills/
│   ├── memo/SKILL.md
│   ├── survey/SKILL.md
│   └── ...
```

## Available Skills (26)

### Documentation & Logging

| Skill | Description | Slash |
|-------|-------------|-------|
| **memo** | Quick memo to Obsidian Daily note | `/memo` |
| **work-log** | Log work notes from the current conversation | `/work-log` |
| **obsidian-work-logger** | Structured session work logs in Obsidian format | `/obsidian-work-logger` |
| **interview** | Interactive interview to elicit thoughts and ideas | `/interview` |
| **meeting-minutes** | Create / update meeting minutes | `/meeting-minutes` |
| **survey** | Build index summaries from PDFs, repos, or web docs | `/survey` |

### Planning & Specification

| Skill | Description | Slash |
|-------|-------------|-------|
| **plan-with-qa** | Clarify vague specs via AskUserQuestion, then output a Markdown plan | `/plan-with-qa` |
| **write-spec** | Generate implementation specs from survey results, source, or PDFs | `/write-spec` |
| **empirical-prompt-tuning** | Iteratively improve agent prompts via blind execution + dual-side evaluation | `/empirical-prompt-tuning` |

### Review & Analysis

| Skill | Description | Slash |
|-------|-------------|-------|
| **cross-review** | Cross-document consistency review across multiple files | `/cross-review` |
| **codex-consult** | Design consultation via Codex CLI (read-only) | `/codex-consult` |
| **codex-review** | Code review via Codex CLI | `/codex-review` |
| **config-analyzer** | Audit Claude Code config (CLAUDE.md, settings, hooks, skills, MCP) against best practices | `/config-analyzer` |

### Git & Release Workflow

| Skill | Description | Slash |
|-------|-------------|-------|
| **git-history** | Investigate git history and trace change provenance | `/git-history` |
| **safe-pathspec-commit** | Commit only target files without dragging in parallel WIP | `/safe-pathspec-commit` |
| **inherit-wip** | Take over and finish a predecessor / paused worker's uncommitted WIP | `/inherit-wip` |
| **release-apply** | Apply drafted release notes / CHANGELOG / migration; tag, push, GH release, close issues | `/release-apply` |

### Architecture & Decision Records

| Skill | Description | Slash |
|-------|-------------|-------|
| **adr** | Create / list / supersede Architecture Decision Records (manual only) | `/adr` |

### ROS2, Robotics & 3DGS

| Skill | Description | Slash |
|-------|-------------|-------|
| **ros2-inspector** | Generate rclpy scripts to inspect ROS2 topics / nodes / params | `/ros2-inspector` |
| **ros-analyze** | ROS2 system state analysis (nodes, topics, fleet) | `/ros-analyze` |
| **ros2-launch-debug** | Debug ROS2 launch files (params, dependencies, QoS) | `/ros2-launch-debug` |
| **analyze-logs** | ROS / kachaka-api log analysis with error-code reference | `/analyze-logs` |
| **nerfstudio-trainer** | Launch nerfstudio splatfacto (3D Gaussian Splatting) training with project-tuned params | `/nerfstudio-trainer` |

### mesh-mem / MCP / Distributed Tooling

| Skill | Description | Slash |
|-------|-------------|-------|
| **add-mesh-peer** | Add a new host to an existing mesh-mem (Zenoh) mesh end-to-end | `/add-mesh-peer` |
| **mcp-smoke-via-claude-p** | Run automated 5-case MCP smoke tests via `claude -p --output-format json` | `/mcp-smoke-via-claude-p` |

### Setup & Configuration

| Skill | Description | Slash |
|-------|-------------|-------|
| **setup-claude-local** | Bootstrap local ADR + `.claude/CLAUDE.md` + Obsidian vault integration | `/setup-claude-local` |
| **config-analyzer** | (See Review & Analysis above) | `/config-analyzer` |

## Installation

### Method 1: Plugin install (recommended)

Register this repo as a plugin marketplace and install the bundled skills as a plugin:

```bash
# Add this repo as a marketplace
claude plugin marketplace add h-wata/claude-skills

# Install all skills
claude plugin install h-wata-skills@h-wata-claude-skills
```

Or use the interactive plugin manager:

```
/plugin
```

### Method 2: Symlink (for development)

```bash
git clone https://github.com/h-wata/claude-skills.git ~/work/claude-skills
cd ~/work/claude-skills

mkdir -p ~/.claude/skills
for skill in skills/*/; do
  skill_name="$(basename "$skill")"
  [ -f "$skill/SKILL.md" ] && ln -sf "$(pwd)/$skill" ~/.claude/skills/$skill_name
done
```

### Configuration

`config.json` holds shared settings referenced by multiple skills (e.g. the Obsidian Daily note path):

```json
{
    "obsidian_daily_path": "~/Documents/my-obsidian/Daily"
}
```

## Repository Structure

```
claude-skills/
├── .claude-plugin/
│   └── marketplace.json      # Plugin marketplace definition
├── config.json               # Shared config for skills
├── skills/
│   └── <skill-name>/
│       ├── SKILL.md          # YAML frontmatter + instructions (required)
│       ├── scripts/          # Executable helpers (optional)
│       ├── references/       # Documentation resources (optional)
│       └── assets/           # Templates / output files (optional)
```

### SKILL.md frontmatter

```yaml
---
name: skill-name
description: What the skill does and when to use it
allowed-tools: Read, Grep, Bash    # Tools allowed without permission prompt
disable-model-invocation: true     # Manual-only invocation via /name
---
```

## Prerequisites

- [Claude Code](https://docs.anthropic.com/en/docs/claude-code) installed
- For ROS2 skills: ROS2 environment (Humble / Iron / Jazzy)
- For `nerfstudio-trainer`: nerfstudio + a CUDA-capable GPU
- For `add-mesh-peer` / `mcp-smoke-via-claude-p`: Zenoh / MCP environment
- For Obsidian-backed skills: an Obsidian vault and `config.json` pointing at it

## References

- [Claude Code Skills Documentation](https://docs.anthropic.com/en/docs/claude-code/skills)
- [Claude Code Plugins Documentation](https://docs.anthropic.com/en/docs/claude-code/plugins)
- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [Obsidian Documentation](https://help.obsidian.md/)

## License

Apache License 2.0 — see [LICENSE](LICENSE) for details.
