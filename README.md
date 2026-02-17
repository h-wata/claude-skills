# Claude Skills Collection

A collection of custom skills for [Claude Code](https://docs.anthropic.com/en/docs/claude-code) that extend Claude's capabilities with specialized workflows.

## Overview

This repository manages all personal Claude Code skills in one place. Each skill is a self-contained directory with a `SKILL.md` that provides Claude with domain-specific instructions and workflows.

```
~/work/claude-skills/              <- this repo
├── .claude-plugin/
│   └── marketplace.json           <- plugin marketplace definition
├── config.json                    <- shared config
├── skills/
│   ├── memo/SKILL.md
│   ├── survey/SKILL.md
│   └── ...
```

## Available Skills

### Documentation & Logging

| Skill | Description | Slash Command |
|-------|-------------|---------------|
| **memo** | Quick memo to Obsidian Daily note | `/memo` |
| **work-log** | Log work notes from current conversation | `/work-log` |
| **obsidian-work-logger** | Structured session work logs in Obsidian format | `/obsidian-work-logger` |
| **interview** | Interactive interview for brainstorming, reviews, and note-taking | `/interview` |

### Code Analysis & Review

| Skill | Description | Slash Command |
|-------|-------------|---------------|
| **survey** | Create index summaries from PDFs, repos, and web docs | `/survey` |
| **write-spec** | Generate implementation specs from survey results and source code | `/write-spec` |
| **cross-review** | Cross-document consistency review (5 perspectives) | `/cross-review` |
| **git-history** | Investigate git history and track change provenance | `/git-history` |
| **codex-consult** | Design consultation via Codex CLI (read-only) | `/codex-consult` |
| **codex-review** | Code review via Codex CLI | `/codex-review` |

### ROS2 & Robotics

| Skill | Description | Slash Command |
|-------|-------------|---------------|
| **ros2-inspector** | Generate rclpy scripts to inspect ROS2 systems | `/ros2-inspector` |
| **ros-analyze** | ROS2 system state analysis (nodes, topics, fleet) | `/ros-analyze` |
| **analyze-logs** | ROS/kachaka-api log analysis with error code reference | `/analyze-logs` |

### Planning & Workflow

| Skill | Description | Slash Command |
|-------|-------------|---------------|
| **plan-with-qa** | Implementation planning with interactive Q&A to clarify specs | `/plan-with-qa` |
| **meeting-minutes** | Meeting minutes creation and updates | `/meeting-minutes` |

## Installation

### Method 1: Plugin Install

Register this repository as a plugin marketplace, then install all skills as a plugin:

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

### Method 2: Symlink

For development or if you want to manage skills individually:

```bash
git clone https://github.com/h-wata/claude-skills.git ~/work/claude-skills
cd ~/work/claude-skills

# Create symlinks for all skills
mkdir -p ~/.claude/skills
for skill in skills/*/; do
  skill_name="$(basename "$skill")"
  [ -f "$skill/SKILL.md" ] && ln -sf "$(pwd)/$skill" ~/.claude/skills/$skill_name
done
```

### Configuration

`config.json` holds shared settings used by multiple skills (e.g., Obsidian daily notes path):

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
│   └── skill-name/
│       ├── SKILL.md          # Skill instructions with YAML frontmatter (required)
│       ├── scripts/          # Executable scripts (optional)
│       ├── references/       # Documentation resources (optional)
│       └── assets/           # Templates and output files (optional)
```

### SKILL.md Frontmatter

```yaml
---
name: skill-name
description: What this skill does
allowed-tools: Read, Grep, Bash    # Tools allowed without permission prompt
disable-model-invocation: true     # Manual-only invocation via /name
---
```

## Prerequisites

- [Claude Code](https://docs.anthropic.com/en/docs/claude-code) installed
- For ROS2 skills: ROS2 environment (Humble, Iron, or Jazzy)

## References

- [Claude Code Skills Documentation](https://docs.anthropic.com/en/docs/claude-code/skills)
- [Claude Code Plugins Documentation](https://docs.anthropic.com/en/docs/claude-code/plugins)
- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [Obsidian Documentation](https://help.obsidian.md/)

## License

Apache License 2.0 - see LICENSE file for details.
