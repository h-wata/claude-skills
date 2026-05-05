---
name: config-analyzer
description: Analyze and audit Claude Code configuration files (CLAUDE.md, settings.json, settings.local.json, hooks, skills, agents, MCP servers) against Anthropic's official best practices. Use when the user asks to review, audit, check, or improve their Claude Code setup, or when they want to compare their configuration against best practices. Triggers on requests like "analyze my Claude config", "review my CLAUDE.md", "audit my settings", "improve my Claude Code setup", or "/config-analyzer".
---

# Config Analyzer

Analyze Claude Code configuration against Anthropic's official best practices and provide actionable improvement recommendations.

## Workflow

### 1. Gather Configuration Files

Read the following files (skip if not found):

```
~/.claude/CLAUDE.md                    # Global CLAUDE.md
~/.claude/settings.json                # User settings
~/.claude.json                         # User MCP servers
.claude/settings.json                  # Project settings (if in a project)
.claude/settings.local.json            # Local settings (if in a project)
.mcp.json                              # Project MCP servers (if in a project)
CLAUDE.md                              # Project CLAUDE.md (if in a project)
```

Also scan:
- `~/.claude/skills/` — installed skills (list names and symlink targets)
- `~/.claude/agents/` — custom subagents
- `.claude/skills/` and `.claude/agents/` — project-level (if in a project)

### 2. Analyze Each Component

For each component found, evaluate against best practices in [references/best-practices.md](references/best-practices.md).

#### CLAUDE.md Analysis
- **Line count**: Flag if over 200 lines (target: ~60 lines)
- **Content audit**: Check for items that should be excluded (self-evident rules, info derivable from code, detailed API docs, frequently-changing info)
- **Missing essentials**: Check for bash commands, code style, testing instructions, workflow rules
- **Structure**: Readability, use of emphasis for critical rules
- **@imports**: Check if referenced files exist
- **Scope correctness**: Global vs project-level content placement

#### settings.json Analysis
- **Permissions**: Check for overly broad allows, missing denys for dangerous ops
- **Scope hygiene**: Personal settings in project files, redundant settings across scopes
- **Hooks**: Validate hook configuration, check for missing useful hooks
- **Plugins**: List installed plugins, check for conflicts

#### Skills Analysis
- **Skill inventory**: List all skills with descriptions
- **CLAUDE.md alignment**: Check if skills overlap with CLAUDE.md content (move domain knowledge to skills)
- **Trigger coverage**: Check if descriptions are specific enough

#### Subagents Analysis
- **Tool scope**: Check if tools list follows least privilege
- **Model selection**: Verify appropriate model for task complexity
- **System prompt quality**: Check for specificity

#### MCP Servers Analysis
- **Scope correctness**: User vs project placement
- **Lazy loading**: Check if deferred tools are configured
- **Redundancy**: Duplicate servers across scopes

### 3. Generate Report

Output a structured Markdown report:

```markdown
# Claude Code Configuration Analysis

## Summary
- Overall assessment (1-2 sentences)
- Total issues found: N (critical: X, warning: Y, info: Z)

## CLAUDE.md
### Current State
- Location: [path]
- Lines: N (target: ~60, max: 200)
- @imports: [list]

### Issues
- [CRITICAL/WARNING/INFO] Description — Recommendation

### Recommendations
- Specific actionable items

## Settings
### Current State
- Scopes in use: [list]
- Permissions: N allows, N denys
- Hooks: N configured

### Issues
...

## Skills
### Inventory
| Name | Description | Source |
|------|-------------|--------|
...

### Issues
...

## Subagents
...

## MCP Servers
...

## Action Items (Priority Order)
1. [HIGH] ...
2. [MEDIUM] ...
3. [LOW] ...
```

### Severity Levels
- **CRITICAL**: Actively causing problems (e.g., bloated CLAUDE.md over 200 lines, security-risky permissions)
- **WARNING**: Suboptimal but functional (e.g., domain knowledge in CLAUDE.md instead of skills, missing useful hooks)
- **INFO**: Minor improvements (e.g., formatting, organization suggestions)
