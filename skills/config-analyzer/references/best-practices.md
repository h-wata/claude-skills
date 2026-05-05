# Claude Code Configuration Best Practices Reference

## Table of Contents
- [CLAUDE.md Best Practices](#claudemd-best-practices)
- [settings.json Best Practices](#settingsjson-best-practices)
- [Hooks Best Practices](#hooks-best-practices)
- [Skills Best Practices](#skills-best-practices)
- [Subagents Best Practices](#subagents-best-practices)
- [MCP Servers Best Practices](#mcp-servers-best-practices)
- [Permissions Best Practices](#permissions-best-practices)

## CLAUDE.md Best Practices

### What to Include
- Bash commands Claude can't guess
- Code style rules that differ from defaults
- Testing instructions and preferred test runners
- Repository etiquette (branch naming, PR conventions)
- Architectural decisions specific to your project
- Developer environment quirks (required env vars)
- Common gotchas or non-obvious behaviors

### What to Exclude
- Anything Claude can figure out by reading code
- Standard language conventions Claude already knows
- Detailed API documentation (link to docs instead)
- Information that changes frequently
- Long explanations or tutorials
- File-by-file descriptions of the codebase
- Self-evident practices like "write clean code"

### Guidelines
- Keep concise: target under 200 lines per file, 60 lines is a strong target
- For each line ask: "Would removing this cause Claude to make mistakes?" If not, cut it
- Bloated CLAUDE.md files cause Claude to ignore actual instructions
- Use emphasis (IMPORTANT, YOU MUST) for critical rules
- Check into git for team sharing
- Use @path/to/import syntax to reference other files
- Treat like code: review when things go wrong, prune regularly

### Placement
- `~/.claude/CLAUDE.md` — global, all sessions
- `./CLAUDE.md` or `.claude/CLAUDE.md` — project root, shared via git
- Parent directories — monorepo support
- Child directories — loaded on demand

### Anti-patterns
- Kitchen sink: too many rules causing Claude to ignore important ones
- Duplicating what code/linters already enforce
- Including information that changes frequently
- Adding domain knowledge that belongs in skills instead

## settings.json Best Practices

### Scope Selection
| Scope | Location | Use for |
|-------|----------|---------|
| User | `~/.claude/settings.json` | Personal preferences across all projects |
| Project | `.claude/settings.json` | Team-shared settings (commit to git) |
| Local | `.claude/settings.local.json` | Personal overrides per project (gitignored) |
| Managed | System-level | Organization-wide policies |

### Precedence (high to low)
1. Managed (can't be overridden)
2. Command line arguments
3. Local
4. Project
5. User

### Key Settings
- `permissions.allow` / `permissions.deny` — allowlist/denylist for tools
- `hooks` — deterministic automation at workflow points
- `env` — environment variables
- `plugins` — installed plugins list

### Anti-patterns
- Overly broad permissions (prefer specific tool allowlists)
- Mixing personal settings in project-scoped files
- Redundant permissions at multiple scopes
- Missing deny rules for dangerous operations

## Hooks Best Practices

### When to Use Hooks (vs CLAUDE.md)
- Actions that must happen every time with zero exceptions
- Deterministic behavior (unlike CLAUDE.md which is advisory)
- Automated formatting, linting, validation after edits
- Blocking writes to protected directories

### Hook Points
- PreToolUse / PostToolUse — before/after tool execution
- Notification — for alerting
- Stop — when Claude finishes

### Anti-patterns
- Using hooks for things that should be CLAUDE.md guidance
- Slow hooks that block workflow
- Hooks without error handling

## Skills Best Practices

### When to Use Skills (vs CLAUDE.md)
- Domain knowledge relevant only sometimes
- Reusable workflows invoked on demand
- Information that would bloat CLAUDE.md if always loaded

### SKILL.md Guidelines
- Keep SKILL.md body under 500 lines
- Use progressive disclosure: metadata → body → references
- `description` field is the primary trigger mechanism
- Use `disable-model-invocation: true` for side-effect workflows

### Anti-patterns
- Putting always-needed info in skills instead of CLAUDE.md
- Oversized SKILL.md without splitting into references
- Vague descriptions that don't help Claude decide when to trigger

## Subagents Best Practices

### When to Use
- Tasks that read many files (protects main context)
- Specialized focus (security review, code analysis)
- Parallel investigation

### Configuration
- Specify appropriate `tools` list (principle of least privilege)
- Choose `model` based on task complexity
- Write clear system prompt with specific focus

### Anti-patterns
- Giving subagents too many tools
- Vague system prompts
- Using subagents for simple tasks that don't need isolation

## MCP Servers Best Practices

### Configuration Scopes
- `~/.claude.json` — user-level MCP servers
- `.mcp.json` — project-level MCP servers

### Guidelines
- Use Tool Search / deferred tools to reduce context usage
- Prefer authenticated CLI tools (gh, aws) when available
- Configure per-project MCP servers in `.mcp.json`

### Anti-patterns
- Loading all MCP tools upfront (use lazy loading)
- Duplicate MCP servers at different scopes
- MCP servers without proper authentication

## Permissions Best Practices

### Guidelines
- Allowlist specific commands rather than broad patterns
- Use `/permissions` to manage frequently-used tools
- Consider sandboxing for untrusted operations
- Review permissions regularly

### Anti-patterns
- Using `--dangerously-skip-permissions` outside sandboxed environments
- Overly permissive glob patterns
- Not separating read-only from write operations
