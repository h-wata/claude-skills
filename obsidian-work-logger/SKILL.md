---
name: obsidian-work-logger
description: This skill helps create structured work logs in Obsidian format after a conversation session. Claude should use this skill when the user asks to summarize or log the current session's work, create a work log, or save conversation results to Obsidian. The skill generates work logs including conversation summary, files created/edited, and referenced resources.
---

# Obsidian Work Logger

## Overview

This skill enables automatic creation of structured work logs in Obsidian-compatible markdown format. It captures the essence of a conversation session including what was accomplished, which files were modified, and what resources were referenced.

## When to Use This Skill

Use this skill when the user requests to:
- "Create a work log for this session"
- "Summarize our conversation to Obsidian"
- "Log what we did today"
- "Save this session to my notes"
- "Document the work we just completed"

## How to Use This Skill

### Step 1: Gather Session Information

Collect the following information from the current conversation:

1. **Conversation Summary**
   - Main objective or purpose of the session
   - Key decisions made
   - Problems solved
   - Outcomes achieved

2. **Files Created or Edited**
   - File paths of any files written or modified
   - Brief description of changes made to each file
   - Tools used (Write, Edit, NotebookEdit, etc.)

3. **Referenced Resources**
   - URLs fetched with WebFetch
   - Files read with Read tool
   - External documentation or repositories referenced
   - Any web searches performed

4. **Important Commands Executed**
   - Key bash commands run (if relevant to the work)
   - Installation commands
   - Build or test commands

### Step 2: Generate Document Title

Automatically generate a concise, descriptive title based on the conversation's main topic. The title should:
- Be 3-7 words long
- Capture the essence of what was accomplished
- Use title case
- Be suitable as a filename (no special characters except hyphens and underscores)

Examples:
- "Anthropic Skills Investigation and Documentation"
- "Custom Obsidian Logger Skill Development"
- "API Integration and Testing Setup"

### Step 3: Create the Work Log Document

Use the template from `assets/work-log-template.md` to structure the work log. The document should include:

1. **Frontmatter** with appropriate tags
2. **Date and Time** of the session
3. **Summary** section with conversation overview
4. **Work Completed** section with specific accomplishments
5. **Files Modified** section listing all changes
6. **Resources Referenced** section with links
7. **Next Steps** (if applicable) for follow-up work

### Step 4: Determine Output Location

Ask the user for the Obsidian directory path if not already known. Common patterns:
- `~/Documents/Obsidian/Work Logs/`
- `~/Documents/my_obsidian/Tec_insight/`
- User's specified directory

Default filename format: `{generated-title}.md`

### Step 5: Write the Document

Use the Write tool to create the work log file at the specified location with the generated content.

## Work Log Template Structure

```markdown
---
tags:
  - WorkLog
  - [relevant-topic-tags]
date: YYYY-MM-DD
session_time: HH:MM
---

# [Generated Title]

## Session Overview

**Date**: YYYY-MM-DD
**Duration**: [Approximate duration if known]

## Summary

[2-4 sentence summary of the conversation and its purpose]

## Work Completed

- [Bullet point of accomplishment 1]
- [Bullet point of accomplishment 2]
- [Bullet point of accomplishment 3]

## Files Modified

### Created
- `path/to/file1.ext` - Description of what was created
- `path/to/file2.ext` - Description of what was created

### Edited
- `path/to/file3.ext` - Description of changes made
- `path/to/file4.ext` - Description of changes made

## Resources Referenced

### Documentation
- [Title](URL) - Brief description
- [Title](URL) - Brief description

### Code/Repositories
- [Repository Name](URL) - Brief description

### Local Files
- `path/to/local/file.ext` - Why it was referenced

## Key Commands

```bash
command1
command2
```

## Key Insights

- [Important learning or decision from the session]
- [Another key insight]

## Next Steps

- [ ] Follow-up task 1
- [ ] Follow-up task 2

## Related Notes

- [[Related Note 1]]
- [[Related Note 2]]
```

## Best Practices

1. **Be Concise but Complete**: Capture essential information without overwhelming detail
2. **Use Relative Paths**: When possible, use relative paths for better portability
3. **Link Resources**: Always include full URLs for web resources
4. **Tag Appropriately**: Add relevant topic tags to improve discoverability in Obsidian
5. **Maintain Consistency**: Use the same structure for all work logs
6. **Include Context**: Add enough context so the log makes sense months later

## Resources

This skill includes:

### assets/
- `work-log-template.md` - The base template for creating work logs
