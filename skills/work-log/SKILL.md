---
name: work-log
description: Log work notes based on the current conversation
allowed-tools: Bash, Read
---

First, read ~/work/claude-skills/config.json to get the `obsidian_daily_path`.

Based on the conversation in this thread, create a work log entry.

## Output
Append to {obsidian_daily_path}/$(date +%Y-%m-%d).md

## Steps
1. Read config.json to get output path
2. Create file if it doesn't exist
3. Add timestamp heading (HH:MM format)
4. Append to end of file

## Content to include
- What was worked on in this thread
- What changes were made
- Key decisions or important points

Write the memo content in Japanese.

If $ARGUMENTS is provided, incorporate it as additional context.

Additional instructions: $ARGUMENTS
