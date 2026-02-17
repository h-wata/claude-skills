---
name: memo
description: Quick memo to Obsidian Daily note
allowed-tools: Bash, Read
---

First, read ~/work/claude-skills/config.json to get the `obsidian_daily_path`.

Then append the following memo to {obsidian_daily_path}/$(date +%Y-%m-%d).md

## Steps
1. Read config.json to get output path
2. Create file if it doesn't exist
3. Add timestamp heading (HH:MM format)
4. Append to end of file

## Memo content
$ARGUMENTS
