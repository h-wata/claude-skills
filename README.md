# Claude Skills Collection

A collection of custom skills for [Anthropic's Claude Code](https://claude.com/claude-code) that extend Claude's capabilities with specialized workflows.

## Overview

This repository contains production-ready skills built using the [Anthropic Skills framework](https://github.com/anthropics/skills). Each skill is a self-contained package that provides Claude with domain-specific knowledge and workflows through progressive disclosure.

## Available Skills

### obsidian-work-logger

Automatically creates structured work logs in Obsidian-compatible markdown format after conversation sessions.

**Use cases:**
- Documenting development sessions
- Creating project work logs
- Capturing conversation summaries with file changes and resources

**Key features:**
- Auto-generates descriptive titles based on conversation content
- Captures files created/edited during the session
- Records referenced resources and URLs
- Creates Obsidian-compatible frontmatter with tags
- Suggests next steps and follow-up tasks

### ros2-inspector

Generates Python scripts using `rclpy` to systematically inspect ROS2 systems.

**Use cases:**
- Investigating ROS2 topics, nodes, and parameters
- Debugging communication issues (QoS compatibility, message frequencies)
- Creating system documentation and snapshots
- Exporting configuration for version control

**Key features:**
- Comprehensive topic analysis (publishers, subscribers, QoS, frequencies)
- Node information gathering (connections, services, parameters)
- Parameter extraction with YAML export
- Full system inspection reports in Markdown format
- Handles large ROS2 systems with proper discovery timeouts

## Prerequisites

- [Claude Code CLI](https://claude.com/claude-code) installed
- For ros2-inspector: ROS2 environment (Humble, Iron, or Jazzy recommended)

## Installation

### Method 1: Direct Installation (Recommended)

1. Clone this repository:
```bash
git clone https://github.com/h-wata/claude-skills.git
cd claude-skills
```

2. Install skills using the skills repository tools:
```bash
git clone https://github.com/anthropics/skills.git
cd skills

# Install obsidian-work-logger
python scripts/package_skill.py ../claude-skills/obsidian-work-logger

# Install ros2-inspector
python scripts/package_skill.py ../claude-skills/ros2-inspector
```

### Method 2: Manual ZIP Installation

1. Download the desired skill directory
2. Create a ZIP file preserving the directory structure
3. Use Claude Code's skill installation interface to upload the ZIP

### Method 3: Development Mode

For active development, you can link the skill directories directly to your Claude Code skills directory (location varies by platform).

## Usage

### Using obsidian-work-logger

After completing a conversation session, simply ask:

```
"Create a work log for this session"
"Log what we did today to Obsidian"
"Save this session to my notes"
```

Claude will automatically:
1. Gather information about the session
2. Generate a descriptive title
3. Create a structured markdown document
4. Ask for your Obsidian directory location
5. Write the work log file

### Using ros2-inspector

To inspect a running ROS2 system:

```
"Inspect ROS2 topics in this system"
"Show me what nodes are running with ROS_DOMAIN_ID=13"
"Analyze topic frequencies and QoS settings"
"Export all parameters to YAML"
```

Claude will:
1. Determine the inspection scope
2. Select and customize the appropriate inspection script
3. Execute the script with proper discovery timeouts
4. Format results in Markdown or YAML
5. Optionally create system diagrams

## Development

### Skill Structure

Each skill follows the Anthropic Skills specification:

```
skill-name/
├── SKILL.md              # Skill instructions with YAML frontmatter
├── scripts/              # Executable scripts (optional)
├── references/           # Documentation resources (optional)
└── assets/              # Templates and output files (optional)
```

### Creating New Skills

1. Use the initialization script from the [skills repository](https://github.com/anthropics/skills):
```bash
python scripts/init_skill.py
```

2. Edit the generated `SKILL.md` with your instructions

3. Add supporting files in `scripts/`, `references/`, or `assets/` directories

4. Validate and package:
```bash
python scripts/package_skill.py path/to/your-skill
```

### Contributing

Contributions are welcome! Please:

1. Fork this repository
2. Create a feature branch
3. Test your skill thoroughly
4. Submit a pull request with:
   - Clear description of the skill's purpose
   - Usage examples
   - Any prerequisites or dependencies

## References

- [Anthropic Skills Repository](https://github.com/anthropics/skills)
- [What are skills? - Claude Support](https://support.claude.com/en/articles/12512176-what-are-skills)
- [Equipping agents for the real world with Agent Skills](https://anthropic.com/engineering/equipping-agents-for-the-real-world-with-agent-skills)
- [ROS2 Documentation](https://docs.ros.org/en/rolling/)
- [Obsidian Documentation](https://help.obsidian.md/)

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

Skills are provided as-is for educational and development purposes. Please ensure you have appropriate permissions when using skills that interact with external systems or services.

## Acknowledgments

- Built with the [Anthropic Skills framework](https://github.com/anthropics/skills)
- ROS2 inspector based on the `rclpy` client library
- Work logger designed for [Obsidian](https://obsidian.md/) markdown compatibility
