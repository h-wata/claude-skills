---
name: ros2-inspector
description: This skill helps inspect and analyze ROS2 systems by generating Python scripts that automatically gather information about topics, nodes, and parameters. Claude should use this skill when the user asks to investigate ROS2 topics (including specific topics and their publishers/subscribers), nodes, parameters, or wants to understand the structure of a running ROS2 system. The skill creates inspection scripts using rclpy and formats results in Markdown or YAML.
---

# ROS2 Inspector

## Overview

This skill enables automatic generation of ROS2 inspection scripts that gather comprehensive information about running ROS2 systems. Instead of manually running multiple `ros2` CLI commands, this skill creates Python scripts using `rclpy` to systematically collect data about topics, nodes, and parameters.

## When to Use This Skill

Use this skill when the user requests to:
- "Inspect ROS2 topics in this system"
- "Show me what nodes are running"
- "Find which nodes publish/subscribe to a specific topic"
- "Detect publishers and subscribers for [topic_name]"
- "Investigate connections for a specific topic"
- "Analyze the ROS2 system structure"
- "Check topic frequencies and QoS settings"
- "Get all parameters from running nodes"
- "Create a system snapshot/report"

## How to Use This Skill

### Step 1: Determine Inspection Scope

Ask the user what they want to inspect (if not clear from the request):
- **Topics**: Message types, publishers, subscribers, QoS settings, frequencies
- **Nodes**: Running nodes, their subscriptions, publications, services, parameters
- **Parameters**: Parameter values across nodes
- **Full System**: Complete snapshot of all above

### Step 2: Select Appropriate Script Template

Based on the inspection scope, use the corresponding script from `scripts/`:

1. **`scripts/inspect_topics.py`** - Comprehensive topic analysis
   - Lists all topics
   - Shows message types
   - Displays publishers and subscribers
   - Checks QoS settings
   - Measures publication frequencies

2. **`scripts/inspect_nodes.py`** - Node information gathering
   - Lists all nodes
   - Shows node connections (topics, services)
   - Displays node parameters

3. **`scripts/inspect_params.py`** - Parameter extraction
   - Gets all parameters from specified nodes
   - Formats output in YAML for easy saving

4. **`scripts/inspect_system.py`** - Full system inspection
   - Combines all above inspections
   - Generates comprehensive system report

### Step 3: Customize the Script

Modify the script based on user requirements:
- Filter by namespace
- Focus on specific topics/nodes
- Adjust measurement duration for frequency checks
- Change output format (console, Markdown, YAML)

### Step 4: Execute and Format Results

1. Run the inspection script using the Bash tool
2. Capture the output
3. If requested, format results into:
   - **Markdown report**: Structured document with sections for topics, nodes, parameters
   - **YAML export**: Machine-readable format for version control or comparison
   - **Console output**: Direct display of results

### Step 5: Optional - Create System Diagram

If visualization is requested, generate a Mermaid diagram showing:
- Node relationships
- Topic connections
- Publisher-subscriber patterns

## Script Templates

### Topic Inspection Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time

class TopicInspector(Node):
    def __init__(self):
        super().__init__('topic_inspector')

    def inspect_topics(self):
        topic_list = self.get_topic_names_and_types()
        results = []

        for topic_name, topic_types in topic_list:
            info = {
                'name': topic_name,
                'types': topic_types,
                'publishers': self.count_publishers(topic_name),
                'subscribers': self.count_subscribers(topic_name)
            }
            results.append(info)

        return results

def main():
    rclpy.init()
    inspector = TopicInspector()
    topics = inspector.inspect_topics()

    # Format output
    print("# ROS2 Topics\\n")
    for topic in topics:
        print(f"## {topic['name']}")
        print(f"- Type: {', '.join(topic['types'])}")
        print(f"- Publishers: {topic['publishers']}")
        print(f"- Subscribers: {topic['subscribers']}")
        print()

    inspector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Inspection Template

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class NodeInspector(Node):
    def __init__(self):
        super().__init__('node_inspector')

    def inspect_nodes(self):
        node_names = self.get_node_names()
        results = []

        for node_name in node_names:
            info = {
                'name': node_name,
                'namespace': '/',
            }
            results.append(info)

        return results

def main():
    rclpy.init()
    inspector = NodeInspector()
    nodes = inspector.inspect_nodes()

    print("# ROS2 Nodes\\n")
    for node in nodes:
        print(f"## {node['name']}")
        print(f"- Namespace: {node['namespace']}")
        print()

    inspector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Output Formats

### Markdown Report Format

```markdown
# ROS2 System Inspection Report

**Date**: YYYY-MM-DD HH:MM:SS
**ROS_DOMAIN_ID**: [value]

## Topics

### /topic_name
- **Type**: `package_name/msg/MessageType`
- **Publishers**: 2
  - `/node1`
  - `/node2`
- **Subscribers**: 1
  - `/node3`
- **QoS**: Reliable, Volatile, Depth: 10
- **Frequency**: 10.5 Hz

## Nodes

### /node1
- **Namespace**: `/`
- **Publications**: `/topic1`, `/topic2`
- **Subscriptions**: `/topic3`
- **Services**: `/service1`
- **Parameters**: 5 parameters

## Parameters

### /node1
\```yaml
param1: value1
param2: value2
\```
```

### YAML Export Format

```yaml
ros2_system_snapshot:
  timestamp: "2025-10-21T18:00:00"
  domain_id: 0

  topics:
    - name: /topic_name
      type: package_name/msg/MessageType
      publishers:
        - /node1
        - /node2
      subscribers:
        - /node3
      qos:
        reliability: reliable
        durability: volatile
        depth: 10
      frequency: 10.5

  nodes:
    - name: /node1
      namespace: /
      publications: [/topic1, /topic2]
      subscriptions: [/topic3]
      services: [/service1]

  parameters:
    /node1:
      param1: value1
      param2: value2
```

## Common Use Cases

### Use Case 1: Quick Topic Overview
**Request**: "Show me all ROS2 topics"
**Action**: Run `scripts/inspect_topics.py` with basic output

### Use Case 2: Detect Specific Topic Connections
**Request**: "Please detect the node to publish and subscribe 'map_scan_front'"
**Action**:
1. Run `scripts/inspect_topics.py` to get comprehensive topic information
2. Extract publisher and subscriber nodes for the specific topic
3. Report message type, QoS settings, and optionally frequency

### Use Case 3: Debug Communication Issues
**Request**: "Why isn't /my_topic receiving messages?"
**Action**:
1. Run topic inspection to check publishers/subscribers
2. Check QoS compatibility
3. Verify message frequencies

### Use Case 4: System Documentation
**Request**: "Create documentation of the current ROS2 system"
**Action**:
1. Run full system inspection
2. Generate Markdown report
3. Create Mermaid diagram of node connections

### Use Case 5: Configuration Snapshot
**Request**: "Export all parameters for version control"
**Action**: Run parameter inspection with YAML output

## Best Practices

1. **Check ROS_DOMAIN_ID**: Always verify which ROS domain is being inspected
2. **Use Appropriate Timeouts**: For frequency measurements, run for sufficient duration (5-10 seconds)
3. **Handle Large Systems**: For systems with many nodes/topics, consider filtering by namespace
4. **Save Snapshots**: Keep YAML exports for comparing system states over time
5. **Verify Permissions**: Ensure the inspection script has access to all nodes/topics

## ROS2 CLI Command Reference

The skill generates scripts that replace these common manual commands:

| Task | Manual Command | Generated Script Feature |
|------|---------------|-------------------------|
| List topics | `ros2 topic list` | `inspect_topics.py --list` |
| Topic info | `ros2 topic info -v /topic` | Included in topic inspection |
| Topic echo | `ros2 topic echo /topic` | Sample message capture |
| Topic frequency | `ros2 topic hz /topic` | Automatic frequency measurement |
| List nodes | `ros2 node list` | `inspect_nodes.py` |
| Node info | `ros2 node info /node` | Detailed node analysis |
| List parameters | `ros2 param list` | `inspect_params.py` |
| Get parameter | `ros2 param get /node param` | Bulk parameter retrieval |
| Parameter dump | `ros2 param dump /node` | YAML export |

## Resources

This skill includes:

### scripts/
- `inspect_topics.py` - Comprehensive topic analysis script
- `inspect_nodes.py` - Node information gathering script
- `inspect_params.py` - Parameter extraction script
- `inspect_system.py` - Full system inspection script

### references/
- `ros2_cli_reference.md` - Quick reference for ROS2 CLI commands
- `qos_profiles.md` - Common QoS profile settings and compatibility

## Troubleshooting

**Issue**: Script doesn't find any topics/nodes
- Check if ROS2 system is running
- Verify ROS_DOMAIN_ID matches the target system
- Ensure ROS2 is sourced in the environment

**Issue**: Permission denied errors
- Check if user has access to ROS2 endpoints
- Verify DDS security settings if enabled

**Issue**: Frequency measurements seem wrong
- Increase measurement duration
- Check if topic is being published consistently
- Verify system clock synchronization
