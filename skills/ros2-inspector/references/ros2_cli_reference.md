# ROS2 CLI Command Reference

Quick reference for common ROS2 CLI commands used in system inspection.

## Topic Commands

### List Topics
```bash
ros2 topic list
```
Lists all active topics in the system.

Options:
- `-t, --show-types`: Show topic types alongside names
- `-v, --verbose`: Show detailed information

### Topic Info
```bash
ros2 topic info /topic_name
```
Shows information about a specific topic.

Options:
- `-v, --verbose`: Show detailed QoS information

### Topic Echo
```bash
ros2 topic echo /topic_name
```
Prints messages being published to a topic.

Options:
- `-n, --no-arr`: Don't print array fields
- `--truncate-length LENGTH`: Truncate arrays

### Topic Frequency
```bash
ros2 topic hz /topic_name
```
Measures the publication rate (Hz) of a topic.

Options:
- `--window WINDOW`: Number of messages for rolling average (default: 10000)

### Topic Bandwidth
```bash
ros2 topic bw /topic_name
```
Displays bandwidth usage of a topic.

### Topic Publish
```bash
ros2 topic pub /topic_name <msg_type> "{data: value}"
```
Publishes messages to a topic.

Options:
- `-r, --rate`: Publish rate in Hz
- `-1, --once`: Publish once and exit

## Node Commands

### List Nodes
```bash
ros2 node list
```
Lists all active nodes.

### Node Info
```bash
ros2 node info /node_name
```
Shows detailed information about a node including its subscribers, publishers, services, and actions.

## Service Commands

### List Services
```bash
ros2 service list
```
Lists all active services.

Options:
- `-t, --show-types`: Show service types

### Service Type
```bash
ros2 service type /service_name
```
Prints the service type.

### Service Call
```bash
ros2 service call /service_name <srv_type> "{request}"
```
Calls a service with a request.

## Parameter Commands

### List Parameters
```bash
ros2 param list
```
Lists parameters of all nodes.

### Get Parameter
```bash
ros2 param get /node_name parameter_name
```
Gets the value of a parameter.

### Set Parameter
```bash
ros2 param set /node_name parameter_name value
```
Sets the value of a parameter.

### Dump Parameters
```bash
ros2 param dump /node_name
```
Dumps all parameters of a node to YAML format.

Options:
- `--output-dir DIR`: Output directory for parameter file

### Load Parameters
```bash
ros2 param load /node_name param_file.yaml
```
Loads parameters from a YAML file.

## Interface Commands

### Show Interface
```bash
ros2 interface show <msg_type>
```
Displays the structure of a message/service/action type.

### List Interfaces
```bash
ros2 interface list
```
Lists all available interfaces.

Options:
- `-m, --only-msgs`: Only list message types
- `-s, --only-srvs`: Only list service types
- `-a, --only-actions`: Only list action types

## Bag Commands

### Record Bag
```bash
ros2 bag record <topic1> <topic2> ...
```
Records topics to a bag file.

Options:
- `-a, --all`: Record all topics
- `-o, --output`: Output bag name
- `--compression-mode`: Compression (none/file/message)

### Play Bag
```bash
ros2 bag play <bag_file>
```
Plays back a bag file.

Options:
- `-r, --rate`: Playback rate multiplier
- `-l, --loop`: Loop playback
- `--start-offset`: Start playback at offset (seconds)

### Bag Info
```bash
ros2 bag info <bag_file>
```
Displays information about a bag file.

## Doctor Commands

### System Check
```bash
ros2 doctor
```
Checks the ROS2 setup and identifies issues.

Options:
- `--report`: Generate a full report

## Daemon Commands

### Daemon Status
```bash
ros2 daemon status
```
Checks the daemon status.

### Daemon Stop
```bash
ros2 daemon stop
```
Stops the daemon.

### Daemon Start
```bash
ros2 daemon start
```
Starts the daemon.

## Common Patterns

### Inspect Full System
```bash
# Get all topics with types
ros2 topic list -t

# Get all nodes
ros2 node list

# For each topic, get detailed info
for topic in $(ros2 topic list); do
    ros2 topic info -v $topic
done

# For each node, get detailed info
for node in $(ros2 node list); do
    ros2 node info $node
done
```

### Debug Communication
```bash
# Check if topic exists
ros2 topic list | grep /my_topic

# Check topic type
ros2 topic info -v /my_topic

# Check message structure
ros2 interface show <topic_type>

# Monitor messages
ros2 topic echo /my_topic

# Check frequency
ros2 topic hz /my_topic
```

### Save System Configuration
```bash
# Export all parameters
for node in $(ros2 node list); do
    ros2 param dump $node --output-dir ./params/
done

# Record all topics for analysis
ros2 bag record -a -o system_snapshot
```
