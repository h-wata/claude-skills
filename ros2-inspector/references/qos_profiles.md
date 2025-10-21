# ROS2 QoS (Quality of Service) Profiles

## Overview

QoS (Quality of Service) policies in ROS2 determine how data is transmitted between publishers and subscribers. Mismatched QoS settings between publishers and subscribers can prevent communication.

## Key QoS Policies

### Reliability

Determines how data delivery is guaranteed.

**Reliable**
- Guarantees message delivery
- Will retransmit lost messages
- Higher overhead
- Use for: Critical data that must not be lost

**Best Effort**
- No delivery guarantee
- Won't retransmit lost messages
- Lower overhead
- Use for: Sensor data where latest value matters most

### Durability

Determines whether late-joining subscribers receive historical data.

**Volatile**
- No historical data
- Subscribers only receive messages published after subscription
- Use for: Real-time data streams

**Transient Local**
- Keeps last N messages (based on history depth)
- Late-joining subscribers receive historical messages
- Use for: Configuration data, state information

### History

Controls how many messages are buffered.

**Keep Last N**
- Keeps the last N messages in queue
- Older messages are discarded
- Use for: Most common case

**Keep All**
- Keeps all messages (within resource limits)
- Use for: Logging, data recording

### Depth

Number of messages to keep in queue (for Keep Last history).

- Default: 10
- Common values: 1 (latest only), 10 (default), 100+ (recording)

### Lifespan

Maximum time a message stays valid.

- Default: Infinite
- Use for: Time-sensitive data that becomes stale

### Deadline

Expected maximum time between messages.

- Default: Infinite
- Use for: Detecting publisher failures

### Liveliness

How system detects that publisher is alive.

**Automatic**
- System automatically checks node liveness
- Default for most cases

**Manual by Topic**
- Publisher must explicitly signal liveness per topic

## Pre-defined QoS Profiles

ROS2 provides several standard QoS profiles:

### System Default
```python
Reliability: Reliable
Durability: Volatile
History: Keep Last (10)
```
General purpose communication.

### Sensor Data
```python
Reliability: Best Effort
Durability: Volatile
History: Keep Last (5)
```
For high-frequency sensor data where latest value matters.

### Parameters
```python
Reliability: Reliable
Durability: Volatile
History: Keep Last (1000)
```
For parameter changes.

### Services
```python
Reliability: Reliable
Durability: Volatile
History: Keep Last (10)
```
For service communication.

### Clock
```python
Reliability: Best Effort
Durability: Volatile
History: Keep Last (1)
```
For /clock topic.

## QoS Compatibility Rules

For communication to work, QoS policies must be compatible:

### Reliability Compatibility
| Publisher | Subscriber | Compatible? |
|-----------|------------|-------------|
| Reliable  | Reliable   | ✓ |
| Reliable  | Best Effort| ✓ |
| Best Effort| Reliable  | ✗ |
| Best Effort| Best Effort| ✓ |

**Rule**: Reliable publisher can communicate with any subscriber, but Best Effort publisher can only communicate with Best Effort subscriber.

### Durability Compatibility
| Publisher | Subscriber | Compatible? |
|-----------|------------|-------------|
| Volatile  | Volatile   | ✓ |
| Volatile  | Transient Local | ✗ |
| Transient Local | Volatile | ✓ |
| Transient Local | Transient Local | ✓ |

**Rule**: Transient Local publisher can communicate with any subscriber, but Volatile publisher can only communicate with Volatile subscriber.

## Common QoS Issues

### Issue 1: No Communication

**Symptoms**: Topics exist but no data flowing

**Check**:
```bash
ros2 topic info -v /topic_name
```

**Solution**: Ensure QoS compatibility between publisher and subscriber

### Issue 2: Late-joining Subscribers Miss Data

**Symptoms**: Subscriber doesn't receive initial configuration

**Solution**: Use Transient Local durability on publisher

### Issue 3: Message Loss on Unreliable Network

**Symptoms**: Periodic data gaps

**Solution**: Use Reliable reliability instead of Best Effort

### Issue 4: High Latency

**Symptoms**: Delays in message delivery

**Possible causes**:
- Reliable mode retransmitting on poor network
- History depth too large
- Message size too large

**Solutions**:
- Use Best Effort for time-sensitive data
- Reduce history depth
- Optimize message size

## Setting QoS in Python (rclpy)

### Using Pre-defined Profiles
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

# Sensor data profile
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=5
)

# Create publisher with QoS
self.publisher = self.create_publisher(
    MsgType,
    '/topic_name',
    qos_profile
)
```

### Custom QoS Profile
```python
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.duration import Duration

custom_qos = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=100,
    deadline=Duration(seconds=1),
    lifespan=Duration(seconds=10)
)
```

## Best Practices

1. **Match Use Case to QoS**
   - Sensor data: Best Effort, Volatile
   - Commands: Reliable, Volatile
   - Configuration: Reliable, Transient Local

2. **Start with Defaults**
   - Use system defaults unless you have specific requirements
   - Profile first, customize later if needed

3. **Document QoS Requirements**
   - Clearly document expected QoS for each topic
   - Include in interface documentation

4. **Test Compatibility**
   - Use `ros2 topic info -v` to verify QoS settings
   - Test with realistic network conditions

5. **Consider Network Conditions**
   - Unreliable network → Reliable might cause delays
   - Low bandwidth → Best Effort might work better
   - Balance reliability vs. latency needs

## Debugging QoS

### Check Topic QoS
```bash
# Verbose topic info shows QoS
ros2 topic info -v /topic_name
```

### Monitor QoS Events
```python
# In rclpy, you can add QoS event callbacks
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.event_handler import SubscriptionEventCallbacks, QoSDeadlineRequestedInfo

def deadline_callback(event):
    print(f'Deadline missed: {event}')

event_callbacks = SubscriptionEventCallbacks(
    deadline=deadline_callback
)

sub = node.create_subscription(
    MsgType,
    '/topic',
    callback,
    qos_profile,
    event_callbacks=event_callbacks
)
```

### Common Debugging Commands
```bash
# Check if publisher and subscriber QoS match
ros2 topic info -v /topic_name

# See actual message flow
ros2 topic hz /topic_name

# Monitor for issues
ros2 topic echo /topic_name --qos-reliability best_effort
```
