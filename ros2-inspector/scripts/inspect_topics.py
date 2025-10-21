#!/usr/bin/env python3
"""
ROS2 Topic Inspector
Comprehensive analysis of ROS2 topics including types, publishers, subscribers, and frequencies.
"""

import argparse
from collections import defaultdict
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
import yaml


class TopicInspector(Node):

    def __init__(self):
        super().__init__('topic_inspector')
        self.subscribers = {}
        self.message_counts = defaultdict(int)
        self.message_times = defaultdict(list)

    def inspect_topics(self, measure_frequency=False, duration=5.0, output_format='markdown', discovery_timeout=5.0):
        """Inspect all topics in the system."""
        # Wait for topic discovery to complete
        self.get_logger().info(f'Waiting {discovery_timeout}s for topic discovery...')
        time.sleep(discovery_timeout)

        topic_list = self.get_topic_names_and_types()
        self.get_logger().info(f'Found {len(topic_list)} topics')
        results = []

        for topic_name, topic_types in topic_list:
            # Get basic info
            pub_count = self.count_publishers(topic_name)
            sub_count = self.count_subscribers(topic_name)

            # Get publisher info with QoS
            publishers_info = self.get_publishers_info_by_topic(topic_name)
            subscribers_info = self.get_subscriptions_info_by_topic(topic_name)

            info = {
                'name': topic_name,
                'types': list(topic_types),
                'publisher_count': pub_count,
                'subscriber_count': sub_count,
                'publishers': [],
                'subscribers': [],
                'qos': None,
                'frequency': None,
            }

            # Extract publisher node names and QoS
            for pub_info in publishers_info:
                info['publishers'].append({'node': pub_info.node_name, 'namespace': pub_info.node_namespace})
                if pub_info.qos_profile and not info['qos']:
                    info['qos'] = {
                        'reliability': self._qos_reliability_to_string(pub_info.qos_profile.reliability),
                        'durability': self._qos_durability_to_string(pub_info.qos_profile.durability),
                        'depth': pub_info.qos_profile.depth,
                    }

            # Extract subscriber node names
            for sub_info in subscribers_info:
                info['subscribers'].append({'node': sub_info.node_name, 'namespace': sub_info.node_namespace})

            results.append(info)

        # Measure frequencies if requested
        if measure_frequency and results:
            self.get_logger().info(f'Measuring topic frequencies for {duration} seconds...')
            self._measure_frequencies(results, duration)

        return results

    def _measure_frequencies(self, topics, duration):
        """Measure publication frequencies for topics."""
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message

        # Create subscribers for each topic
        for topic_info in topics:
            topic_name = topic_info['name']
            if topic_info['types']:
                try:
                    msg_type_str = topic_info['types'][0]
                    # Parse message type (e.g., 'std_msgs/msg/String')
                    parts = msg_type_str.split('/')
                    if len(parts) == 3:
                        package, _, msg_name = parts
                        msg_type = get_message(f'{package}/{msg_name}')

                        def create_callback(tn):

                            def callback(msg):
                                self.message_counts[tn] += 1
                                self.message_times[tn].append(time.time())

                            return callback

                        sub = self.create_subscription(msg_type, topic_name, create_callback(topic_name), 10)
                        self.subscribers[topic_name] = sub
                except Exception as e:
                    self.get_logger().warn(f'Could not subscribe to {topic_name}: {e}')

        # Spin for duration
        start_time = time.time()
        while time.time() - start_time < duration:
            rclpy.spin_once(self, timeout_sec=0.1)

        # Calculate frequencies
        for topic_info in topics:
            topic_name = topic_info['name']
            if topic_name in self.message_counts and self.message_counts[topic_name] > 0:
                count = self.message_counts[topic_name]
                elapsed = self.message_times[topic_name][-1] - self.message_times[topic_name][0]
                if elapsed > 0:
                    topic_info['frequency'] = count / elapsed

    def _qos_reliability_to_string(self, reliability):
        """Convert QoS reliability to string."""
        if reliability == QoSReliabilityPolicy.RELIABLE:
            return 'reliable'
        elif reliability == QoSReliabilityPolicy.BEST_EFFORT:
            return 'best_effort'
        return 'unknown'

    def _qos_durability_to_string(self, durability):
        """Convert QoS durability to string."""
        if durability == QoSDurabilityPolicy.TRANSIENT_LOCAL:
            return 'transient_local'
        elif durability == QoSDurabilityPolicy.VOLATILE:
            return 'volatile'
        return 'unknown'


def format_markdown(topics, domain_id):
    """Format results as Markdown."""
    from datetime import datetime

    output = []
    output.append('# ROS2 Topic Inspection Report\n')
    output.append(f'**Date**: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
    output.append(f'**ROS_DOMAIN_ID**: {domain_id}\n')
    output.append(f'**Total Topics**: {len(topics)}\n')
    output.append('\n## Topics\n')

    for topic in topics:
        output.append(f'\n### `{topic["name"]}`\n')
        output.append(f'- **Type**: `{", ".join(topic["types"])}`\n')
        output.append(f'- **Publishers**: {topic["publisher_count"]}\n')
        for pub in topic['publishers']:
            output.append(f'  - `{pub["namespace"]}/{pub["node"]}`\n')
        output.append(f'- **Subscribers**: {topic["subscriber_count"]}\n')
        for sub in topic['subscribers']:
            output.append(f'  - `{sub["namespace"]}/{sub["node"]}`\n')

        if topic['qos']:
            qos = topic['qos']
            output.append(
                f'- **QoS**: {qos["reliability"].capitalize()}, {qos["durability"].capitalize()}, Depth: {qos["depth"]}\n'
            )

        if topic['frequency'] is not None:
            output.append(f'- **Frequency**: {topic["frequency"]:.2f} Hz\n')

    return ''.join(output)


def format_yaml(topics, domain_id):
    """Format results as YAML."""
    from datetime import datetime

    data = {'ros2_topic_snapshot': {'timestamp': datetime.now().isoformat(), 'domain_id': domain_id, 'topics': []}}

    for topic in topics:
        topic_data = {
            'name': topic['name'],
            'types': topic['types'],
            'publishers': [f'{p["namespace"]}/{p["node"]}' for p in topic['publishers']],
            'subscribers': [f'{s["namespace"]}/{s["node"]}' for s in topic['subscribers']],
        }

        if topic['qos']:
            topic_data['qos'] = topic['qos']

        if topic['frequency'] is not None:
            topic_data['frequency'] = round(topic['frequency'], 2)

        data['ros2_topic_snapshot']['topics'].append(topic_data)

    return yaml.dump(data, default_flow_style=False, sort_keys=False)


def main(args=None):
    parser = argparse.ArgumentParser(description='Inspect ROS2 topics')
    parser.add_argument('--measure-freq', action='store_true', help='Measure topic frequencies')
    parser.add_argument('--duration', type=float, default=5.0, help='Duration for frequency measurement (seconds)')
    parser.add_argument('--discovery-timeout',
                        type=float,
                        default=5.0,
                        help='Time to wait for topic discovery (seconds)')
    parser.add_argument('--format', choices=['markdown', 'yaml', 'console'], default='markdown', help='Output format')
    parser.add_argument('--output', type=str, help='Output file path')

    parsed_args = parser.parse_args(args)

    rclpy.init()
    inspector = TopicInspector()

    try:
        topics = inspector.inspect_topics(
            measure_frequency=parsed_args.measure_freq,
            duration=parsed_args.duration,
            output_format=parsed_args.format,
            discovery_timeout=parsed_args.discovery_timeout,
        )

        domain_id = os.environ.get('ROS_DOMAIN_ID', '0')

        # Format output
        if parsed_args.format == 'markdown':
            output_text = format_markdown(topics, domain_id)
        elif parsed_args.format == 'yaml':
            output_text = format_yaml(topics, domain_id)
        else:  # console
            output_text = format_markdown(topics, domain_id)

        # Write or print output
        if parsed_args.output:
            with open(parsed_args.output, 'w') as f:
                f.write(output_text)
            inspector.get_logger().info(f'Output written to {parsed_args.output}')
        else:
            print(output_text)

    finally:
        inspector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
