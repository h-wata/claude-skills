#!/usr/bin/env python3
"""
ROS2 Node Inspector
Gather information about running ROS2 nodes including their topics, services, and parameters.
"""

import argparse
import os
import time

import rclpy
from rclpy.node import Node
import yaml


class NodeInspector(Node):

    def __init__(self):
        super().__init__('node_inspector')

    def inspect_nodes(self, discovery_timeout=5.0):
        """Inspect all nodes in the system."""
        # Wait for node discovery to complete
        self.get_logger().info(f'Waiting {discovery_timeout}s for node discovery...')
        time.sleep(discovery_timeout)

        node_names_namespaces = self.get_node_names_and_namespaces()
        self.get_logger().info(f'Found {len(node_names_namespaces)} nodes')
        results = []

        for node_name, namespace in node_names_namespaces:
            full_node_name = f'{namespace}{node_name}' if namespace != '/' else f'/{node_name}'

            # Get topics this node publishes and subscribes to
            pubs = self.get_publisher_names_and_types_by_node(node_name, namespace)
            subs = self.get_subscriber_names_and_types_by_node(node_name, namespace)
            services = self.get_service_names_and_types_by_node(node_name, namespace)

            info = {
                'name': node_name,
                'namespace': namespace,
                'full_name': full_node_name,
                'publications': [topic_name for topic_name, _ in pubs],
                'subscriptions': [topic_name for topic_name, _ in subs],
                'services': [service_name for service_name, _ in services],
            }

            results.append(info)

        return results


def format_markdown(nodes, domain_id):
    """Format results as Markdown."""
    from datetime import datetime

    output = []
    output.append('# ROS2 Node Inspection Report\n')
    output.append(f'**Date**: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
    output.append(f'**ROS_DOMAIN_ID**: {domain_id}\n')
    output.append(f'**Total Nodes**: {len(nodes)}\n')
    output.append('\n## Nodes\n')

    for node in nodes:
        output.append(f'\n### `{node["full_name"]}`\n')
        output.append(f'- **Namespace**: `{node["namespace"]}`\n')

        if node['publications']:
            output.append(f'- **Publications** ({len(node["publications"])}):\n')
            for topic in node['publications']:
                output.append(f'  - `{topic}`\n')

        if node['subscriptions']:
            output.append(f'- **Subscriptions** ({len(node["subscriptions"])}):\n')
            for topic in node['subscriptions']:
                output.append(f'  - `{topic}`\n')

        if node['services']:
            output.append(f'- **Services** ({len(node["services"])}):\n')
            for service in node['services']:
                output.append(f'  - `{service}`\n')

    return ''.join(output)


def format_yaml(nodes, domain_id):
    """Format results as YAML."""
    from datetime import datetime

    data = {'ros2_node_snapshot': {'timestamp': datetime.now().isoformat(), 'domain_id': domain_id, 'nodes': []}}

    for node in nodes:
        node_data = {
            'name': node['name'],
            'namespace': node['namespace'],
            'publications': node['publications'],
            'subscriptions': node['subscriptions'],
            'services': node['services'],
        }
        data['ros2_node_snapshot']['nodes'].append(node_data)

    return yaml.dump(data, default_flow_style=False, sort_keys=False)


def main(args=None):
    parser = argparse.ArgumentParser(description='Inspect ROS2 nodes')
    parser.add_argument('--discovery-timeout',
                        type=float,
                        default=5.0,
                        help='Time to wait for node discovery (seconds)')
    parser.add_argument('--format', choices=['markdown', 'yaml', 'console'], default='markdown', help='Output format')
    parser.add_argument('--output', type=str, help='Output file path')

    parsed_args = parser.parse_args(args)

    rclpy.init()
    inspector = NodeInspector()

    try:
        nodes = inspector.inspect_nodes(discovery_timeout=parsed_args.discovery_timeout)
        domain_id = os.environ.get('ROS_DOMAIN_ID', '0')

        # Format output
        if parsed_args.format == 'markdown':
            output_text = format_markdown(nodes, domain_id)
        elif parsed_args.format == 'yaml':
            output_text = format_yaml(nodes, domain_id)
        else:  # console
            output_text = format_markdown(nodes, domain_id)

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
