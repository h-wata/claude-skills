#!/usr/bin/env python3
"""
ROS2 Parameter Inspector
Extract and export parameters from ROS2 nodes.
"""

import argparse
import os
import time

from rcl_interfaces.srv import GetParameters
from rcl_interfaces.srv import ListParameters
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import yaml


class ParameterInspector(Node):

    def __init__(self):
        super().__init__('parameter_inspector')

    def inspect_parameters(self, node_filter=None, discovery_timeout=5.0):
        """Inspect parameters from all or specific nodes."""
        # Wait for node discovery to complete
        self.get_logger().info(f'Waiting {discovery_timeout}s for node discovery...')
        time.sleep(discovery_timeout)

        node_names_namespaces = self.get_node_names_and_namespaces()
        self.get_logger().info(f'Found {len(node_names_namespaces)} nodes')
        results = {}

        for node_name, namespace in node_names_namespaces:
            full_node_name = f'{namespace}{node_name}' if namespace != '/' else f'/{node_name}'

            # Filter nodes if specified
            if node_filter and node_filter not in full_node_name:
                continue

            try:
                # Create service clients for this node
                list_params_client = self.create_client(ListParameters, f'{full_node_name}/list_parameters')
                get_params_client = self.create_client(GetParameters, f'{full_node_name}/get_parameters')

                # Wait for services
                if not list_params_client.wait_for_service(timeout_sec=1.0):
                    continue

                # List parameters
                list_req = ListParameters.Request()
                list_req.prefixes = []
                list_req.depth = 10
                list_future = list_params_client.call_async(list_req)
                rclpy.spin_until_future_complete(self, list_future, timeout_sec=1.0)

                if list_future.result() is not None and list_future.result().result.names:
                    # Get parameter values
                    get_req = GetParameters.Request()
                    get_req.names = list_future.result().result.names
                    get_future = get_params_client.call_async(get_req)
                    rclpy.spin_until_future_complete(self, get_future, timeout_sec=1.0)

                    if get_future.result() is not None:
                        params = {}
                        for name, value in zip(get_req.names, get_future.result().values):
                            params[name] = self._parameter_value_to_dict(value)

                        results[full_node_name] = params

                # Destroy clients
                self.destroy_client(list_params_client)
                self.destroy_client(get_params_client)

            except Exception as e:
                self.get_logger().warn(f'Could not get parameters from {full_node_name}: {e}')

        return results

    def _parameter_value_to_dict(self, param_value):
        """Convert ROS2 ParameterValue to Python native type."""
        from rcl_interfaces.msg import ParameterType

        if param_value.type == ParameterType.PARAMETER_BOOL:
            return param_value.bool_value
        elif param_value.type == ParameterType.PARAMETER_INTEGER:
            return param_value.integer_value
        elif param_value.type == ParameterType.PARAMETER_DOUBLE:
            return param_value.double_value
        elif param_value.type == ParameterType.PARAMETER_STRING:
            return param_value.string_value
        elif param_value.type == ParameterType.PARAMETER_BYTE_ARRAY:
            return list(param_value.byte_array_value)
        elif param_value.type == ParameterType.PARAMETER_BOOL_ARRAY:
            return list(param_value.bool_array_value)
        elif param_value.type == ParameterType.PARAMETER_INTEGER_ARRAY:
            return list(param_value.integer_array_value)
        elif param_value.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return list(param_value.double_array_value)
        elif param_value.type == ParameterType.PARAMETER_STRING_ARRAY:
            return list(param_value.string_array_value)
        else:
            return None


def format_markdown(params, domain_id):
    """Format results as Markdown."""
    from datetime import datetime

    output = []
    output.append('# ROS2 Parameter Inspection Report\n')
    output.append(f'**Date**: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}\n')
    output.append(f'**ROS_DOMAIN_ID**: {domain_id}\n')
    output.append(f'**Total Nodes**: {len(params)}\n')
    output.append('\n## Parameters\n')

    for node_name, node_params in params.items():
        output.append(f'\n### `{node_name}`\n')
        if node_params:
            output.append('```yaml\n')
            output.append(yaml.dump(node_params, default_flow_style=False, sort_keys=True))
            output.append('```\n')
        else:
            output.append('*No parameters*\n')

    return ''.join(output)


def format_yaml(params, domain_id):
    """Format results as YAML."""
    from datetime import datetime

    data = {
        'ros2_parameter_snapshot': {
            'timestamp': datetime.now().isoformat(),
            'domain_id': domain_id,
            'parameters': params,
        }
    }

    return yaml.dump(data, default_flow_style=False, sort_keys=False)


def main(args=None):
    parser = argparse.ArgumentParser(description='Inspect ROS2 parameters')
    parser.add_argument('--node', type=str, help='Filter by node name (substring match)')
    parser.add_argument('--discovery-timeout',
                        type=float,
                        default=5.0,
                        help='Time to wait for node discovery (seconds)')
    parser.add_argument('--format', choices=['markdown', 'yaml', 'console'], default='markdown', help='Output format')
    parser.add_argument('--output', type=str, help='Output file path')

    parsed_args = parser.parse_args(args)

    rclpy.init()
    inspector = ParameterInspector()

    try:
        params = inspector.inspect_parameters(node_filter=parsed_args.node,
                                              discovery_timeout=parsed_args.discovery_timeout)
        domain_id = os.environ.get('ROS_DOMAIN_ID', '0')

        # Format output
        if parsed_args.format == 'markdown':
            output_text = format_markdown(params, domain_id)
        elif parsed_args.format == 'yaml':
            output_text = format_yaml(params, domain_id)
        else:  # console
            output_text = format_markdown(params, domain_id)

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
