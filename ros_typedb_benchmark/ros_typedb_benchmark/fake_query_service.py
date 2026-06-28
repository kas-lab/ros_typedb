# Copyright 2026 Gustavo Rezende Silva
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Fake Query service used to isolate ROS service-client behavior."""

from __future__ import annotations

import argparse
import sys
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.executors import SingleThreadedExecutor

from ros_typedb_msgs.srv import Query


def build_fake_service_argument_parser() -> argparse.ArgumentParser:
    """Build command-line arguments for the fake Query service."""
    parser = argparse.ArgumentParser(
        prog='ros_typedb_fake_query_service',
        description='Serve the ros_typedb Query service without TypeDB work.',
    )
    parser.add_argument(
        '--service-name',
        default='/ros_typedb_interface/query',
        help='Query service name. Defaults to /ros_typedb_interface/query.',
    )
    parser.add_argument(
        '--response-delay-s',
        type=float,
        default=0.0,
        help='Optional delay before each response. Defaults to 0.',
    )
    parser.add_argument(
        '--executor',
        choices=('single', 'multi'),
        default='single',
        help='Server executor. Defaults to single.',
    )
    parser.add_argument(
        '--executor-threads',
        type=int,
        default=2,
        help='Thread count for --executor multi. Defaults to 2.',
    )
    return parser


def _build_fake_service_executor(args: argparse.Namespace):
    if args.executor == 'multi':
        return MultiThreadedExecutor(num_threads=args.executor_threads)
    return SingleThreadedExecutor()


def run_fake_query_service(args: argparse.Namespace) -> None:
    """Run an immediate-response Query service for ROS client diagnostics."""
    if args.response_delay_s < 0:
        raise ValueError(
            '--response-delay-s must be greater than or equal to zero'
        )
    if args.executor_threads < 1:
        raise ValueError('--executor-threads must be greater than zero')

    node = rclpy.create_node('ros_typedb_fake_query_service')
    executor = _build_fake_service_executor(args)
    executor.add_node(node)

    def handle_query(
        request: Query.Request,
        response: Query.Response,
    ) -> Query.Response:
        if args.response_delay_s > 0:
            time.sleep(args.response_delay_s)
        response.success = True
        response.error_message = ''
        response.results = []
        return response

    service = node.create_service(Query, args.service_name, handle_query)
    node.get_logger().info(
        f'Fake Query service ready on {args.service_name}'
    )
    try:
        executor.spin()
    finally:
        node.destroy_service(service)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()


def fake_query_service_main(argv: list[str] | None = None) -> int:
    """Run the fake Query service CLI."""
    parser = build_fake_service_argument_parser()
    args = parser.parse_args(argv)

    rclpy.init(args=None)
    try:
        run_fake_query_service(args)
        return 0
    except (RuntimeError, ValueError, OSError, KeyboardInterrupt) as exc:
        if isinstance(exc, KeyboardInterrupt):
            return 0
        print(f'ros_typedb fake query service failed: {exc}', file=sys.stderr)
        return 1
    finally:
        rclpy.shutdown()
