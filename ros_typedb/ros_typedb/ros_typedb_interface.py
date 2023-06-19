# Copyright 2023 Gustavo Rezende Silva
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

import rclpy

from rclpy.lifecycle import Node
from rclpy.lifecycle import Publisher
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.typedb_interface import TypeDBInterface
from ros_typedb_msgs.srv import Query

from std_msgs.msg import String


class ROSTypeDBInterface(Node):
    def __init__(self, node_name, **kwargs):
        super().__init__(node_name, **kwargs)

    def init_typedb_interface(
            self,
            address,
            database_name,
            schema_path=None,
            data_path=None,
            force_database=False,
            force_data=False):

        self.typedb_interface = TypeDBInterface(
            address,
            database_name,
            schema_path,
            data_path,
            force_database,
            force_data
        )

        self.typedb_interface.insert_data_event = self.insert_data_event
        self.typedb_interface.delete_data_event = self.delete_data_event

    def data_event(self, event_type):
        event = String()
        event.data = event_type
        self.event_pub.publish(event)

    def insert_data_event(self):
        self.data_event('insert')

    def delete_data_event(self):
        self.data_event('delete')

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.get_logger().info("on_configure() is called.")

        self.event_pub = self.create_lifecycle_publisher(
            String, 'typedb/events', 10)

        self.insert_query_service = self.create_service(
            Query,
            'typedb/insert_query',
            self.insert_query_service_cb)

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.destroy_publisher(self.event_pub)
        self.destroy_service(self.insert_query_service)

        self.get_logger().info('on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def insert_query_service_cb(self, req, response):
        self.get_logger().info('Insert query requested: {}'.format(req.query))
        result = self.typedb_interface.insert_database(req.query)
        if result is None:
            response.success = False
        else:
            response.success = True
        return response


def main():
    rclpy.init()

    executor = rclpy.executors.MultiThreadedExecutor()
    lc_node = ROSTypeDBInterface('ros_typedb_interface')
    lc_node.init_typedb_interface(
        "localhost:1729",
        "test_database",
        force_database=True,
        schema_path='/home/gus/exp_metacontrol_ws/src/ros_typedb/ros_typedb/test/typedb_test_data/schema.tql',
        data_path='/home/gus/exp_metacontrol_ws/src/ros_typedb/ros_typedb/test/typedb_test_data/data.tql',
        force_data=True)
    executor.add_node(lc_node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        lc_node.destroy_node()


if __name__ == '__main__':
    main()
