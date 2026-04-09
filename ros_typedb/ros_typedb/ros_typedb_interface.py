# Copyright 2026 KAS Lab
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
"""ros_typedb_interface - ROS 2 lifecycle node for TypeDB interaction."""

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.lifecycle import Node
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn

from ros_typedb.ros_typedb_helpers import query_result_to_ros_msg

from ros_typedb_msgs.srv import Query

from std_msgs.msg import String

from std_srvs.srv import Empty
from typedb_utils.typedb_interface import TypeDBInterface
from typedb_utils.typedb_interface import TypeDBQueryError

_READ_QUERY_TYPES = {
    Query.Request.FETCH,
    Query.Request.GET,
    Query.Request.GET_AGGREGATE,
}


class ROSTypeDBInterface(Node):
    """ROS lifecycle node to interact with typedb."""

    def __init__(self, node_name: str, **kwargs):
        """
        Initialize the lifecycle node and declare runtime parameters.

        :param node_name: ROS node name.
        :param kwargs: Extra keyword arguments passed to ``Node``.
        """
        super().__init__(node_name, **kwargs)
        self.declare_parameter('address', 'localhost:1729')
        self.declare_parameter('database_name', 'ros_typedb')
        self.declare_parameter('force_database', True)
        self.declare_parameter('force_data', True)
        self.declare_parameter('schema_path', [''])
        self.declare_parameter('data_path', [''])

        self.declare_parameter('sort_fetch_results', False)

        self.typedb_interface_class = TypeDBInterface

        self.query_cb_group = MutuallyExclusiveCallbackGroup()

    def init_typedb_interface(
            self,
            address: str,
            database_name: str,
            schema_path: list[str] | str | None = None,
            data_path: list[str] | str | None = None,
            force_database: bool = False,
            force_data: bool = False) -> None:
        """
        Initialize self.typedb_interface.

        :param address: TypeDB server address.
        :param database_name: database name.
        :param schema_path: list with paths to schema files (.tql).
        :param data_path: list with paths to data files (.tql).
        :param force_database: if database should override an existing database
        :param force_data: if the database data should be overridden.
        :return: None.
        """
        self.typedb_interface = self.typedb_interface_class(
            address,
            database_name,
            schema_path,
            data_path,
            force_database,
            force_data,
        )

        self.typedb_interface.insert_data_event = self.insert_data_event
        self.typedb_interface.delete_data_event = self.delete_data_event

    def publish_data_event(self, event_type: str) -> None:
        """
        Publish message in the `/event` topic.

        :param event_type: event to be published, e.g., 'insert' or 'delete'.
        :return: None.
        """
        self.event_pub.publish(String(data=event_type))

    def insert_data_event(self) -> None:
        """
        Publish an insert event in the ``/event`` topic.

        :return: None.
        """
        self.publish_data_event('insert')

    def delete_data_event(self) -> None:
        """
        Publish a delete event in the ``/event`` topic.

        :return: None.
        """
        self.publish_data_event('delete')

    def on_configure(self, _state: State) -> TransitionCallbackReturn:
        """
        Configure ROSTypeDBInterface when the configure transition is called.

        :param _state: Lifecycle transition state (unused).
        :return: Transition callback status.
        """
        self.get_logger().info(self.get_name() + ': on_configure() is called.')

        self.init_typedb_interface(
            address=self.get_parameter('address').value,
            database_name=self.get_parameter('database_name').value,
            schema_path=self.get_parameter('schema_path').value,
            data_path=self.get_parameter('data_path').value,
            force_database=self.get_parameter('force_database').value,
            force_data=self.get_parameter('force_data').value,
        )

        self.event_pub = self.create_lifecycle_publisher(
            String,
            self.get_name() + '/events',
            10,
            callback_group=ReentrantCallbackGroup())

        self.query_service = self.create_service(
            Query,
            self.get_name() + '/query',
            self.query_service_cb,
            callback_group=self.query_cb_group)

        self.delete_db_service = self.create_service(
            Empty,
            self.get_name() + '/delete_database',
            self.delete_db_cb,
            callback_group=self.query_cb_group)

        self.get_logger().info(self.get_name() + ':on_configure() completed.')
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, _state: State) -> TransitionCallbackReturn:
        """
        Cleanup ROSTypeDBInterface when the cleanup transition is called.

        :param _state: Lifecycle transition state (unused).
        :return: Transition callback status.
        """
        self.destroy_publisher(self.event_pub)
        self.destroy_service(self.query_service)
        self.destroy_service(self.delete_db_service)

        self.get_logger().info(self.get_name() + ' :on_cleanup() is called.')
        return TransitionCallbackReturn.SUCCESS

    def query_service_cb(
        self,
        req: Query.Request,
        response: Query.Response
    ) -> Query.Response:
        """
        Handle callback for ~/query service.

        Perform the query requested with the ~/query service.

        :param req: Incoming service request with query type and TypeQL text.
        :param response: Pre-allocated ROS service response.
        :return: Filled response with success flag and optional payload.
        """
        query_handlers = {
            Query.Request.INSERT: self.typedb_interface.insert_database,
            Query.Request.DELETE: self.typedb_interface.delete_from_database,
            Query.Request.FETCH: self.typedb_interface.fetch_database,
            Query.Request.GET: self.typedb_interface.get_database,
            Query.Request.GET_AGGREGATE: self.typedb_interface.get_aggregate_database,
            Query.Request.UPDATE: self.typedb_interface.update_database,
        }
        query_func = query_handlers.get(req.query_type)
        if query_func is None:
            self.get_logger().warning(
                'Query type {} not recognized'.format(req.query_type))
            response.success = False
            return response

        try:
            query_result = query_func(req.query)
            response = query_result_to_ros_msg(req.query_type, query_result)
        except TypeDBQueryError as err:
            self.get_logger().error(
                f'TypeDB query failed for query_type {req.query_type}: {err}')
            response.success = False
            return response
        except Exception as err:
            self.get_logger().error(
                f'Unexpected error in query callback for query_type '
                f'{req.query_type}: {err}')
            response.success = False
            return response
        if req.query_type in _READ_QUERY_TYPES:
            return response
        response.success = bool(query_result)
        return response

    def delete_db_cb(
        self,
        _req: Empty.Request,
        response: Empty.Response
    ) -> Empty.Response:
        """
        Handle callback for ~/delete_database service.

        :param _req: Empty request payload (unused).
        :param response: Empty service response.
        :return: Response after deleting the configured database.
        """
        self.typedb_interface.delete_database()
        return response
