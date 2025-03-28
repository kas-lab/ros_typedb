import sys

from ros_typedb_msgs.srv import Query
import rclpy
from rclpy.node import Node


class RosDbClient(Node):

    def __init__(self):
        super().__init__('ros_database_client_async')
        self.cli = self.create_client(Query, '/ros_typedb/query')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Query.Request()

    def get_rooms(self, ):
        self.req.query_type = "get"
        self.req.query = """
			match
                $room isa room, has name $room_name;
            get $room, $room_name;
		"""
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    rclpy.init(args=args)

    ros_db_client = RosDbClient()
    response = ros_db_client.get_rooms()
    for result in response.results:
        ros_db_client.get_logger().info(f"{result.attributes[0].name}, {result.attributes[0].label}, {result.attributes[0].value.string_value}")

    ros_db_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()