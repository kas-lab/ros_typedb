uint8 THING = 1
uint8 ATTRIBUTE = 2
uint8 SUB_QUERY = 3

uint8 type  # THING, ATTRIBUTE, or SUB_QUERY

uint8 result_index  # The index of the result in the ResultTree
ros_typedb_msgs/IndexList[] children_index  # The index of this QueryResult children in the ResultTree. This is only useful for SUB_QUERY results

ros_typedb_msgs/Thing thing  # Should only be set when type is THING
ros_typedb_msgs/Attribute attribute  # Should only be set when type is ATTRIBUTE

string sub_query_name  # Should only be set when type is SUB_QUERY