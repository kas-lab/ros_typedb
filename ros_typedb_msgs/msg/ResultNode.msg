
uint8 ENTITY = 1
uint8 RELATION = 2
uint8 SUB_QUERY = 3

uint8 node_type

uint8 node_id
uint8[] children_id

string name  # variable name or subquery name
string type_name

ros_typedb_msgs/Attribute[] attributes