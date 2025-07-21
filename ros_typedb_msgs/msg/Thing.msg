uint8 ENTITY = 1
uint8 RELATION = 2

uint8 type  # ENTITY or RELATION
string variable_name  # The name of the TypeDB variable used in the TQL query, e.g., $name, $address etc.
string type_name # The specific type of this Thing, e.g., Person, Company etc.

ros_typedb_msgs/Attribute[] attributes  # List of the queried attributes of this Thing