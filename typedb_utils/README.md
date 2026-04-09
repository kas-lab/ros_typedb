# typedb_utils

`typedb_utils` is the reusable, non-ROS TypeDB 3 Python layer extracted from
the `ros_typedb` repository.

It provides:

- `TypeDBInterface` for connecting to TypeDB, loading schema/data, and running
  define/insert/delete/fetch/get/aggregate queries
- helper functions for converting Python values to TypeQL literals and building
  common query fragments

This package is still kept inside the same monorepo so it can be built with
`colcon` together with `ros_typedb`, but it has no ROS runtime dependencies.

Example:

```python
from typedb_utils.typedb_interface import TypeDBInterface


db = TypeDBInterface(
    'localhost:1729',
    'example_db',
    schema_path=['/abs/path/schema.tql'],
    data_path=['/abs/path/data.tql'],
    force_database=True,
)

rows = db.fetch_database(
    """
    match
        $p isa person;
    fetch { "email": $p.email };
    """
)
```
