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

## Packaging

`typedb_utils` is both an `ament_python` workspace package and a standard
Python distribution. You can build it locally with:

```bash
cd typedb_utils
python3 -m pip install --upgrade build twine
python3 -m build
python3 -m twine check dist/*
```

The repository also includes GitHub Actions workflow
[`publish-typedb-utils.yml`](../.github/workflows/publish-typedb-utils.yml),
which builds the package from the `typedb_utils/` subdirectory and publishes it
to PyPI via Trusted Publishing on tags matching `typedb_utils-v*`.

Release flow:

1. bump the version in `typedb_utils/setup.py` and `typedb_utils/package.xml`
2. push the commit to `main`
3. create and push a matching tag such as `typedb_utils-v0.1.0`
4. ensure the PyPI project is configured with this GitHub workflow as a Trusted
   Publisher
