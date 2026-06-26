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

from datetime import datetime
import logging
import threading
from threading import Lock
import time
from typing import cast

import pytest

from ros_typedb.typedb_interface import convert_py_type_to_query_type
from ros_typedb.typedb_interface import DatabaseQueryType
from ros_typedb.typedb_interface import string_to_string_array
from ros_typedb.typedb_interface import TypeDBInterface

from typedb.driver import SessionType
from typedb.driver import TransactionType
from typedb.driver import TypeDBOptions


# ---------------------------------------------------------------------------
# Helpers shared by timeout tests
# ---------------------------------------------------------------------------

def _make_hung_tdb(query_timeout_s):
    """
    Fake TypeDBInterface (no __init__) with an unresponsive server.

    FakeDriver.close() unblocks the hung session so the daemon thread can exit.
    _query_timeout_s is set so the FIXED code can apply its timeout; the
    current (unfixed) database_query ignores the attribute and hangs forever.
    """
    blocked = threading.Event()

    class HungTransaction:

        def __enter__(self): return self
        def __exit__(self, *args): return False

        class query:

            @staticmethod
            def fetch(q):
                blocked.wait(timeout=10)
                return []

        def commit(self): pass

    class HungSession:

        def __enter__(self): return self
        def __exit__(self, *args): return False
        def transaction(self, *args, **kwargs): return HungTransaction()

    class FakeDriver:

        class databases:

            @staticmethod
            def contains(name): return True

        def session(self, *args, **kwargs): return HungSession()
        def close(self): blocked.set()

    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb._database_query_lock = Lock()
    tdb._infer = False
    tdb._sort_fetch_results = False
    tdb.database_name = 'test_database'
    tdb.last_error = ''
    tdb._address = 'localhost:1729'
    tdb._driver_timeout_s = 10.0
    tdb.logger = logging.getLogger()
    tdb.driver = FakeDriver()
    tdb._query_timeout_s = query_timeout_s
    return tdb


def _call_in_thread(fn, wall_clock_s=1.5):
    """
    Run fn() in a daemon thread, returning (thread, exc_box).

    Joins for wall_clock_s.  Caller checks thread.is_alive() to detect a hang.
    """
    exc_box = [None]

    def run():
        try:
            fn()
        except Exception as e:  # noqa: B902
            exc_box[0] = e

    t = threading.Thread(target=run, daemon=True)
    t.start()
    t.join(timeout=wall_clock_s)
    return t, exc_box


def test_database_query_raises_timeout_error_on_stalled_server():
    """
    database_query raises TimeoutError when the server stops responding.

    When _query_timeout_s is set and the TypeDB server stalls mid-query,
    database_query must raise TimeoutError within the deadline instead of
    blocking the caller indefinitely.
    """
    tdb = _make_hung_tdb(query_timeout_s=0.05)

    thread, exc_box = _call_in_thread(
        lambda: tdb.database_query(
            SessionType.DATA, TransactionType.READ, 'fetch',
            'match $x isa thing; fetch $x;'),
        wall_clock_s=1.5)

    if thread.is_alive():
        pytest.fail(
            'database_query blocked for 1.5 s on a stalled server — '
            'timeout mechanism is not working; _query_timeout_s was 0.05 s.')

    assert isinstance(exc_box[0], TimeoutError), (
        f'Expected TimeoutError, got {exc_box[0]!r}')


def test_database_query_releases_lock_after_timeout_on_stalled_server():
    """
    Lock is released after timeout so subsequent callers are not blocked.

    When a query times out because the server stopped responding, closing the
    stale driver must unblock the worker thread so it can exit the lock
    context. If the lock is never released, all subsequent database_query
    calls pile up indefinitely.
    """
    tdb = _make_hung_tdb(query_timeout_s=0.05)

    thread, _ = _call_in_thread(
        lambda: tdb.database_query(
            SessionType.DATA, TransactionType.READ, 'fetch',
            'match $x isa thing; fetch $x;'),
        wall_clock_s=1.5)

    if thread.is_alive():
        pytest.fail(
            'database_query blocked for 1.5 s on a stalled server — '
            'cannot verify lock release because the call never returned.')

    acquired = tdb._database_query_lock.acquire(timeout=2.0)
    assert acquired, (
        '_database_query_lock not released after TimeoutError; '
        'subsequent callers would block indefinitely.')
    tdb._database_query_lock.release()


def test_fetch_database_returns_none_on_stalled_server():
    """
    fetch_database returns None and sets last_error when the server stalls.

    fetch_database wraps database_query and must catch TimeoutError, populate
    last_error, and return None — it must not propagate the exception or block.
    """
    tdb = _make_hung_tdb(query_timeout_s=0.05)

    result_box = [None]
    thread, exc_box = _call_in_thread(
        lambda: result_box.__setitem__(0, tdb.fetch_database(
            'match $x isa thing; fetch $x;')),
        wall_clock_s=1.5)

    if thread.is_alive():
        pytest.fail(
            'fetch_database blocked for 1.5 s on a stalled server — '
            'did not return within the query timeout.')

    assert result_box[0] is None, (
        f'fetch_database should return None on timeout, got {result_box[0]!r}')
    assert exc_box[0] is None, (
        f'fetch_database should not propagate exceptions, got {exc_box[0]!r}')
    assert tdb.last_error, 'last_error should be populated after a timeout'


def _make_tdb_with_slow_driver(timeout_s):
    """Fake TypeDBInterface whose driver blocks until closed."""
    blocked = threading.Event()

    class SlowTransaction:

        def __enter__(self): return self
        def __exit__(self, *args): return False

        class query:

            @staticmethod
            def fetch(q):
                blocked.wait(timeout=5)
                return []

        def commit(self): pass

    class SlowSession:

        def __enter__(self): return self
        def __exit__(self, *args): return False
        def transaction(self, *args, **kwargs): return SlowTransaction()

    class FakeDriver:

        class databases:

            @staticmethod
            def contains(name): return True

        def session(self, *args, **kwargs): return SlowSession()
        def close(self): blocked.set()

    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb._query_timeout_s = timeout_s
    tdb._database_query_lock = Lock()
    tdb._infer = False
    tdb._sort_fetch_results = False
    tdb.database_name = 'test_database'
    tdb.last_error = ''
    tdb._address = 'localhost:1729'
    tdb._driver_timeout_s = 10.0
    tdb.logger = logging.getLogger()
    tdb.driver = FakeDriver()
    return tdb


def test_database_query_raises_timeout_error():
    """database_query raises TimeoutError when query hangs past timeout."""
    tdb = _make_tdb_with_slow_driver(timeout_s=0.05)

    start = time.monotonic()
    with pytest.raises(TimeoutError):
        tdb.database_query(
            SessionType.DATA, TransactionType.READ, 'fetch',
            'match $x isa thing; fetch $x;')
    assert time.monotonic() - start < 1.0


def test_database_query_lock_released_after_timeout():
    """After a timeout, lock is released so the next call can proceed."""
    tdb = _make_tdb_with_slow_driver(timeout_s=0.05)

    with pytest.raises(TimeoutError):
        tdb.database_query(
            SessionType.DATA, TransactionType.READ, 'fetch',
            'match $x isa thing; fetch $x;')

    acquired = tdb._database_query_lock.acquire(timeout=2.0)
    assert acquired, 'lock was not released within 2 seconds after timeout'
    tdb._database_query_lock.release()


def test_database_query_no_timeout_when_query_timeout_s_is_none():
    """Query runs without deadline when no timeout is configured."""
    calls = []

    class FastTransaction:

        def __enter__(self): return self
        def __exit__(self, *args): return False

        class query:

            @staticmethod
            def fetch(q):
                calls.append(q)
                return []

        def commit(self): pass

    class FastSession:

        def __enter__(self): return self
        def __exit__(self, *args): return False
        def transaction(self, *args, **kwargs): return FastTransaction()

    class FakeDriver:

        class databases:

            @staticmethod
            def contains(name): return True

        def session(self, *args, **kwargs): return FastSession()

    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb._query_timeout_s = None
    tdb._database_query_lock = Lock()
    tdb._infer = False
    tdb.database_name = 'test_database'
    tdb.last_error = ''
    tdb.logger = logging.getLogger()
    tdb.driver = FakeDriver()

    result = tdb.database_query(
        SessionType.DATA, TransactionType.READ, 'fetch',
        'match $x isa thing; fetch $x;')
    assert result == []
    assert calls == ['match $x isa thing; fetch $x;']


def test_database_query_per_call_timeout_overrides_default():
    """A per-call timeout takes precedence over _query_timeout_s=None."""
    tdb = _make_tdb_with_slow_driver(timeout_s=None)

    with pytest.raises(TimeoutError):
        tdb.database_query(
            SessionType.DATA, TransactionType.READ, 'fetch',
            'match $x isa thing; fetch $x;',
            timeout=0.05)


def test_database_query_sets_transaction_timeout_millis():
    """transaction_timeout_millis is set to effective_timeout * 1000."""
    captured_options = []

    class CapturingTransaction:

        def __enter__(self): return self
        def __exit__(self, *args): return False

        class query:

            @staticmethod
            def fetch(q): return []

        def commit(self): pass

    class CapturingSession:

        def __enter__(self): return self
        def __exit__(self, *args): return False

        def transaction(self, txn_type, options):
            captured_options.append(options)
            return CapturingTransaction()

    class FakeDriver:

        class databases:

            @staticmethod
            def contains(name): return True

        def session(self, *args, **kwargs): return CapturingSession()

    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb._query_timeout_s = None
    tdb._database_query_lock = Lock()
    tdb._infer = False
    tdb.database_name = 'test_database'
    tdb.last_error = ''
    tdb.logger = logging.getLogger()
    tdb.driver = FakeDriver()

    tdb.database_query(
        SessionType.DATA, TransactionType.READ, 'fetch',
        'match $x isa thing; fetch $x;',
        timeout=2.5)

    assert captured_options[0].transaction_timeout_millis == 2500


def test_create_session_uses_fresh_default_options():
    """create_session creates a fresh TypeDBOptions when none is provided."""
    captured_options = []

    class FakeDriver:

        def session(self, database_name, session_type, options):
            captured_options.append(options)
            return 'session'

    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb.driver = FakeDriver()

    first_session = tdb.create_session('test_database', SessionType.DATA)
    second_session = tdb.create_session('test_database', SessionType.DATA)

    assert first_session == 'session'
    assert second_session == 'session'
    assert len(captured_options) == 2
    assert isinstance(captured_options[0], TypeDBOptions)
    assert isinstance(captured_options[1], TypeDBOptions)
    assert captured_options[0] is not captured_options[1]


def test_database_query_batch_uses_one_transaction():
    """database_query can run an ordered query batch in one transaction."""
    calls = []

    class CapturingQuery:

        @staticmethod
        def delete(query):
            calls.append(('delete', query))

        @staticmethod
        def insert(query):
            calls.append(('insert', query))
            return iter(['inserted'])

    class CapturingTransaction:

        query = CapturingQuery()

        def __enter__(self): return self
        def __exit__(self, *args): return False
        def commit(self): calls.append(('commit', None))

    class CapturingSession:

        def __enter__(self): return self
        def __exit__(self, *args): return False

        def transaction(self, txn_type, options):
            calls.append(('transaction', txn_type))
            return CapturingTransaction()

    class FakeDriver:

        class databases:

            @staticmethod
            def contains(name): return True

        def session(self, *args, **kwargs): return CapturingSession()

    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb._query_timeout_s = None
    tdb._database_query_lock = Lock()
    tdb._infer = False
    tdb.database_name = 'test_database'
    tdb.last_error = ''
    tdb.logger = logging.getLogger()
    tdb.driver = FakeDriver()

    result = tdb.database_query(
        SessionType.DATA,
        TransactionType.WRITE,
        ['delete', 'insert'],
        ['delete query', 'insert query'])

    assert result == [True, ['inserted']]
    assert calls == [
        ('transaction', TransactionType.WRITE),
        ('delete', 'delete query'),
        ('insert', 'insert query'),
        ('commit', None),
    ]


def test_database_query_batch_rejects_mismatched_query_lists():
    """database_query rejects batches with unmatched query types and queries."""
    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb._query_timeout_s = None
    tdb._database_query_lock = Lock()
    tdb._infer = False
    tdb.database_name = 'test_database'
    tdb.last_error = ''
    tdb.logger = logging.getLogger()

    class FakeDriver:

        class databases:

            @staticmethod
            def contains(name): return True

    tdb.driver = FakeDriver()

    with pytest.raises(ValueError, match='same length'):
        tdb.database_query(
            SessionType.DATA,
            TransactionType.WRITE,
            ['delete', 'insert'],
            ['delete query'])


def _make_validation_tdb():
    """Create a TypeDBInterface that fails if validation opens a session."""
    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb._query_timeout_s = None
    tdb._database_query_lock = Lock()
    tdb._infer = False
    tdb.database_name = 'test_database'
    tdb.last_error = ''
    tdb.logger = logging.getLogger()

    class FakeDriver:

        class databases:

            @staticmethod
            def contains(name): return True

        def session(self, *args, **kwargs):
            raise AssertionError('validation should fail before session open')

    tdb.driver = FakeDriver()
    return tdb


def test_database_query_rejects_unsupported_query_type():
    """database_query rejects unknown query types before opening a session."""
    tdb = _make_validation_tdb()
    invalid_query_type = cast(DatabaseQueryType, 'bad_query')

    with pytest.raises(ValueError, match='Unsupported TypeDB query type'):
        tdb.database_query(
            SessionType.DATA,
            TransactionType.WRITE,
            invalid_query_type,
            'insert $x isa thing;')


def test_database_query_rejects_write_query_in_read_transaction():
    """database_query rejects write query types in read transactions."""
    tdb = _make_validation_tdb()

    with pytest.raises(ValueError, match='Read transactions only support'):
        tdb.database_query(
            SessionType.DATA,
            TransactionType.READ,
            'insert',
            'insert $x isa thing;')


def test_database_query_rejects_read_query_in_write_transaction():
    """database_query rejects read query types in write transactions."""
    tdb = _make_validation_tdb()

    with pytest.raises(ValueError, match='only support query types'):
        tdb.database_query(
            SessionType.DATA,
            TransactionType.WRITE,
            'fetch',
            'match $x isa thing; fetch $x;')


def test_database_query_rejects_define_query_in_data_session():
    """database_query requires schema write queries to use schema sessions."""
    tdb = _make_validation_tdb()

    with pytest.raises(ValueError, match='only support query types'):
        tdb.database_query(
            SessionType.DATA,
            TransactionType.WRITE,
            'define',
            'define person sub entity;')


def test_raw_wrapper_logs_validation_error(caplog, monkeypatch):
    """Public wrappers catch validation errors and log them."""
    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb.last_error = ''
    tdb.logger = logging.getLogger()

    def raise_validation_error(*args, **kwargs):
        raise ValueError('Unsupported TypeDB query type: bad_query')

    monkeypatch.setattr(tdb, 'database_query', raise_validation_error)

    with caplog.at_level(logging.WARNING):
        result = tdb.insert_database('insert $x isa thing;')

    assert result is None
    assert tdb.last_error == 'Unsupported TypeDB query type: bad_query'
    assert 'Unsupported TypeDB query type: bad_query' in caplog.text


def test_string_to_string_array_preserves_raw_path_with_comma():
    path = '/tmp/schema,with-comma.tql'

    assert string_to_string_array(path) == [path]


def test_string_to_string_array_parses_quoted_list_with_comma_in_path():
    assert string_to_string_array(
        "['/tmp/schema,with-comma.tql', '/tmp/data.tql']"
    ) == ['/tmp/schema,with-comma.tql', '/tmp/data.tql']


def test_string_to_string_array_keeps_unbracketed_strings_as_single_values():
    assert string_to_string_array('/tmp/schema.tql,/tmp/data.tql') == [
        '/tmp/schema.tql,/tmp/data.tql'
    ]


@pytest.mark.parametrize('method_name,extra_kwargs', [
    ('insert_database', {}),
    ('update_database', {}),
    ('delete_from_database', {}),
    ('define_database', {}),
    ('fetch_database', {}),
    ('get_database', {}),
    ('get_aggregate_database', {}),
])
def test_raw_wrapper_passes_timeout_to_database_query(
        monkeypatch, method_name, extra_kwargs):
    """Each raw wrapper forwards timeout= to database_query."""
    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb.last_error = ''
    tdb._sort_fetch_results = False
    tdb._infer = False
    tdb.logger = logging.getLogger()

    captured = {}

    def fake_database_query(*args, **kwargs):
        captured['timeout'] = kwargs.get('timeout')
        return (
            [] if method_name in ('fetch_database', 'get_database')
            else True)

    monkeypatch.setattr(tdb, 'database_query', fake_database_query)
    getattr(tdb, method_name)('match $x isa thing; fetch $x;', timeout=7.0)

    assert captured.get('timeout') == 7.0


@pytest.mark.parametrize('method_name', [
    'insert_database',
    'update_database',
    'delete_from_database',
    'define_database',
    'fetch_database',
    'get_database',
    'get_aggregate_database',
])
def test_raw_wrapper_catches_timeout_error_and_sets_last_error(
        monkeypatch, method_name):
    """Each wrapper catches TimeoutError, sets last_error, and returns None."""
    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb.last_error = ''
    tdb._sort_fetch_results = False
    tdb._infer = False
    tdb.logger = logging.getLogger()

    def raise_timeout(*args, **kwargs):
        raise TimeoutError('Query timed out after 0.05s')

    monkeypatch.setattr(tdb, 'database_query', raise_timeout)
    result = getattr(tdb, method_name)('match $x isa thing; fetch $x;')

    assert result is None
    assert 'timed out' in tdb.last_error.lower()


def test_ensure_database_exists_recreates_from_configured_paths(monkeypatch):
    """Missing database is recreated with configured schema and data files."""
    class FakeDatabases:

        def __init__(self):
            self.exists = False
            self.created = []

        def contains(self, name):
            return self.exists

        def create(self, name):
            self.created.append(name)
            self.exists = True

    fake_databases = FakeDatabases()
    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb.database_name = 'test_database'
    tdb.driver = type('FakeDriver', (), {'databases': fake_databases})()
    tdb.logger = logging.getLogger()
    tdb._schema_paths = ['schema.tql']
    tdb._data_paths = ['data.tql']
    loaded = []

    monkeypatch.setattr(
        tdb,
        '_load_schema_unlocked',
        lambda path: loaded.append(('schema', path)))
    monkeypatch.setattr(
        tdb,
        '_load_data_unlocked',
        lambda path: loaded.append(('data', path)))
    monkeypatch.setattr(
        tdb,
        'database_query',
        lambda *args, **kwargs: pytest.fail(
            'ensure_database_exists must not call database_query'))

    tdb.ensure_database_exists()

    assert fake_databases.created == ['test_database']
    assert loaded == [('schema', 'schema.tql'), ('data', 'data.tql')]


def test_ensure_database_exists_refuses_empty_recreate():
    """Missing database without init files must fail instead of going blank."""
    class FakeDatabases:

        def __init__(self):
            self.created = []

        def contains(self, name):
            return False

        def create(self, name):
            self.created.append(name)

    fake_databases = FakeDatabases()
    tdb = TypeDBInterface.__new__(TypeDBInterface)
    tdb.database_name = 'test_database'
    tdb.driver = type('FakeDriver', (), {'databases': fake_databases})()
    tdb.logger = logging.getLogger()
    tdb._schema_paths = []
    tdb._data_paths = []

    with pytest.raises(RuntimeError, match='cannot be recreated'):
        tdb.ensure_database_exists()

    assert fake_databases.created == []


def test_init_skips_schema_reload_when_database_is_reused(monkeypatch):
    """reload_schema=False skips define queries for an existing database."""
    loaded_schema_paths = []

    monkeypatch.setattr(
        TypeDBInterface, 'connect_driver', lambda self, *args, **kwargs: None)
    monkeypatch.setattr(
        TypeDBInterface, 'create_database',
        lambda self, *args, **kwargs: False)
    monkeypatch.setattr(
        TypeDBInterface, 'load_schema',
        lambda self, path: loaded_schema_paths.append(path))
    monkeypatch.setattr(
        TypeDBInterface, 'delete_all_data', lambda self: None)
    monkeypatch.setattr(
        TypeDBInterface, 'load_data', lambda self, path: None)

    TypeDBInterface(
        'localhost:1729',
        'test_database',
        schema_path=['schema.tql'],
        reload_schema=False)

    assert loaded_schema_paths == []


def test_init_loads_schema_for_new_database_when_reload_disabled(monkeypatch):
    """Fresh databases still get schema even when reload_schema=False."""
    loaded_schema_paths = []

    monkeypatch.setattr(
        TypeDBInterface, 'connect_driver', lambda self, *args, **kwargs: None)
    monkeypatch.setattr(
        TypeDBInterface, 'create_database',
        lambda self, *args, **kwargs: True)
    monkeypatch.setattr(
        TypeDBInterface, 'load_schema',
        lambda self, path: loaded_schema_paths.append(path))
    monkeypatch.setattr(
        TypeDBInterface, 'delete_all_data', lambda self: None)
    monkeypatch.setattr(
        TypeDBInterface, 'load_data', lambda self, path: None)

    TypeDBInterface(
        'localhost:1729',
        'test_database',
        schema_path=['schema.tql'],
        reload_schema=False)

    assert loaded_schema_paths == ['schema.tql']


def test_delete_all_data_uses_single_atomic_database_query(monkeypatch):
    """delete_all_data runs all deletes in one write transaction."""
    typedb_interface = TypeDBInterface.__new__(TypeDBInterface)
    typedb_interface.last_error = ''
    typedb_interface.logger = logging.getLogger()
    captured = {}

    def fake_database_query(session_type, transaction_type, query_type, query):
        captured['session_type'] = session_type
        captured['transaction_type'] = transaction_type
        captured['query_type'] = query_type
        captured['query'] = query
        return True

    monkeypatch.setattr(
        typedb_interface,
        'database_query',
        fake_database_query)
    monkeypatch.setattr(
        typedb_interface,
        'delete_from_database',
        lambda query: pytest.fail('delete_all_data must use one batch'))

    typedb_interface.delete_all_data()

    assert captured['session_type'] == SessionType.DATA
    assert captured['transaction_type'] == TransactionType.WRITE
    assert captured['query_type'] == ['delete', 'delete', 'delete']
    assert captured['query'] == [
        'match $e isa entity; delete $e isa entity;',
        'match $r isa relation; delete $r isa relation;',
        'match $a isa attribute; delete $a isa attribute;',
    ]


def test_delete_all_data_raises_when_delete_batch_fails(monkeypatch):
    """delete_all_data stops instead of loading data over residual data."""
    typedb_interface = TypeDBInterface.__new__(TypeDBInterface)
    typedb_interface.last_error = ''
    typedb_interface.logger = logging.getLogger()

    def fake_database_query(*args):
        raise RuntimeError('delete failed')

    monkeypatch.setattr(
        typedb_interface,
        'database_query',
        fake_database_query)

    with pytest.raises(RuntimeError, match='delete failed'):
        typedb_interface.delete_all_data()

    assert typedb_interface.last_error == 'delete failed'


def test_update_attribute_in_thing_uses_atomic_database_query(monkeypatch):
    """Single-attribute update deletes and inserts in one transaction."""
    typedb_interface = TypeDBInterface.__new__(TypeDBInterface)
    typedb_interface.last_error = ''
    typedb_interface.logger = logging.getLogger()
    captured = {}

    def fake_database_query(session_type, transaction_type, query_type, query):
        captured['session_type'] = session_type
        captured['transaction_type'] = transaction_type
        captured['query_type'] = query_type
        captured['query'] = query
        return [True, ['insert result']]

    monkeypatch.setattr(
        typedb_interface,
        'database_query',
        fake_database_query)
    monkeypatch.setattr(
        typedb_interface,
        'delete_attribute_from_thing',
        lambda *args: pytest.fail('update must not call public delete'))
    monkeypatch.setattr(
        typedb_interface,
        'insert_attribute_in_thing',
        lambda *args: pytest.fail('update must not call public insert'))

    result = typedb_interface.update_attribute_in_thing(
        'person', 'email', "o'brien@test.test", 'age', 42)

    assert result == ['insert result']
    assert captured['session_type'] == SessionType.DATA
    assert captured['transaction_type'] == TransactionType.WRITE
    assert captured['query_type'] == ['delete', 'insert']
    assert "has email 'o\\'brien@test.test'" in captured['query'][0]
    assert 'has age $attribute' in captured['query'][0]
    assert 'delete $thing has $attribute;' in captured['query'][0]
    assert 'insert $thing has age 42;' in captured['query'][1]


def test_update_attributes_in_thing_uses_atomic_database_query(monkeypatch):
    """Multi-attribute update deletes and inserts in one transaction."""
    typedb_interface = TypeDBInterface.__new__(TypeDBInterface)
    typedb_interface.last_error = ''
    typedb_interface.logger = logging.getLogger()
    captured = {}

    def fake_database_query(session_type, transaction_type, query_type, query):
        captured['session_type'] = session_type
        captured['transaction_type'] = transaction_type
        captured['query_type'] = query_type
        captured['query'] = query
        return [True, ['insert result']]

    monkeypatch.setattr(
        typedb_interface,
        'database_query',
        fake_database_query)
    monkeypatch.setattr(
        typedb_interface,
        'delete_attributes_from_thing',
        lambda *args: pytest.fail('update must not call public delete'))
    monkeypatch.setattr(
        typedb_interface,
        'insert_attributes_in_thing',
        lambda *args: pytest.fail('update must not call public insert'))

    result = typedb_interface.update_attributes_in_thing({
        'person': [
            {
                'prefix': 'p1',
                'attributes': {'email': 'test@test.test'},
                'update_attributes': {'height': 1.5, 'age': 17},
            },
        ],
    })

    assert result == ['insert result']
    assert captured['session_type'] == SessionType.DATA
    assert captured['transaction_type'] == TransactionType.WRITE
    assert captured['query_type'] == ['delete', 'insert']
    assert 'delete $p1 has $p1_height, has $p1_age;' in captured['query'][0]
    assert 'insert  $p1  isa person,' in captured['query'][1]
    assert 'has height 1.5, has age 17;' in captured['query'][1]


def test_update_attribute_in_thing_returns_none_when_batch_fails(monkeypatch):
    """Single-attribute update preserves wrapper-style failure behavior."""
    typedb_interface = TypeDBInterface.__new__(TypeDBInterface)
    typedb_interface.last_error = ''
    typedb_interface.logger = logging.getLogger()

    def fake_database_query(*args):
        raise RuntimeError('update failed')

    monkeypatch.setattr(
        typedb_interface,
        'database_query',
        fake_database_query)

    result = typedb_interface.update_attribute_in_thing(
        'person', 'email', 'test@test.test', 'age', 42)

    assert result is None
    assert typedb_interface.last_error == 'update failed'


def test_update_attributes_in_thing_returns_none_when_batch_fails(monkeypatch):
    """Multi-attribute update preserves wrapper-style failure behavior."""
    typedb_interface = TypeDBInterface.__new__(TypeDBInterface)
    typedb_interface.last_error = ''
    typedb_interface.logger = logging.getLogger()

    def fake_database_query(*args):
        raise RuntimeError('update failed')

    monkeypatch.setattr(
        typedb_interface,
        'database_query',
        fake_database_query)

    result = typedb_interface.update_attributes_in_thing({
        'person': [
            {
                'prefix': 'p1',
                'attributes': {'email': 'test@test.test'},
                'update_attributes': {'age': 42},
            },
        ],
    })

    assert result is None
    assert typedb_interface.last_error == 'update failed'


def test_convert_py_type_to_query_type_escapes_strings():
    assert convert_py_type_to_query_type("O'Brien") == "'O\\'Brien'"
    assert convert_py_type_to_query_type(r'C:\\tmp') == r"'C:\\\\tmp'"
    assert convert_py_type_to_query_type('$person') == '$person'


def test_delete_thing_uses_query_type_conversion(monkeypatch):
    typedb_interface = TypeDBInterface.__new__(TypeDBInterface)
    captured_query = None

    def fake_delete_from_database(query):
        nonlocal captured_query
        captured_query = query
        return True

    monkeypatch.setattr(
        typedb_interface, 'delete_from_database', fake_delete_from_database)

    assert typedb_interface.delete_thing('person', 'name', "O'Brien") is True
    assert "has name 'O\\'Brien'" in captured_query
    assert 'has name "O\'Brien"' not in captured_query


def test_driver_connection_timeout(monkeypatch):
    def slow_core_driver(address):
        time.sleep(1.0)

    monkeypatch.setattr(
        'ros_typedb.typedb_interface.TypeDB.core_driver',
        slow_core_driver
    )

    start_time = time.monotonic()
    with pytest.raises(TimeoutError):
        TypeDBInterface(
            'localhost:1729',
            'test_database',
            driver_timeout_s=0.01
        )

    assert time.monotonic() - start_time < 0.5


def test_driver_connection_timeout_closes_late_driver(monkeypatch):
    class LateDriver:

        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    late_driver = LateDriver()

    def slow_core_driver(address):
        time.sleep(0.05)
        return late_driver

    monkeypatch.setattr(
        'ros_typedb.typedb_interface.TypeDB.core_driver',
        slow_core_driver
    )

    with pytest.raises(TimeoutError):
        TypeDBInterface(
            'localhost:1729',
            'test_database',
            driver_timeout_s=0.01
        )

    deadline = time.monotonic() + 0.5
    while not late_driver.closed and time.monotonic() < deadline:
        time.sleep(0.01)

    assert late_driver.closed


@pytest.mark.parametrize('key_value, expected_query_value', [
    ('test@email.test', "'test@email.test'"),
    (33, '33'),
    (3.237, '3.237'),
    (True, 'true'),
    (datetime.fromisoformat('2026-06-14T12:34:56.789'),
     '2026-06-14T12:34:56.789'),
])
def test_delete_thing_formats_key_value_for_type(key_value, expected_query_value):
    typedb_interface = TypeDBInterface.__new__(TypeDBInterface)
    queries = []

    def capture_delete(query):
        queries.append(query)
        return True

    typedb_interface.delete_from_database = capture_delete

    assert typedb_interface.delete_thing('person', 'email', key_value) is True
    assert f'has email {expected_query_value};' in queries[0]


def test_database_query_reconnects_after_failed_health_check(monkeypatch):
    class FakeDatabases:
        """Fake database collection with controllable health checks."""

        def __init__(self, fail_after_first_contains=False):
            self.fail_after_first_contains = fail_after_first_contains
            self.contains_count = 0

        def contains(self, database_name):
            self.contains_count += 1
            if self.fail_after_first_contains and self.contains_count > 1:
                raise RuntimeError('server unavailable')
            return True

        def create(self, database_name):
            raise AssertionError('database should already exist')

    class FakeDriver:
        """Fake TypeDB driver."""

        def __init__(self, fail_after_first_contains=False):
            self.databases = FakeDatabases(fail_after_first_contains)
            self.closed = False
            self.session_count = 0

        def session(self, database_name, session_type, options):
            self.session_count += 1
            return FakeSession()

        def close(self):
            self.closed = True

    class FakeSession:
        """Fake TypeDB session context manager."""

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback):
            return False

        def transaction(self, transaction_type, options):
            return FakeTransaction()

    class FakeQuery:
        """Fake TypeDB query API."""

        def fetch(self, query):
            return [{'person': {'type': {'root': 'entity', 'label': 'person'}}}]

    class FakeTransaction:
        """Fake TypeDB transaction context manager."""

        query = FakeQuery()

        def __enter__(self):
            return self

        def __exit__(self, exc_type, exc, traceback):
            return False

    drivers = [FakeDriver(fail_after_first_contains=True), FakeDriver()]
    monkeypatch.setattr(
        'ros_typedb.typedb_interface.TypeDB.core_driver',
        lambda address: drivers.pop(0)
    )

    typedb_interface = TypeDBInterface('localhost:1729', 'test_database')
    stale_driver = typedb_interface.driver

    result = typedb_interface.fetch_database('match $p isa person; fetch $p;')

    assert result == [
        {'person': {'type': {'root': 'entity', 'label': 'person'}}}]
    assert stale_driver.closed is True
    assert stale_driver.session_count == 0
    assert typedb_interface.driver.session_count == 1


def test_ensure_server_alive_refuses_empty_recreate(monkeypatch):
    class FakeDatabases:
        """Fake database collection tracking database creation."""

        def __init__(self):
            self.created_database = None

        def contains(self, database_name):
            return self.created_database == database_name

        def create(self, database_name):
            self.created_database = database_name

    class FakeDriver:
        """Fake TypeDB driver."""

        def __init__(self):
            self.databases = FakeDatabases()

        def close(self):
            pass

    monkeypatch.setattr(
        'ros_typedb.typedb_interface.TypeDB.core_driver',
        lambda address: FakeDriver()
    )

    typedb_interface = TypeDBInterface('localhost:1729', 'test_database')
    typedb_interface.driver.databases.created_database = None

    with pytest.raises(RuntimeError, match='cannot be recreated'):
        typedb_interface.ensure_server_alive()

    assert typedb_interface.driver.databases.created_database is None
