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
"""Loader for external YAML/JSON stress experiment profiles."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

from ros_typedb_benchmark.stress_config import InvariantSpec
from ros_typedb_benchmark.stress_config import QUERY_TYPE_BY_NAME
from ros_typedb_benchmark.stress_config import QuerySpec
from ros_typedb_benchmark.stress_mixed import MixedQueryProfile
from ros_typedb_benchmark.stress_mixed import MixedQueryTemplate


@dataclass(frozen=True)
class StressProfile:
    """Loaded external stress profile."""

    description: str
    setup_specs: list[QuerySpec]
    teardown_specs: list[QuerySpec]
    read_query_specs: list[QuerySpec] | None
    mixed_profile: MixedQueryProfile | None
    invariant_specs: list[InvariantSpec] | None


def load_stress_profile(profile_path: Path) -> StressProfile:
    """Load a YAML or JSON stress profile from disk.

    YAML is used for ``.yaml``/``.yml`` extensions; JSON for everything else.
    Raises ``FileNotFoundError`` if the file does not exist, ``ValueError``
    for invalid content.
    """
    profile_path = Path(profile_path)
    if not profile_path.exists():
        raise FileNotFoundError(
            f'profile file not found: {profile_path}'
        )
    raw = profile_path.read_text(encoding='utf-8')
    if profile_path.suffix.lower() in ('.yaml', '.yml'):
        import yaml  # noqa: PLC0415
        payload = yaml.safe_load(raw) or {}
    else:
        payload = json.loads(raw)

    description = payload.get('description', '')
    setup_specs = _parse_setup_specs(payload, profile_path)
    teardown_specs = _parse_query_spec_list(
        payload.get('teardown_queries', []), 'teardown_queries'
    )
    read_query_specs = _parse_read_query_specs(payload)
    mixed_profile = _parse_mixed_profile(payload)
    invariant_specs = _parse_invariant_specs(payload)

    return StressProfile(
        description=description,
        setup_specs=setup_specs,
        teardown_specs=teardown_specs,
        read_query_specs=read_query_specs,
        mixed_profile=mixed_profile,
        invariant_specs=invariant_specs,
    )


def _parse_setup_specs(
    payload: dict,
    profile_path: Path,
) -> list[QuerySpec]:
    """Build setup specs from schema_path, data_path, and setup_queries."""
    specs: list[QuerySpec] = []

    schema_path = payload.get('schema_path')
    if schema_path:
        schema_file = _resolve_path(schema_path, profile_path)
        specs.append(
            QuerySpec(
                query=schema_file.read_text(encoding='utf-8'),
                query_type='define',
            )
        )

    data_path = payload.get('data_path')
    if data_path:
        data_file = _resolve_path(data_path, profile_path)
        specs.append(
            QuerySpec(
                query=data_file.read_text(encoding='utf-8'),
                query_type='insert',
            )
        )

    specs.extend(
        _parse_query_spec_list(
            payload.get('setup_queries', []), 'setup_queries'
        )
    )
    return specs


def _resolve_path(path_str: str, profile_path: Path) -> Path:
    """Resolve a path from the profile, relative to the profile file."""
    p = Path(path_str)
    if not p.is_absolute():
        p = profile_path.parent / p
    if not p.exists():
        raise FileNotFoundError(f'referenced file not found: {p}')
    return p


def _parse_query_spec_list(
    items: list,
    section: str,
) -> list[QuerySpec]:
    """Parse a list of query dicts into QuerySpec objects."""
    specs = []
    for item in items:
        query_type = item.get('query_type', '')
        if query_type not in QUERY_TYPE_BY_NAME:
            raise ValueError(
                f'unsupported query type {query_type!r} in {section}'
            )
        specs.append(
            QuerySpec(
                query=item['query'],
                query_type=query_type,
            )
        )
    return specs


def _parse_read_query_specs(payload: dict) -> list[QuerySpec] | None:
    """Parse read_queries section; None if absent."""
    raw = payload.get('read_queries')
    if raw is None:
        return None
    return _parse_query_spec_list(raw, 'read_queries')


def _parse_mixed_profile(payload: dict) -> MixedQueryProfile | None:
    """Parse mixed section; None if absent."""
    raw = payload.get('mixed')
    if raw is None:
        return None
    templates = []
    for item in raw.get('queries', []):
        query_type = item.get('query_type', '')
        if query_type not in QUERY_TYPE_BY_NAME:
            raise ValueError(
                f'unsupported query type {query_type!r} in mixed.queries'
            )
        templates.append(
            MixedQueryTemplate(
                query=item['query'],
                query_type=query_type,
                tracks_cleanup=item.get('tracks_cleanup', False),
                clears_cleanup=item.get('clears_cleanup', False),
            )
        )
    if not templates:
        raise ValueError('mixed profile has no queries')
    cleanup_query = raw.get('cleanup_query', '')
    if not cleanup_query:
        raise ValueError('mixed profile missing cleanup_query')
    return MixedQueryProfile(
        name='external',
        cleanup_query=cleanup_query,
        templates=templates,
    )


def _parse_invariant_specs(payload: dict) -> list[InvariantSpec] | None:
    """Parse invariants section; None if absent."""
    raw = payload.get('invariants')
    if raw is None:
        return None
    specs = []
    for item in raw:
        query_type = item.get('query_type', '')
        if query_type not in QUERY_TYPE_BY_NAME:
            raise ValueError(
                f'unsupported query type {query_type!r} in invariants'
            )
        specs.append(
            InvariantSpec(
                name=item['name'],
                query=item['query'],
                query_type=query_type,
                expected_value=item['expected_value'],
            )
        )
    return specs
