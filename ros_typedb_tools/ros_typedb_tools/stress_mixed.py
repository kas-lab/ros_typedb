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
"""Mixed read/write profile loading for stress experiments."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
from typing import Any

from ros_typedb_tools.stress_config import MIXED_PROFILE_FILES
from ros_typedb_tools.stress_config import QUERY_TYPE_BY_NAME
from ros_typedb_tools.stress_config import QuerySpec


MIXED_PROFILE_DIR = Path(__file__).with_name('profiles')


@dataclass(frozen=True)
class MixedQueryTemplate:
    """One query template from a mixed read/write profile."""

    query: str
    query_type: str
    tracks_cleanup: bool = False
    clears_cleanup: bool = False


@dataclass(frozen=True)
class MixedQueryProfile:
    """Loaded mixed read/write profile."""

    name: str
    cleanup_query: str
    templates: list[MixedQueryTemplate]


def resolve_mixed_profile_name(args: Any) -> str:
    """Resolve the selected mixed profile name."""
    mixed_profile = getattr(args, 'mixed_profile', 'auto')
    if mixed_profile != 'auto':
        return mixed_profile

    invariant_profile = getattr(args, 'invariant_profile', 'none')
    if invariant_profile == 'plan-schema-mixed':
        return 'plan-schema'
    if invariant_profile in MIXED_PROFILE_FILES:
        return invariant_profile
    return 'test-data'


def load_mixed_query_profile(args: Any) -> MixedQueryProfile:
    """Load the mixed read/write profile selected by CLI arguments."""
    profile_name = resolve_mixed_profile_name(args)
    profile_filename = MIXED_PROFILE_FILES.get(profile_name)
    if profile_filename is None:
        raise ValueError(f'unsupported mixed profile: {profile_name}')
    return _load_mixed_query_profile(
        profile_name,
        MIXED_PROFILE_DIR / profile_filename,
    )


def _load_mixed_query_profile(
    profile_name: str,
    profile_path: Path,
) -> MixedQueryProfile:
    """Load one packaged mixed read/write profile."""
    payload = json.loads(profile_path.read_text(encoding='utf-8'))
    templates = []
    for query in payload.get('queries', []):
        query_type = query['query_type']
        if query_type not in QUERY_TYPE_BY_NAME:
            raise ValueError(f'unsupported mixed query type: {query_type}')
        templates.append(
            MixedQueryTemplate(
                query=query['query'],
                query_type=query_type,
                tracks_cleanup=query.get('tracks_cleanup', False),
                clears_cleanup=query.get('clears_cleanup', False),
            )
        )
    if not templates:
        raise ValueError(f'mixed profile has no queries: {profile_name}')
    return MixedQueryProfile(
        name=profile_name,
        cleanup_query=payload['cleanup_query'],
        templates=templates,
    )


def make_mixed_key(
    *,
    run_id: str,
    client_id: int,
    cycle: int,
) -> str:
    """Build the unique key for one experiment-owned temporary entity."""
    return f'ros-typedb-stress-{run_id}-c{client_id}-n{cycle}'


def build_mixed_query_spec(
    *,
    profile: MixedQueryProfile,
    run_id: str,
    client_id: int,
    client_request_index: int,
) -> QuerySpec:
    """
    Build one mixed-mode query for a client's local request sequence.

    The profile decides which schema-specific entity is written. The generated
    key is unique to this experiment run, so mixed mode does not touch baseline
    fixture data.
    """
    operation_index = client_request_index % len(profile.templates)
    cycle = client_request_index // len(profile.templates)
    template = profile.templates[operation_index]
    key = make_mixed_key(
        run_id=run_id,
        client_id=client_id,
        cycle=cycle,
    )
    cleanup_key = (
        key
        if template.tracks_cleanup or template.clears_cleanup
        else None
    )
    return QuerySpec(
        query=_render_mixed_template(template.query, key=key, cycle=cycle),
        query_type=template.query_type,
        cleanup_key=cleanup_key,
    )


def build_mixed_cleanup_query_spec(
    profile: MixedQueryProfile,
    key: str,
) -> QuerySpec:
    """Build a delete query for one experiment-owned temporary key."""
    return QuerySpec(
        query=_render_mixed_template(profile.cleanup_query, key=key, cycle=0),
        query_type='delete',
        cleanup_key=key,
    )


def _render_mixed_template(template: str, *, key: str, cycle: int) -> str:
    """Substitute the supported mixed-profile placeholders."""
    return template.format(
        key=key,
        cycle=cycle,
        cycle_float=f'{float(cycle):.1f}',
        cycle_plus_one=cycle + 1,
        cycle_plus_one_float=f'{float(cycle + 1):.1f}',
    )
