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
"""Resolve raw stress CLI arguments into concrete runtime configuration."""

from __future__ import annotations

import argparse
from pathlib import Path

from ros_typedb_benchmark.stress_config import build_invariant_specs
from ros_typedb_benchmark.stress_config import build_query_specs
from ros_typedb_benchmark.stress_config import DEFAULT_TYPEDB_START_COMMAND
from ros_typedb_benchmark.stress_config import DEFAULT_TYPEDB_STOP_COMMAND
from ros_typedb_benchmark.stress_config import QuerySpec
from ros_typedb_benchmark.stress_config import ResolvedStressConfig
from ros_typedb_benchmark.stress_config import validate_experiment_args
from ros_typedb_benchmark.stress_mixed import load_mixed_query_profile
from ros_typedb_benchmark.stress_mixed import MixedQueryProfile
from ros_typedb_benchmark.stress_profile import load_stress_profile
from ros_typedb_benchmark.stress_profile import StressProfile


def resolve_stress_config(
    args: argparse.Namespace,
) -> ResolvedStressConfig:
    """
    Build the concrete configuration used by the runner and output writer.

    This is the only place that combines the raw CLI flags with packaged
    profiles. The runner receives ordinary query specs and an optional loaded
    mixed profile; it does not need to know how ``auto`` profile selection
    works.
    """
    validate_experiment_args(args)

    external_profile = _load_external_profile(args)
    _check_profile_conflicts(args, external_profile)

    setup_specs = _build_setup_specs(args, external_profile)
    teardown_specs = (
        external_profile.teardown_specs if external_profile else []
    )
    invariant_specs = _resolve_invariant_specs(args, external_profile)
    mixed_profile = _resolve_mixed_profile(args, external_profile)
    query_specs = _build_runtime_query_specs(args, mixed_profile,
                                             external_profile)
    debug_events_output = (
        Path(args.debug_events_output).expanduser()
        if args.debug_events_output
        else None
    )
    fault = getattr(args, 'fault', 'none')
    fault_at_s = getattr(args, 'fault_at_s', None)
    fault_recovery_timeout_s = getattr(args, 'fault_recovery_timeout_s', 30.0)
    typedb_container = getattr(args, 'typedb_container', None)
    typedb_stop_command = getattr(
        args, 'typedb_stop_command', DEFAULT_TYPEDB_STOP_COMMAND
    ) or DEFAULT_TYPEDB_STOP_COMMAND
    typedb_start_command = getattr(
        args, 'typedb_start_command', DEFAULT_TYPEDB_START_COMMAND
    ) or DEFAULT_TYPEDB_START_COMMAND
    typedb_restart_delay_s = getattr(args, 'typedb_restart_delay_s', 2.0)
    fault_command_timeout_s = getattr(args, 'fault_command_timeout_s', 30.0)
    lifecycle_change_state_service_name = getattr(
        args, 'lifecycle_change_state_service_name', None
    )
    lifecycle_get_state_service_name = getattr(
        args, 'lifecycle_get_state_service_name', None
    )
    lifecycle_reactivate = getattr(args, 'lifecycle_reactivate', True)
    lifecycle_transition_timeout_s = getattr(
        args, 'lifecycle_transition_timeout_s', 10.0
    )

    return ResolvedStressConfig(
        query_specs=query_specs,
        invariant_specs=invariant_specs,
        mixed_profile=mixed_profile,
        mixed_profile_name=(
            mixed_profile.name if mixed_profile is not None else None
        ),
        invariant_profile_name=args.invariant_profile,
        max_in_flight=args.max_in_flight or args.clients,
        debug_events_output=debug_events_output,
        fault=fault,
        fault_at_s=fault_at_s,
        fault_recovery_timeout_s=fault_recovery_timeout_s,
        typedb_container=typedb_container,
        typedb_stop_command=typedb_stop_command,
        typedb_start_command=typedb_start_command,
        typedb_restart_delay_s=typedb_restart_delay_s,
        fault_command_timeout_s=fault_command_timeout_s,
        lifecycle_change_state_service_name=(
            lifecycle_change_state_service_name
        ),
        lifecycle_get_state_service_name=lifecycle_get_state_service_name,
        lifecycle_reactivate=lifecycle_reactivate,
        lifecycle_transition_timeout_s=lifecycle_transition_timeout_s,
        setup_specs=setup_specs,
        teardown_specs=teardown_specs,
    )


def _load_external_profile(
    args: argparse.Namespace,
) -> StressProfile | None:
    """Load the external profile file if --profile-file is given."""
    profile_file = getattr(args, 'profile_file', None)
    if not profile_file:
        return None
    return load_stress_profile(Path(profile_file))


def _check_profile_conflicts(
    args: argparse.Namespace,
    profile: StressProfile | None,
) -> None:
    """Raise ValueError when profile file and CLI selectors define the same section."""
    if profile is None:
        return
    if (
        profile.invariant_specs is not None
        and args.invariant_profile != 'none'
    ):
        raise ValueError(
            '--profile-file defines invariants; remove --invariant-profile '
            'or set it to none'
        )
    if (
        profile.mixed_profile is not None
        and args.mode == 'mixed'
        and getattr(args, 'mixed_profile', 'auto') != 'auto'
    ):
        raise ValueError(
            '--profile-file defines a mixed section; remove --mixed-profile '
            'or leave it at auto'
        )


def _build_setup_specs(
    args: argparse.Namespace,
    profile: StressProfile | None,
) -> list[QuerySpec]:
    """Build setup specs from CLI flags and profile file."""
    specs: list[QuerySpec] = []

    schema_path = getattr(args, 'schema_path', None)
    if schema_path:
        specs.append(
            QuerySpec(
                query=Path(schema_path).read_text(encoding='utf-8'),
                query_type='define',
            )
        )

    data_path = getattr(args, 'data_path', None)
    if data_path:
        specs.append(
            QuerySpec(
                query=Path(data_path).read_text(encoding='utf-8'),
                query_type='insert',
            )
        )

    if profile is not None:
        specs.extend(profile.setup_specs)

    return specs


def _resolve_invariant_specs(
    args: argparse.Namespace,
    profile: StressProfile | None,
) -> list:
    """Profile invariants win; fall back to CLI --invariant-profile."""
    if profile is not None and profile.invariant_specs is not None:
        return profile.invariant_specs
    return build_invariant_specs(args)


def _resolve_mixed_profile(
    args: argparse.Namespace,
    profile: StressProfile | None,
) -> MixedQueryProfile | None:
    """Profile mixed section wins; fall back to CLI --mode mixed."""
    if profile is not None and profile.mixed_profile is not None:
        return profile.mixed_profile
    if args.mode == 'mixed' and args.query is None:
        return load_mixed_query_profile(args)
    return None


def _build_runtime_query_specs(
    args: argparse.Namespace,
    mixed_profile: MixedQueryProfile | None,
    profile: StressProfile | None,
) -> list[QuerySpec]:
    """Build fixed query specs or metadata specs for dynamic mixed mode."""
    if mixed_profile is not None:
        return [
            QuerySpec(query=template.query, query_type=template.query_type)
            for template in mixed_profile.templates
        ]

    if profile is not None and profile.read_query_specs is not None:
        return profile.read_query_specs

    return build_query_specs(args)
