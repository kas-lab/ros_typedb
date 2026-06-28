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

from ros_typedb_tools.stress_config import build_invariant_specs
from ros_typedb_tools.stress_config import build_query_specs
from ros_typedb_tools.stress_config import DEFAULT_TYPEDB_START_COMMAND
from ros_typedb_tools.stress_config import DEFAULT_TYPEDB_STOP_COMMAND
from ros_typedb_tools.stress_config import QuerySpec
from ros_typedb_tools.stress_config import ResolvedStressConfig
from ros_typedb_tools.stress_config import validate_experiment_args
from ros_typedb_tools.stress_mixed import load_mixed_query_profile
from ros_typedb_tools.stress_mixed import MixedQueryProfile


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

    invariant_specs = build_invariant_specs(args)
    mixed_profile = _resolve_dynamic_mixed_profile(args)
    query_specs = _build_runtime_query_specs(args, mixed_profile)
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
    )


def _resolve_dynamic_mixed_profile(
    args: argparse.Namespace,
) -> MixedQueryProfile | None:
    """Load a mixed profile only when the workload is profile-driven."""
    if args.mode == 'mixed' and args.query is None:
        return load_mixed_query_profile(args)
    return None


def _build_runtime_query_specs(
    args: argparse.Namespace,
    mixed_profile: MixedQueryProfile | None,
) -> list[QuerySpec]:
    """Build fixed query specs or metadata specs for dynamic mixed mode."""
    if mixed_profile is None:
        return build_query_specs(args)

    # Dynamic mixed requests are rendered per client and request index at
    # runtime. These QuerySpecs preserve the selected profile's query mix for
    # JSON output and debug readability.
    return [
        QuerySpec(query=template.query, query_type=template.query_type)
        for template in mixed_profile.templates
    ]
