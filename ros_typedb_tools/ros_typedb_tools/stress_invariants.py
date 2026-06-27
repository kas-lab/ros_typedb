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
"""Invariant evaluation helpers for ros_typedb stress experiments."""

from __future__ import annotations

import time
from typing import Any

from rcl_interfaces.msg import ParameterType

from ros_typedb_tools.stress_config import InvariantRecord
from ros_typedb_tools.stress_config import InvariantSpec


def _parameter_value_to_python(parameter_value: Any) -> Any:
    """Convert a ROS ParameterValue into the Python scalar it carries."""
    if parameter_value.type == ParameterType.PARAMETER_BOOL:
        return parameter_value.bool_value
    if parameter_value.type == ParameterType.PARAMETER_INTEGER:
        return parameter_value.integer_value
    if parameter_value.type == ParameterType.PARAMETER_DOUBLE:
        return parameter_value.double_value
    if parameter_value.type == ParameterType.PARAMETER_STRING:
        return parameter_value.string_value
    return None


def _extract_first_scalar(response: Any) -> Any:
    """Extract the first scalar attribute value from a Query response."""
    if not response.results:
        return None
    if not response.results[0].results:
        return None
    first_result = response.results[0].results[0]
    return _parameter_value_to_python(first_result.attribute.value)


def evaluate_invariant_response(
    *,
    index: int,
    phase: str,
    spec: InvariantSpec,
    started_at_s: float,
    ended_at_s: float,
    response: Any | None,
    timed_out: bool = False,
    exception_message: str = '',
) -> InvariantRecord:
    """Evaluate one invariant service response."""
    actual_value = _extract_first_scalar(response) if response else None
    error_parts = []

    if timed_out:
        error_parts.append('invariant request timed out')
    if exception_message:
        error_parts.append(exception_message)
    if response is None:
        error_parts.append('no response')
    elif not response.success:
        error_parts.append(
            response.error_message or 'service returned failure'
        )
    if actual_value != spec.expected_value:
        error_parts.append(
            f'expected {spec.expected_value!r}, got {actual_value!r}'
        )

    return InvariantRecord(
        index=index,
        phase=phase,
        name=spec.name,
        query_type=spec.query_type,
        query=spec.query,
        started_at_s=started_at_s,
        ended_at_s=ended_at_s,
        latency_s=ended_at_s - started_at_s,
        success=not error_parts,
        timed_out=timed_out,
        error_message='; '.join(error_parts),
        expected_value=spec.expected_value,
        actual_value=actual_value,
    )


def build_timeout_invariant_record(
    *,
    index: int,
    phase: str,
    spec: InvariantSpec,
    started_at_s: float,
) -> InvariantRecord:
    """Build an invariant record for a client-side timeout."""
    ended_at_s = time.monotonic()
    return evaluate_invariant_response(
        index=index,
        phase=phase,
        spec=spec,
        started_at_s=started_at_s,
        ended_at_s=ended_at_s,
        response=None,
        timed_out=True,
    )
