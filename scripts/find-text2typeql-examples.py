#!/usr/bin/env python3
# Copyright 2026 KAS Lab
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
"""Find relevant TypeQL examples from the local text2typeql dataset."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
import re
from typing import Any


def _default_dataset_file() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / 'agent-data'
        / 'text2typeql'
        / 'dataset'
        / 'all_queries.csv'
    )


def _tokenize(text: str) -> set[str]:
    return set(re.findall(r'[a-z0-9_]+', text.lower()))


def _record_source_file(base_dataset_dir: Path, source: str, domain: str) -> str:
    record_file = base_dataset_dir / source / domain / 'queries.csv'
    if record_file.exists():
        return str(record_file)
    return str(base_dataset_dir / 'all_queries.csv')


def _score_record(question: str, typeql: str, query_tokens: set[str], query_lower: str) -> int:
    haystack = '{}\n{}'.format(question, typeql)
    haystack_tokens = _tokenize(haystack)
    token_overlap = len(query_tokens & haystack_tokens)
    exact_phrase_bonus = 3 if query_lower and query_lower in haystack.lower() else 0
    return token_overlap + exact_phrase_bonus


def _load_records(dataset_file: Path) -> list[dict[str, str]]:
    with dataset_file.open(newline='', encoding='utf-8') as csv_file:
        return list(csv.DictReader(csv_file))


def find_examples(
    dataset_file: Path,
    query: str,
    top_k: int,
    domain: str | None,
    source: str | None,
    regex_pattern: str | None,
    min_score: int,
) -> list[dict[str, Any]]:
    rows = _load_records(dataset_file)
    query_tokens = _tokenize(query)
    query_lower = query.strip().lower()
    regex = re.compile(regex_pattern, re.IGNORECASE) if regex_pattern else None
    dataset_dir = dataset_file.parent

    candidates: list[tuple[int, dict[str, Any]]] = []
    for line_index, row in enumerate(rows, start=2):
        row_domain = row.get('domain', '').strip()
        row_source = row.get('source', '').strip()
        row_question = row.get('question', '').strip()
        row_typeql = row.get('typeql', '').strip()
        row_id = row.get('original_index', '').strip() or str(line_index - 2)

        if domain and row_domain != domain:
            continue
        if source and row_source != source:
            continue

        haystack = '{}\n{}'.format(row_question, row_typeql)
        if regex and not regex.search(haystack):
            continue

        score = _score_record(row_question, row_typeql, query_tokens, query_lower)
        if score < min_score:
            continue

        source_file = _record_source_file(dataset_dir, row_source, row_domain)
        result = {
            'domain': row_domain,
            'english': row_question,
            'typeql': row_typeql,
            'source_file': source_file,
            'line': line_index,
            'record_id': '{}:{}:{}:{}'.format(
                dataset_file.name, row_source, row_domain, row_id
            ),
            'score': score,
        }
        candidates.append((score, result))

    candidates.sort(key=lambda item: item[0], reverse=True)
    return [item[1] for item in candidates[:top_k]]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Search local text2typeql examples by keyword/regex.'
    )
    parser.add_argument(
        'query',
        nargs='?',
        default='',
        help='Natural language query or keywords to match.',
    )
    parser.add_argument(
        '--dataset-file',
        type=Path,
        default=_default_dataset_file(),
        help='Path to CSV dataset file. Default: agent-data/text2typeql/dataset/all_queries.csv',
    )
    parser.add_argument(
        '--top-k',
        type=int,
        default=5,
        help='Maximum number of results to print.',
    )
    parser.add_argument(
        '--domain',
        default=None,
        help='Optional domain filter (e.g., movies, companies, northwind).',
    )
    parser.add_argument(
        '--source',
        default=None,
        help='Optional source filter (e.g., synthetic-1, synthetic-2).',
    )
    parser.add_argument(
        '--regex',
        default=None,
        help='Optional regex filter applied to English and TypeQL text.',
    )
    parser.add_argument(
        '--min-score',
        type=int,
        default=2,
        help='Minimum relevance score required for returned records.',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.dataset_file.exists():
        raise FileNotFoundError(
            'Dataset file not found: {}'.format(args.dataset_file)
        )

    examples = find_examples(
        dataset_file=args.dataset_file,
        query=args.query,
        top_k=args.top_k,
        domain=args.domain,
        source=args.source,
        regex_pattern=args.regex,
        min_score=args.min_score,
    )

    print(json.dumps(examples, indent=2, ensure_ascii=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
