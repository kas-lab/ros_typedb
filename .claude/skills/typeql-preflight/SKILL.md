---
name: typeql-preflight
description: Pre-flight checklist for TypeDB 3 schema edits in the typedb_tactics package. Invoke before writing or editing any .tql schema file to catch known error patterns before running Docker tests.
user-invocable: false
---

Before editing or writing a TypeQL schema file, run through this checklist and flag any violations:

## Syntax constraints (will cause parse/commit errors)

1. **No `rule` blocks** — TypeDB 3 removed rules. Use `fun` with `with fun name($arg: Type) -> ReturnType:` syntax. Violation: `[TSV7]`
2. **Relation variable syntax** — use `$r isa T, links (role: $x)` — NOT `$r (role: $x) isa T`. The latter triggers `[REP1]` even inside `fun` match bodies.
3. **No dual Object+ThingType declaration** — in a single INSERT block, you cannot create a named relation variable `$r` and also use it as a role player in the same block. Make intermediate relations anonymous. Violation: `[REP1]`
4. **No variable reuse across pipeline stages for different types** — if `$e` was bound as `propositional_expression` in stage 1, do not reuse `$e` for `not_expression` in stage 2. Use unique names per stage. Violation: `[INF11]`
5. **Return syntax** — `return max($m);` and `return min($m);` require parentheses. `return max $m;` is invalid.
6. **Boolean returns** — `return true;` is invalid. Use `let $r = true; return first $r;`
7. **`-> boolean` not `-> bool`** — use `-> boolean` in function signatures. `bool` causes `[INF2]`.

## Function body constraints

8. **Cross-OR-branch function call results** — cannot bind a function-call result directly to a variable that appears across OR branches. Use double-let: `let $tmp = func(...); let $out = $tmp;`. Violation: `[CEX18]`
9. **OR branch `isa` type guards** — only include `isa` branches for types that actually play the role being matched in that `fun`. If a type cannot play the role, TypeDB raises `[FIN0]`/`[QUA1]` at schema commit.
10. **No mutual recursion between `fun` bodies** — the function call graph must be a DAG. Violation: `[FUN9]`
11. **`links` can be hoisted before `{ } or { }` blocks** — valid and faster. `isa` constraints before the outer OR block cause a parse error — keep those inside branches.

## Performance (apply when writing new functions)

12. **OR branch ordering** — for `return first` functions, put the branch matching the MOST inputs first. For batch/all-rules queries this is the dominant performance lever (confirmed −37% gain in this codebase).
13. **OR branch ordering does NOT help `return min`/`return max`** — TypeDB evaluates all branches for aggregators; early-exit does not apply.
14. **Keep OR branches inline** — do not split into separate functions for dispatch. Cross-function overhead is 147× in TypeDB 3 (confirmed).
15. **One `match` stage** — the optimizer only reorders within a single stage. Splitting into two stages disables cross-constraint optimization.

## Before committing

- Check `src/typedb_tactics/agent-docs/schema-guide.md` for module intent before adding new types.
- Run tests in Docker: `scripts/run-tests-docker.sh`
- If adding a new `fun`, ensure every OR branch has at least one test case that hits it.
- Use text2typeql examples as a reference: `python3 scripts/find-text2typeql-examples.py "query intent" --top-k 5`
