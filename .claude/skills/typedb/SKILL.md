---
name: typedb
description: Write and debug TypeQL queries for TypeDB 3.8+. Use when working with TypeDB schemas, data queries, insertions, deletions, or functions.
---

# TypeDB 3 Reference

Adapted from `CaliLuke/skills` for local use in this repository.

## Schema roots

- `entity`
- `relation`
- `attribute`

TypeDB 3 does not support `thing` as a schema root.

## Core schema syntax

```typeql
define
  attribute name, value string;
  relation employment, relates employer, relates employee;
  entity person, owns name, plays employment:employee;
```

## TypeDB 3 functions

TypeDB 3 uses `fun`, not `rule`.

```typeql
fun get_active_users() -> { string }:
  match
    $u isa user, has status "active", has email $e;
  return { $e };

fun count_users() -> integer:
  match
    $u isa user;
  return count;

fun user_exists($email: string) -> boolean:
  match
    $u isa user, has email == $email;
  return check;
```

## Using functions

```typeql
match
  let $emails in get_active_users();
fetch { "active_emails": $emails };

match
  let $count = count_users();
fetch { "total_users": $count };

match
  true == user_exists("alice@example.com");
fetch {};
```

## Relation syntax

Anonymous relation:

```typeql
employment (employer: $c, employee: $p);
```

Relation variable:

```typeql
$rel isa employment;
$rel links (employer: $c, employee: $p);
```

Do not write:

```typeql
$rel (employer: $c, employee: $p) isa employment;
```

## Common query operators

- `match`
- `fetch`
- `insert`
- `put`
- `update`
- `delete`
- `sort`
- `offset`
- `limit`
- `reduce`
- `distinct`
- `require`
- `try`
- `not`
- `or`

## Common pitfalls

### No rules in TypeDB 3
- Do not add `rule ... when ... then ...` blocks.

### Exact type vs subtype matching
```typeql
$x isa person;   # includes subtypes
$x isa! person;  # exact type only
```

### Variable reuse in OR branches
- Use unique variable names per branch.

### Aggregates
```typeql
return max($m);
return min($m);
```

Parentheses are required.

### Boolean returns
```typeql
let $r = true;
return first $r;
```

## TypeDB 3 patterns used in this repo

- Prefer `fun` composition over duplicated query fragments.
- Keep constraints in the same `match` stage where possible.
- For repo-specific TypeDB guidance, also consult:
  - `.claude/skills/new-typedb-fun/SKILL.md`
  - `.claude/skills/typeql-preflight/SKILL.md`
