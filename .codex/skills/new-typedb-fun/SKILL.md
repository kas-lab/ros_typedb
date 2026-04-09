---
name: new-typedb-fun
description: Use when defining a new TypeDB 3 `fun` function in a schema file. Provides correct boilerplate and inlines the constraints that cause the most common parse and type errors.
---

# New TypeDB 3 `fun` Function

## Boilerplate Templates

### Set-returning function
```typeql
fun function_name($arg: ArgType) -> { ReturnType }:
  match
    {
      # branch A — unique variable names: $a_x, $a_y, ...
    } or {
      # branch B — unique variable names: $b_x, $b_y, ...
    };
  return { $result };
```

### Single-value function
```typeql
fun function_name($arg: ArgType) -> ReturnType:
  match
    $result isa ReturnType;
    # ...
  return first $result;
```

### Aggregate (count / max / min)
```typeql
fun function_name($arg: ArgType) -> integer:
  match
    # ...
  return count;

fun function_name($arg: ArgType) -> double:
  match
    $val isa ...; # ...
  return max($val);
```

## Critical Constraints

### Variable naming
- Each OR branch must use unique variable names.
- Never name a variable after a type or role.

### Relation syntax inside `fun`
```typeql
# CORRECT
$r isa some_relation;
$r links (role: $x);

# WRONG
$r (role: $x) isa some_relation;
```

### Function-call results across OR branches
```typeql
# WRONG
{ let $out = some_fun($a); } or { let $out = some_fun($b); }

# CORRECT
{ let $tmp = some_fun($a); let $out = $tmp; } or { let $tmp2 = some_fun($b); let $out = $tmp2; }
```

### Recursion
- Mutual recursion between two `fun` functions is not supported.
- Self-recursion is allowed.

### Hoisting constraints
- `links` constraints can be hoisted before the outer `{ } or { }` block.
- `isa` constraints before the outer OR block cause parse errors and should stay inside branches.

### No `rule` blocks
- TypeDB 3 does not support `rule` in `define`. Use `fun` only.

### `plays` role-player type checking
- Every `isa SomeType` in an OR branch must be a valid player of the matched role.

## OR Branch Ordering

For batch / all-rules queries, put the branch matching the most inputs first.

For `return first` functions, early-exit applies.
For `return min` / `return max`, all branches are evaluated, so ordering has no early-exit benefit.

## Calling a `fun` in a query
```typeql
# Set-returning: iterate results
let $result in function_name($arg);

# Single-value: bind and use
let $val = function_name($arg);
$val > 0;

# Boolean guard pattern
true == bool_function($arg);
```
