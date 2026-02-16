"""Structural check for the USDA subset written by tools/build_scene.py (no USD runtime needed)."""

import re

DEF_RE = re.compile(r'def(?:\s+(\w+))?\s+"(\w+)"')


def lint(text):
    """Return ``(prim_paths, targets, errors)``.

    Checks that strings, asset paths and target paths are terminated, that brackets balance, that
    every prim body belongs to a ``def`` and that sibling prims have unique names.
    """
    errors, paths, targets = [], set(), []
    stack, prims, pending = [], [], None
    i, n = 0, len(text)
    while i < n:
        c = text[i]
        if text.startswith('"""', i):
            end = text.find('"""', i + 3)
            if end < 0:
                errors.append(f"unterminated triple-quoted string at {i}")
                break
            i = end + 3
            continue
        if c in '"@<':
            closing = {'"': '"', "@": "@", "<": ">"}[c]
            end = text.find(closing, i + 1)
            if end < 0 or "\n" in text[i:end]:
                errors.append(f"unterminated {c} at {i}")
                break
            if c == "<":
                targets.append(text[i + 1:end])
            i = end + 1
            continue
        if c == "#":
            end = text.find("\n", i)
            i = n if end < 0 else end
            continue
        if text.startswith("def", i) and (i == 0 or text[i - 1] in " \t\n"):
            match = DEF_RE.match(text, i)
            if match:
                pending = match.group(2)
                i = match.end()
                continue
        if c in "([{":
            name = None
            if c == "{" and not any(kind in "([" for kind, _ in stack):
                if pending is None:
                    errors.append(f"prim body without def at {i}")
                name, pending = pending, None
                path = "/" + "/".join(prims + [name])
                if path in paths:
                    errors.append(f"duplicate prim {path}")
                paths.add(path)
                prims.append(name)
            stack.append((c, name))
        elif c in ")]}":
            if not stack or stack[-1][0] != {")": "(", "]": "[", "}": "{"}[c]:
                errors.append(f"unbalanced {c} at {i}")
                break
            _, name = stack.pop()
            if name is not None:
                prims.pop()
        i += 1
    if stack:
        errors.append(f"unclosed brackets: {[kind for kind, _ in stack]}")
    if pending is not None:
        errors.append(f"def {pending!r} has no body")
    return paths, targets, errors
