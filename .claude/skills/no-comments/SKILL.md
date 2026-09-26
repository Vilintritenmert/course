---
name: no-comments
description: Use whenever writing or editing code (any language) in this project. Enforces not adding inline/block comments to code — write self-explanatory code instead. Trigger on any code-writing or code-editing task.
---

# No Comments

When writing or editing code in this project, do not add comments — no `//`, `/* */`, `#`, docstrings-as-narration, or any other comment syntax explaining what the code does.

## Rules

- Do not add explanatory comments to new code you write.
- Do not add comments to existing code you edit, even if the surrounding code already has some — don't copy that pattern into new code.
- Prefer clear naming (functions, variables, types) over comments to convey intent.
- Do not remove pre-existing comments unless the user asks or the edit makes them obsolete (e.g. the commented code is deleted).
- Exceptions where a comment is acceptable only if truly necessary and requested implicitly by context:
  - Required legal/license headers.
  - Directives tools depend on (e.g. `// eslint-disable-next-line`, `# noqa`, `// NOLINT`, pragma/compiler directives).
  - `TODO`/`FIXME` markers the user explicitly asks for.
- If tempted to write a comment to explain "why", instead refactor (extract a well-named function/variable) so the code is self-explanatory. If that's not possible, keep the comment minimal and only for genuinely non-obvious rationale (e.g. a workaround for a specific bug/platform quirk) — never for describing "what" the code does.
