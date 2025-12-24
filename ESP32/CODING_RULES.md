CODING_RULES
=============

This document contains the coding rules for the `ESP32/` firmware subtree. All rules are written in English and must be followed by contributors and automated tools.

**1. Language and OpenSpec**
- All source code, comments, OpenSpec files, proposals, specs, design documents, and committed text MUST be written in English only.
- Chat and interactive discussions may occur in other languages, but committed files and code must remain English to ensure maintainability and consistency.

**2. Code formatting and style**
- Avoid single-line function bodies, lambda bodies, or compressing complex statements on one line. Use multi-line blocks for readability.
- Use consistent indentation (spaces preferred). Keep logical spaces around operators and after commas.
- Place statements on separate lines. End statements with a semicolon and a newline.
- Do not chain multiple statements on a single line.
- Opening and closing braces should be on their own lines or follow the projects existing brace style consistently.
- Prefer explicit blocks for control statements (if/for/while/switch) — always use braces even for single statements.

**Namespace usage guidance**
- Prefer grouping related definitions inside a single `namespace` block per translation unit (source file). While C++ allows reopening the same namespace multiple times, grouping improves readability and reduces surprising initialization/order issues.
- If reopening a namespace within the same file is required (for example to separate tests or clearly distinct logical sections), include a brief comment explaining the reason.

**3. Statement layout and blocks**
- Keep functions concise and single-purpose. If a function grows beyond a few dozen lines, consider extracting responsibilities into helper functions or separate modules.
- Avoid writing return or other logic as a single-line expression across the project. Example to avoid:

  // Bad (avoid single-line body)
  int f() { return computeValue(); }

  // Good
  int f() {
    return computeValue();
  }

**4. SOLID and architecture**
- Single Responsibility: Each module, class, and function should have one clear responsibility. If a file contains multiple unrelated responsibilities, split it.
- Open/Closed: Design components to be open for extension but closed for modification. Prefer composition and abstractions.
- Liskov Substitution: Derived types must be substitutable for base types. Keep interface contracts stable.
- Interface Segregation: Avoid large general-purpose interfaces. Provide small, focused interfaces tailored to consumers.
- Dependency Inversion: High-level modules should depend on abstractions (interfaces), not concrete implementations.

Practical guidance:
- When adding more than a few hundred lines or multiple top-level functions, extract responsibilities into separate files and modules.
- Avoid adding unrelated helper functions into an existing file; prefer new well-named source/header pairs.

**5. Naming and symbols**
- Use descriptive names; avoid one-letter variable names (except common loop indices like `i`, `j` when local and trivial).
- Wrap file paths, filenames, and code identifiers in backticks in documentation.

Clarifications — naming and guard braces
- Prefer long, descriptive names that communicate the entity and role. Examples: `telemetryManager`, `leftMotorDriver`, `motorTelemetryTask`.
- Avoid unclear abbreviations such as `mgr`, `impl`, `i`, `tmp`, except for trivial local loop indices (`i`, `j`).
- When interacting with FreeRTOS tasks or passing `void *pv` pointers, cast into a clearly-named pointer variable. For example:

  // Good
  MotorTelemetryManager::Impl *impl_ptr = static_cast<MotorTelemetryManager::Impl *>(pv);

  // Bad
  Impl *i = static_cast<Impl *>(pv);

- Always use braces for conditional guards and early returns to make intent explicit and avoid accidental dangling statements. For example:

  // Good: explicit guard with braces
  if (!telemetryImpl) {
    return;
  }

  // Bad: single-line guard without braces (forbidden)
  if (!telemetryImpl) return;

These clarifications aim to improve readability during reviews and reduce bugs caused by ambiguous identifiers or missing braces.

**6. Comments and commit messages**
- Write comments in clear English. Use comments to explain "why" not "what" where the code is self-explanatory.
- Keep commit messages short and descriptive; follow the repositorys existing conventions.

**7. Tests and validation**
- Add unit tests for new interfaces and public behaviors where practical (host tests allowed under `UNIT_TEST_HOST`).
- Run a build and basic tests after changes that touch core interfaces.

**8. Migration and refactor rules**
- When migrating APIs or configurations, prefer incremental changes with small commits/PRs and frequent builds.
- Keep a temporary compatibility shim if necessary during migration, but remove it when migration is complete and all call-sites have been updated.

**9. Documentation**
- Update `ESP32/README.md` (or appropriate module README) when changes introduce new configuration locations, public APIs, or build steps.

---

If you want, I can add a pre-commit hook or an editorconfig snippet to help enforce the indentation and newline rules automatically.