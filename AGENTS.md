<!-- OPENSPEC:START -->
# OpenSpec Instructions

These instructions are for AI assistants working in this project.

Always open `@/openspec/AGENTS.md` when the request:
- Mentions planning or proposals (words like proposal, spec, change, plan)
- Introduces new capabilities, breaking changes, architecture shifts, or big performance/security work
- Sounds ambiguous and you need the authoritative spec before coding


## Language Policy

All code and OpenSpec files (proposals, specs, design, tasks, documentation, etc.) MUST be written in **English** only. This ensures consistency, maintainability, and ease of collaboration for all contributors.

Chat and interactive discussions with the AI assistant may continue in French if desired, but all committed files and code must remain in English.

Use `@/openspec/AGENTS.md` to learn:
- How to create and apply change proposals
- Spec format and conventions
- Project structure and guidelines

Keep this managed block so 'openspec update' can refresh the instructions.

<!-- OPENSPEC:END -->

## Coding Rules

Coding rules (formatting, SOLID design guidance, language policy and related guidelines) have been consolidated into a dedicated document for the firmware subtree.

Refer to: `ESP32/CODING_RULES.md` for the authoritative, project-level coding rules. All contributors and assistants must follow the rules documented there when modifying code or OpenSpec files.

If you make changes to coding rules, update `ESP32/CODING_RULES.md` and keep the OpenSpec block above unchanged so tooling can refresh instructions.