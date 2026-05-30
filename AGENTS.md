# Agent Guidance

This repository also has `CLAUDE.md` files. Treat them as active project
guidance rather than Claude-only notes:

- Read root `CLAUDE.md` for project context.
- Read `cad/CLAUDE.md` before changing any CAD module.
- For CAD files that import `cad_khana` or use `Assembly()`, `check()`, or
  `inspect()`, read `.claude/skills/cad-khana/SKILL.md` before editing or
  running checks.

CAD verification:

- Prefer `khana check` during iteration.
- Run `khana` from the directory containing the target `assembly.py` so
  `outputs/` is written beside that assembly.
- Use the project virtualenv if `khana` is not on `PATH`, for example
  `../../.venv/bin/khana check assembly.py` from `cad/m03_scanner/`.

Commit message drafts:

- Follow the global convention from `~/.claude/CLAUDE.md`: single-line
  conventional subject, blank line, co-author trailer; body only when it
  genuinely adds context.
- For Codex CLI work, use the exact active model number in the
  co-author trailer when known. Example:
  `Co-authored-by: GPT-5.5 Codex <gpt-5.5-codex@codex-cli>`.
- Do not guess the model number. If the exact Codex model is not exposed
  by the session or provided by the user, ask before drafting the
  co-author trailer.
