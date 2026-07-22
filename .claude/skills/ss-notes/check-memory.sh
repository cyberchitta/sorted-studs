#!/usr/bin/env bash
# Memory-frontmatter checks for the sorted-studs auto-memory corpus.
#
# Two invariants, both of which went red cold on 2026-07-21 (13/16 drifted
# slugs, 6 dangling wikilinks) and which neither prose-reading nor a citation
# sweep can see:
#   1. every file's `name:` slug equals its basename — wikilinks resolve
#      against the slug, so a drifted slug silently breaks every [[link]] to it
#   2. every [[wikilink]] resolves to a memory file
#
# Uses /usr/bin/grep by ABSOLUTE PATH on purpose: Claude Code defines `grep`
# and `rg` as shell functions in its zsh snapshot, and neither exists inside a
# bash script — a bare `grep`/`rg` here resolves to nothing and exits quiet,
# so the check would report clean having read zero files. See the global
# CLAUDE.md rule "A *script* cannot use `rg` at all".
#
# A green run proves nothing until you have watched it go red: add a bogus
# [[link]] or misspell a `name:`, confirm this reports it, then revert.
set -euo pipefail

MEM="${1:-$HOME/.claude/projects/-Users-cyberchitta-GitHub-sorted-studs/memory}"
cd -P "$MEM"

rc=0

# 1. name: must equal the basename
for f in *.md; do
  [ "$f" = MEMORY.md ] && continue
  b="${f%.md}"
  n=$(/usr/bin/grep -m1 '^name:' "$f" | sed 's/^name:[[:space:]]*//;s/[[:space:]]*$//')
  if [ "$n" != "$b" ]; then
    echo "SLUG   $f -> name: '$n'"
    rc=1
  fi
done

# 2. every [[wikilink]] must resolve to a file
/usr/bin/grep -oh '\[\[[^]]*\]\]' *.md | tr -d '[]' | sort -u |
  while read -r l; do
    [ -z "$l" ] && continue
    if [ ! -f "$l.md" ]; then
      echo "DANGLING [[$l]]"
      rc=1
    fi
  done

# the pipe above runs in a subshell; recompute rc from a marker instead
if /usr/bin/grep -oh '\[\[[^]]*\]\]' *.md | tr -d '[]' | sort -u |
     while read -r l; do [ -z "$l" ] && continue; [ -f "$l.md" ] || exit 1; done
then :; else rc=1; fi

[ "$rc" = 0 ] && echo "check-memory: clean"
exit "$rc"
