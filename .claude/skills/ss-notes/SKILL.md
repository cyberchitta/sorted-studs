---
name: ss-notes
description: >-
  How to write, update, and retire sorted-studs' working notes in `_notes/` — the
  per-module `state/` files, `decisions/`, aspirational `specs/`, `backlog/`,
  `handoff/`, `lessons.md`, `explorers/`, and `archive/` that carry CAD design state
  between sessions. Covers what content belongs in which kind (and what belongs in a
  `params.py` docstring, `cad/CLAUDE.md`, or cad-khana instead, so a reader standing
  at the code never misses it), the lessons→`cad/CLAUDE.md`→cad-khana promotion
  pipeline, the archive gate that extracts durable value before filing, and the
  memory-frontmatter housekeeping pass. Invoke before writing/updating a state file
  or handoff, when a note's claims look stale, when deciding where a hard-won CAD
  fact should live, or before moving/archiving/deleting anything under `_notes/`.
---

# Working notes — sorted-studs' delta

`_notes/` is a gitignored symlink into the private `working-notes` repo. In this
repo it is **AI-maintained design state**: updating a `state/` file, correcting a
stale claim, or archiving a finished note is ordinary CAD work, done in the same
slice as the geometry — `CLAUDE.md` requires the state update to land *with* the
change, not after.

The shared `_notes/` mechanism — why `Grep`/`Glob` find nothing here, the `rg -L
--no-ignore-vcs` forms that do, the periodic commit cadence, structural moves
committed immediately — is the **`working-notes` skill**. Load it; do not restate
it here. This file is only what sorted-studs adds on top: a CAD corpus organised
by **genre folder + module filename-prefix, depth 2** (flattened 2026-07-21 from
a source-tree mirror).

## First question: does this belong in `_notes/` at all?

Route by **where the reader is standing when they need it** — not by how permanent
it feels. A CAD constraint filed in a note is a constraint the next session won't
see, because the next session is looking at the assembly, not the note.

| Who must not miss it, and where they are | Where it goes |
|---|---|
| Someone editing *this constant or part* | a docstring **at that seam** in the owning `params.py` / `assembly.py` |
| Someone writing any CAD in *this repo* | `cad/CLAUDE.md` |
| Someone resuming *this task* | the task's `handoff/<topic>.md` |
| Someone picking up *this module's design* | `state/<module>.md` |
| Someone asking "why is it built this way at all?" | `decisions/<module>-<topic>.md` |
| A cross-cutting CAD gotcha that should fire *every* session | `lessons.md`, then promote (pipeline below) |
| Any future session, on any task | a memory file + `MEMORY.md` line |

The load-bearing row is the first. A rule about a specific dimension, frame, or
part belongs **in the code that owns it** — a `params.py` docstring or an
`assembly.py` comment at the seam — because the person about to violate it is
reading `params.py`, not your note. When a handoff or state file teaches you
something that binds future geometry, **that note is not where it ends up**:
promote it and say in the note where it went.

**Read what the destination already says first — a half-right neighbour is worse
than silence.** `cad/CLAUDE.md` principle 2 documents the standalone-mirror idiom;
the *stale*-mirror trap had to be added *at* principle 2 to correct the impression
it left, not appended elsewhere. Skim the surrounding section for the sentence
that will now be read against your addition.

### Which layer of the code, specifically

CAD knowledge splits three ways once you've decided it isn't a note:

- **A `params.py` docstring** — a value and *why it is that value*: a frame
  anchor, an interface mirror, a clearance that a specific geometry forces. Per
  `cad/CLAUDE.md` principle 5, constants live at their owning scope; the docstring
  travels with the constant. Cite the `decisions/` note here when the rationale is
  longer than a docstring holds — that citation is why decision paths are stable.
- **`cad/CLAUDE.md`** — a rule that binds *any* CAD in the repo: a coordinate
  convention, a verification habit, a decomposition principle. Always loaded, so
  every line costs context — put a rule here only once it has bitten.
- **cad-khana** (`../cad-khana/_notes/field-notes.md`, then its `SKILL.md`) — a
  gap in the *library*, not this repo's use of it. See the pipeline below.

## The kinds, and how each is maintained

The kinds differ in **lifetime** and **what triggers a touch**. Maintaining one
as the wrong kind — a living state file kept as a journal, an immutable decision
corrected in place with no dated note — is how a corpus rots.

| Kind | Path | Lifetime | Touch it when |
|---|---|---|---|
| State | `state/<module>.md` | **living, indefinite** | same slice as the geometry |
| Handoff | `handoff/<topic>.md` | living until the task closes → `archive/` | same turn as the code |
| Decision | `decisions/<module>-<topic>.md` | permanent; **path stable, never moved** | a revisit-trigger fires |
| Spec | `specs/<module>-<name>.md` | **aspirational** (see below) | a real spec is written back to public `docs/` |
| Backlog | `backlog/<module>.md` | living | an item is added, promoted, or dropped |
| Lessons | `lessons.md` | staging | a gotcha surfaces; drain per pipeline |
| Explorer | `explorers/*.html` | living artifact | the geometry it visualises changes |
| Archive | `archive/<module>-<topic>.md` | closed, kept | only via the gate below |
| Worklist | `worklist.md` | living | navigation only; never a second copy of a state item |

### State — the distinctive kind, and why it is not a handoff

`state/<module>.md` is a **per-module current-state document that lives
indefinitely.** A module doesn't "close" the way a task does — its design keeps
evolving — so unlike a handoff, a state file is never archived; it is edited
forever. Do not collapse the two: a handoff is per-*task* and retires; a state
file is per-*module* and persists.

Its job: a session that has never seen this module reads it and continues
correctly. So:

- **Correct in place; never layer.** If a line is no longer true, *fix the line*.
  Do not add a newer paragraph below a stale one and leave both — a reader can't
  tell which is current, and the file becomes archaeology. This is the single most
  common way these notes go bad.
- **Record what was rejected and why**, so the next session doesn't re-litigate a
  settled trade. A `## What we deliberately are NOT doing` section with root
  causes is the m05 model.
- **Name the seams** — the `params.py` constants, the `assembly.py` checks, the
  parts the next session will touch — so it doesn't re-derive them by search.
- **Keep open questions phrased as questions.** An unresolved thing written as an
  assertion gets built on.

**The m05 shape is the target.** `state/m05-diverter.md` solved this and is the
model — three moves the weaker files lack:

1. History offloaded to `archive/m05-session-history.md`, so the state file stays
   mostly forward-looking instead of a done-log.
2. A named `## What we deliberately are NOT doing` section with root causes.
3. An explicit `## Lessons — don't-repeat` heading — so its lessons are visible,
   not buried inside ✅-marked completed items where a done-log prune deletes them.

`state/m03-scanner.md` did move 1 and stopped; its stale tail self-flags as
knowingly wrong. Rebuilding it to the m05 shape is deferred work, not a licence to
copy its shape.

### Specs are aspirational, not descriptive

`specs/` holds the former `docs/` module specs, moved out of the code repo
2026-07-21. **They changed faster than the design did** — working notes wearing a
spec's clothes; 11 of 16 carry unresolved TBD/TODO markers. They are **not a
source of truth**: check any spec claim against the module's `state/` file and the
CAD before trusting it. Accurate specs get written back into a public `docs/` per
module *as each design settles* — do not treat the `specs/` file as that
deliverable.

### Decisions never move

Durable rationale that outlives every task touching it: what was decided, what was
rejected, and the revisit-triggers that reopen it. These are **cited from
`params.py` docstrings**, so their paths are stable — amend a decision in place
(dated) when a trigger fires; never relocate the file, because the docstring
pointing at it won't follow.

## The lessons → `cad/CLAUDE.md` → cad-khana promotion pipeline

`lessons.md` is a **staging area**, not a destination. A gotcha lands there first,
then drains outward by audience:

1. **Filter at entry:** *would this have prevented a bug that actually happened in
   this repo?* If not, it doesn't belong in an always-loaded file — leave it in
   `lessons.md` or drop it. Speculative guidance ages badly and costs context
   every session.
2. **Drains to `cad/CLAUDE.md`** when the lesson binds any CAD in *this* repo and
   passed the filter — routed to *where the reader stands when the trap fires*
   (the stale-mirror trap went *at* principle 2, not into a lessons appendix).
3. **Drains further to cad-khana** when the gap is in the *library*: log a
   one-liner in `../cad-khana/_notes/field-notes.md` with a concrete example, and
   only once 2–3 observations point at the same pattern promote it into cad-khana's
   `SKILL.md`. The threshold and mechanism are `cad/CLAUDE.md` §"Feeding
   observations back to cad-khana" — follow it there; don't restate it.

`lessons.md` carries a **"Promoted — do not re-promote"** list so a later pass
doesn't duplicate a rule already drained.

## The archive gate

`archive/README.md` is the gate's home — read it before archiving or deleting.
Archiving is extraction *followed by* filing; all of these hold, and if you can't
satisfy the deletable standard, the note stays where it is.

1. **Closed** — the work shipped, was abandoned, or was superseded *in those
   words*.
2. **Durable value extracted, and the archive stays deletable.** The testable
   standard: *deleting `archive/` outright, at any moment, must lose nothing.* If
   deleting it would lose a shop dimension, a torque check, a frame convention —
   that thing was filed too early. Route each surviving claim per the table at the
   top; the blow-by-blow *stays* in the note, that's what the archive is for.
3. **Header rewritten** to `**Archived <YYYY-MM-DD>** — <outcome>. Durable value →
   <where it went>.`
4. **Inbound citations repaired** in the same pass (see below).

Two rules this corpus learned the hard way (2026-07-21) and that the gate now
enforces:

- **"Superseded" is not "redundant."** A file earns deletion when its *content*
  lives elsewhere — not when the thing it *describes* stopped existing. The
  archive's entire job is to hold the second category so the delete decision can
  wait. Two files were deleted as "duplication" and restored the same session
  because they were unique content whose *mechanism* was merely moot.
- **Distillation is checked per-claim, not per-file.** "We distilled this note" is
  a claim about the parts someone looked at. A kicker handoff whose *lesson* was
  lifted and *rationale* folded still held a live tilt-torque constraint nobody had
  noticed. Walk the substantive claims and grep each into the code; work
  **claim-side, not citation-side** — the dangerous fact is the one whose code site
  carries no pointer back, which a citation-walk is structurally blind to.
- **Sequence it right:** flatten first, delete later. A file that still looks dead
  once it's legible at depth 2 is a far safer delete than one judged while the
  corpus is hard to see. The cost asymmetry is the whole argument — keeping dead
  lines costs nothing anyone notices; re-deriving a lost dimension costs a session.

**A handoff's wrong turns belong wherever its right ones were distilled to.** The
archive is authoritative over the handoffs: where they conflict,
`session-history` records that a handoff *proposal* was wrong in practice
(`PIVOT_X = -50` crashed the hub housing; −110 was kept). A citation to an
archived handoff must carry that warning, because the reader who follows the
citation is exactly the reader at risk.

## Citations: not optional

Every move or delete breaks inbound references — from other notes, from the
`MEMORY.md` index, and in this repo **from `params.py` docstrings, `cad/CLAUDE.md`,
and `README.md`** (`decisions/` and `specs/` paths are cited from tracked code). A
dangling citation is worse than a stale note: it sends a session hunting for
context that isn't there, or invites it to invent what the file said.

After any `mv` or `rm`, sweep both the notes and the tracked repo — and remember
the pre-flatten prefixes still live in old prose:

```bash
# inbound refs across notes + memory (interactive rg; -L follows the symlink)
rg -L --no-ignore-vcs '<topic>' _notes/ \
   ~/.claude/projects/-Users-cyberchitta-GitHub-sorted-studs/memory/
# tracked citations, including the dead cad/-relative and docs/ prefixes
rg -n -o '(_notes|docs|cad/[A-Za-z0-9_/]+)/[A-Za-z0-9/_.-]+\.md' \
   CLAUDE.md cad/ README.md
```

Fix every hit — repoint, or annotate as archived/deleted (`source note … deleted
post-completion — detail below is what survives` keeps the citation honest instead
of merely dead). A negative existence claim ("no file of this name anywhere") is
exactly the claim a `cd`-into-a-subdir scoped search cannot support — root the
search at the repo, or don't make the claim. This bit slice 4 twice on
`hex-cascade.md`.

## Memory-frontmatter housekeeping

The memory index is auto-loaded, so drift there teaches a stale model to *every*
session while an archived note merely sits dated. Two checks catch what neither
prose-reading nor a citation sweep can — run cold on 2026-07-21 they went red
immediately: **13 of 16 files had a drifted `name:` slug and there were 6 dangling
wikilinks.** They ship as `check-memory.sh` beside this file:

```bash
.claude/skills/ss-notes/check-memory.sh
```

It asserts every file's `name:` slug equals its basename (wikilinks resolve
against the slug, so a drifted one breaks every `[[link]]` to it) and that every
`[[wikilink]]` resolves to a file. A wikilink to a **deleted** memory gets
annotated in prose (`[[…]]` implies a file exists); a wikilink to something that
was never a memory — a skill, say — becomes plain text. The script calls
`/usr/bin/grep` by absolute path, because the global `CLAUDE.md` rule means a bare
`rg`/`grep` inside a script silently reads zero files and reports clean.

## Housekeeping pass

No note should be more than a week or two stale. When you run a pass:

1. **Reconcile `worklist.md` against reality** — it is navigation only; if an
   entry has become a second copy of a `state/` item, delete it from the worklist.
   Its characteristic rot is **append-only done-logging**: each closed slice
   arrives as a paragraph, none is ever compressed, and within a week the file is
   a narrative record of work whose real record is a `state/` file or a cross-repo
   handoff (2026-07-26: 130 of 246 lines). A closed slice earns one dated line
   naming where the record lives. What must survive compression is anything a
   *reader deciding what to do next* needs — the open asks, the one still-red
   part, who owes whom.
2. **Sweep `state/` status lines** — correct anything a slice of work has
   falsified; fix stale claims in place, never layer.
3. **Drain `lessons.md`** per the pipeline; update its "do not re-promote" list.
4. **Archive what closed** through the gate — usually one or two `handoff/` files.
5. **Run the citation sweep and `check-memory.sh`** — clean output on both is the
   precondition for calling any reorganisation done. A green checker run proves
   nothing until you've watched it go red: inject a bad reference, confirm it
   reports, remove it.
6. **Commit `working-notes`** — the periodic cadence from the `working-notes`
   skill. Structural moves ride along with it: stage them with `git mv` when you
   make them, so the sweep records a rename rather than a delete plus an
   unrelated creation. *(This line used to say moves "already went in
   immediately" — a stale copy of an earlier cadence, corrected 2026-07-26
   against the `working-notes` skill, which is the authority on when to
   commit.)*

A pass that finds nothing is a successful pass.

## Cross-project reachability — one `_notes` symlink opens every project

Observed live 2026-07-21: one `working-notes` symlink, `ls ~/GitHub/working-notes`,
or `git status` in `_notes/` exposes **every** project's notes — it is one repo.
A session here pulled live operational data (resident names, account numbers,
payment amounts) out of another project's corpus while looking for prior art on
note organisation.

- When reading another project's notes for **prior art on structure**, read the
  *skill* (`.claude/skills/`, committed, written to be read) — not the corpus.
  `ls` for layout; never `cat`/`head` a worklist or handoff.
- `annam` holds data about identifiable people. Treat its `_notes/` as **out of
  bounds** from any sorted-studs session.
- Redaction is not the fix and neither is care — the reachability is structural.
  The finding itself, and re-pricing the repo-split deferral, live in
  `working-notes/DESIGN.md`; that is the owner's call, not this skill's.

## Keep tooling and learnings in this skill

Helper scripts live **here**, beside `SKILL.md`, not in gitignored `_notes/` — a
checker the repo can't see is a checker that rots. And when a pass teaches you
something about *how to maintain these notes*, write it back into this file the
same turn; a lesson about the *CAD* belongs in `cad/CLAUDE.md`, a lesson about a
*module* in its `state/` file. That routing rule applies to this file too.
