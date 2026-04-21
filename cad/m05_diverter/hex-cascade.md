# M05 — Diverter: Hex Tower Cascade

Specific realization of [Module 5 — Diverter](../../docs/m05-diverter.md)
for sorted-studs v1.

## Concept

A hexagonal aluminum-extrusion tower with multiple **floors**. Each floor
carries a radial array of **baskets** and one **rotating ramp** on a
central vertical shaft. Bricks enter the tower through a hopper at the
top and fall straight down the central column under gravity.

On each floor the ramp has two states:

- **Down** — retracted below the drop column; bricks pass through.
- **Up** — swung up into the column; the brick lands on the ramp's
  angled surface and slides off into whichever basket the ramp is
  aimed at.

To route a brick to basket `(floor F, position P)`:

1. Every floor above `F` keeps its ramp **down** (brick passes through).
2. Floor `F`'s ramp rotates to angular position `P` and moves **up**
   before the brick arrives.
3. The brick lands on the ramp and is directed into basket `P` on
   floor `F`.
4. All floors reset to down between bricks.

Bin count scales as **floors × baskets/floor**. For a 3-floor tower
with 6 baskets per floor that's 18 bins in a compact vertical footprint.

## Why this approach

- **Footprint.** Floor area is dominated by the basket ring; adding
  floors adds bins without spreading horizontally. Good for a
  workshop-scale build.
- **Speed.** Ramp rotation on the selected floor can happen while the
  brick is still being scanned; only one axis needs to be ready per
  decision.
- **Modularity.** Floors are identical sub-assemblies — build one,
  replicate vertically.
- **Fails safe.** Ramp default is down; a stuck ramp just passes the
  brick through to the next floor, not into the wrong bin.

## Structure

### Frame (hexagonal tower)

Two concentric hexagons per floor:

- **Outer hex ring** — six horizontal extrusions at each floor level
  forming the tower's outer perimeter. Six vertical corner columns
  (2020) at the outer hex's vertices run the full tower height.
- **Inner hex ring** — six shorter horizontal extrusions at the same
  floor level forming a smaller concentric hex around the central
  drop column. Vertical columns at the inner hex's vertices may run
  full-height or be segmented between floors.
- **Radial rails** — extrusions from each inner-hex vertex out to the
  corresponding outer-hex vertex. They tie the two hex rings together
  and stiffen the inner ring against basket load. A further set of
  radial arms extends inward from the inner hex to the ring bearing
  — the **spider** that holds the bearing coaxial with the column
  without needing a solid floor disk. (The inner-to-bearing arms may
  be coextensive with the inner-to-outer rails on some floors, or a
  separate set — to be decided during frame layout.)
- **3D-printed corner brackets** join the extrusions at every vertex
  (shown as pink star-shaped parts in early reference sketches).
- **Diagonal braces** in each bay between inner and outer hex for
  torsional rigidity.

### Per-floor assembly

- **Basket ring** — one basket per bay between inner and outer hex.
  Each basket has a top flange wider than the bay so it hangs on the
  inner- and outer-hex ring extrusions (drawer on two rails); drops in
  from above, lifts out for emptying. Number per floor is a design
  parameter (see below).
- **Rotating column + flap** — a **square-section tube** with a single
  exit opening cut into one wall. The column has **circular end-collars**
  that ride in **fixed ring bearings** attached to the tower frame,
  letting the whole column section rotate in place to aim the opening
  at the selected basket. A flap hinged inside the wall at the
  opening's tangential edges tilts between DOWN (flush, column sealed
  under the hinge) and UP (tilted into the column to catch bricks).
- **Drive** — a motor on the base floor rotates the central column
  stack (or per-floor, if independent rotation is wanted); a second
  actuator per floor controls the flap's up/down tilt. Gravity-bias
  to DOWN so failures fall-through, not misroute.
- **Bearing mount** — the ring bearing is carried by the spider
  (radial arms from the inner hex inward to the bearing's outer race).
  No solid floor disk on intermediate floors — the drop chute is
  unobstructed, and any brick that passes all flaps falls to the
  bottom fallback bin with nothing in its way.
- **Flap shape** — flat, rigid plate hinged at its middle. The catch
  section (inner of hinge) spans the full column cross-section when UP;
  the spout section (outer of hinge) reaches over the gap between the
  column wall and the basket lip, and tapers to the basket's angular
  slot width at its exit. Brick lands on the catch, slides through the
  hinge, continues on the spout, drops into the basket — never leaves
  the surface.

The rotating-column topology (one opening that moves) is deliberate.
The alternative — a fixed column with N openings (one per basket) —
doesn't scale: the flap would have to fit through a narrow slot, or
leave N−1 slots uncovered at any moment. Rotating the column keeps the
opening size decoupled from basket count.

The square cross-section (instead of cylindrical) matches the flat
flap, so a single flat plate covers the full column interior in UP
state without the circular-vs-rectangular gap that a cylindrical
column would introduce.

### Top / bottom

- Top cap with the hopper opening (blue rectangle + orange rim in the
  image) that funnels scanned bricks into the central drop column.
- Bottom cap that catches any brick that somehow passes all floors
  (the "uncertain" fallback) and holds the electronics / motor
  controllers.

## Parameters (to be decided)

Four knobs size the per-floor mechanism; everything else derives from
them. The four live in `params.py` (shared across sub-assemblies);
column/trap/ramp derivations are in `ramp_mechanism/assembly.py` and
basket derivations in `basket/assembly.py`.

- **`MAX_BRICK_DIM`** — largest LEGO dimension this sorter will handle
  (currently 80 mm). Drives the column's interior side length and the
  minimum spout-exit width.
- **`N_BASKETS_PER_FLOOR`** — 6, 12, or 18. Angular spacing between
  baskets is 360°/N (60°, 30°, 20°). More baskets ⇒ narrower basket
  angular slot ⇒ narrower spout exit required for clean targeting.
- **`R_BASKET_INNER`** — radial distance from column axis to basket
  inner lip. Combined with `N_BASKETS_PER_FLOOR`, sets the basket's
  angular slot width: `2 · R · sin(π / N)`. Must be large enough that
  `SPOUT_EXIT_W ≥ MAX_BRICK_DIM`, or the spout can't pass the largest
  brick. High `N` + large `MAX_BRICK_DIM` pushes `R_BASKET_INNER` and
  therefore the tower's outer footprint upward:
  - 6 baskets / 80 mm bricks → `R ≥ 80 mm`
  - 12 / 80 → `R ≥ 155 mm`
  - 18 / 80 → `R ≥ 230 mm`
  The script `assert`s this and fails loudly if the combination is
  geometrically impossible.
- **Number of floors** (3 in the reference image; 4–5 if vertical space
  allows).

Derived / secondary:

- **Column interior side** = `MAX_BRICK_DIM + 2 · slop` (≈ 90 mm for
  default).
- **Column outer side** = interior + 2 × wall thickness (≈ 96 mm).
- **Collar OD** = `outer_side · √2 + clearance` (≈ 140 mm) — must
  clear the square corners as they sweep during rotation.
- **Spout exit width** = `min(flap_width, 2·R·sin(π/N))`.
- **Exit-opening height** ≥ the larger of (spout peak in DOWN) and
  (brick-clearance above the hinge in UP).
- **Floor spacing** — must fit basket height + flap swing clearance +
  drive clearance.
- **Flap angle** — currently 10° above horizontal in UP; steep enough
  to slide reliably, shallow enough not to launch bricks out of
  baskets (to be verified with a basket-lip diagnostic once the
  basket is modeled).

## Reference design observations

We're drawing on a prior radial-bin tower design (shared as reference
images). Things worth carrying into our ramp mechanism work:

- **Central drop column is the trunk line.** A single vertical chute
  runs the full height of the tower; every floor taps off it. The
  bottom of the chute is the natural home for the uncertain/fallback
  bin — anything not caught on the way down lands there with no extra
  routing logic.
- **Combined catch + guide surface.** In the reference, a single
  angled flap both *stops* the brick and *slides* it into the basket.
  Our ramp should do the same — one surface, not a separate catcher
  plus chute — so the brick never loses contact while changing
  direction.
- **Default-down, fail-through.** Ramp rests in the pass-through
  (down) state. Power loss, stuck actuator, or a missed command all
  degrade to "brick falls to the bottom fallback bin," never to
  "brick lands in the wrong bin." This is the fail-safe stance
  already in the m05 spec.
- **Ramp angle and basket lip.** The reference angles the flap
  steeply enough to slide reliably but not so steep that the brick
  bounces over the basket's far wall. Basket inner lip should be
  higher than the ramp's launch trajectory — a constraint to verify
  in diagnostics once we have a ramp angle picked.

## Open questions

- Ramp up/down mechanism: separate servo vs. cam driven off rotation?
- How to arrest brick momentum on the ramp so it slides rather than
  bounces out of the target basket.
- Is the drop column lined (soft material) to dampen impacts, or bare?
- Can two adjacent floors' ramps be "up" safely if the one below is
  aimed away from the drop column — i.e., can we pipeline two bricks
  in flight?
- **Ring-bearing type** — options include a printed plain bearing
  ring, a lazy-susan turntable, or a commercial slewing ring bearing
  (as sketched in the reference image). Trade-off is load capacity
  and rotational precision vs. cost, weight, and sourcing. Not yet
  settled.

## Build order

Modeling starts with the [corner bracket](corner_bracket/assembly.py)
(simplest printable part, proves the cad-khana workflow), then the
per-floor sub-assemblies:

- [ramp mechanism](ramp_mechanism/assembly.py) — bearing spider, ring
  bearing, square column with bottom collar, hinge pin, trap, and
  fixed ramp. Exercises the full diagnostics + assertion surface on
  the column side.
- [basket](basket/assembly.py) — trapezoidal-prism containers arrayed
  around the column axis; standalone checks against a minimal column
  stub and adjacent-pair overlap.
- [top-level diverter](assembly.py) — composes the two sub-assemblies
  with real geometry and runs cross-boundary interference checks
  (baskets vs column-side parts, ramp tip vs +X basket).

Shared interface constants (`MAX_BRICK_DIM`, `N_BASKETS_PER_FLOOR`,
`R_BASKET_INNER`, `R_HEX_OUTER`, `COL_OUTER_SIDE`) live in
`params.py` and are imported by every sub-assembly.

The base floor adds a motor + shaft + bearing boss to drive column
rotation; those are not in the generic-floor model and will be a
separate base-floor assembly.
