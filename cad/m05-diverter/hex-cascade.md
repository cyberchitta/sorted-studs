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

- 6 vertical corner columns of aluminum extrusion (likely 2020 or
  2040), one at each vertex of the hexagon.
- Horizontal crossbeam rings at each floor level and top/bottom caps,
  also aluminum extrusion.
- 3D-printed **corner brackets** join three extrusions (two horizontal,
  one vertical) at every vertex of every ring. These are the pink
  star-shaped parts in the image.
- Diagonal braces in each hex face for rigidity.

### Per-floor assembly

- **Basket ring** — six (or however many) printed baskets arranged
  radially. Each basket is removable for emptying.
- **Rotating ramp** — a scoop/ramp mounted on the central vertical
  shaft. The ramp tilts up/down and rotates to address baskets.
- **Drive** — servo or stepper on the floor's central hub, driving the
  ramp's rotation; a second actuator (or a cam mechanism off the
  rotation) handles the up/down tilt.
- **Floor plate** — structural disk that carries the basket ring and
  the central shaft bearing.

### Top / bottom

- Top cap with the hopper opening (blue rectangle + orange rim in the
  image) that funnels scanned bricks into the central drop column.
- Bottom cap that catches any brick that somehow passes all floors
  (the "uncertain" fallback) and holds the electronics / motor
  controllers.

## Parameters (to be decided)

- Number of floors (3 in the image; 4–5 if vertical space allows).
- Baskets per floor (6 in the image; could be 4, 8, or 12 — tradeoff
  between basket size and bin count).
- Extrusion profile (2020 / 2040 / mix).
- Drop column diameter — must accommodate the largest brick we sort
  plus slop.
- Ramp angle — steep enough to slide reliably, shallow enough to not
  launch bricks out of baskets.
- Floor spacing — must fit basket height + ramp swing clearance + drive
  clearance.

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

## Build order

Modeling starts with the [corner bracket](corner_bracket/assembly.py)
(simplest printable part, proves the cad-khana workflow), then the
per-floor [ramp mechanism](ramp_mechanism/assembly.py) — shaft, floor
plate, bearing boss, hub-yoke, and ramp — which exercises the full
diagnostics + assertion surface.
