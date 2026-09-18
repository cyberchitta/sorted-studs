---
layout: layouts/article
title: Letting Claude Build Our LEGO Sorter
ogDescription: Most LEGO sorters use one top-down camera. Ours uses four, at the vertices of a tetrahedron — and an AI drew every part.
publishedAt: "2026-05-25"
form: living
group: practice
showrunner: "@restlessronin"
writers:
  - "@claude-opus-4.7"
  - "@claude-fable-5.1"
tags:
  - sorted-studs
  - lego
  - cad
  - multi-view
updates:
  - date: "2026-09-18"
    version: "0.0.2"
    note: Rewritten whole for the one-machine design, with baskets that vary by floor.
---

# sorted-studs

> **Work in progress — nothing here has been built yet.** This page describes the design as it stands and is rewritten whole as the design moves; the byline date is the version's. The first CAD pass never reached fabrication. The second closes as one machine, with 347 parts and 33,250 cross-checked geometric claims, and not one printed part.

An AI-powered LEGO sorting machine, every part vibe Caded with Claude using [cad-khana](https://github.com/cyberchitta/cad-khana). 277 commits over six months, eleven models in the co-author trailers, and three whole-module pivots: the scanner's dump cam became a servo, the singulator's hidden motor moved to its rim, and the singulator chain swung 100° around the diverter to stand on the diverter's own columns.

@restlessronin reads engineering drawings and CAD models but has never drafted a part himself; he sets the concept, flags what's wrong, asks for changes, calls the geometry pivots. The expertise is thin. The output isn't.

## One Machine on One Chassis

Bulk bricks go in at the top of a three-stage singulator, come out one at a time onto a ramp, and land on a scanner turret that photographs each brick from four angles. The turret then tips the brick into the central column of the diverter, which drops it into one of fifty-one baskets.

The diverter is the chassis. The scanner turret stands on the diverter's acrylic top deck with its dump station directly over the column axis. The singulator chain hangs off two of the diverter's corner columns, which continue up through the deck as posts. Every unit is drawn in its own frame and placed by the datums it exports, so the top-level assembly asserts that the three agree about where the deck, the posts, and the ramp's landing are.

<glb-pane src="/assets/exhibits/machine-260918.glb" orbit="-40deg 65deg auto" alt="The whole machine — three singulator stages on a gantry above the scanner turret, both standing on the hex-tower diverter" />

This page covers the CAD. ML, firmware, and the rest of the software will get their own write-ups. The repo opens with the first build; this page is the public record until then.

## Singulator: Three Stages on a Curve

Each stage is a cylindrical [c-channel](https://basically.website/blog/how-we-got-to-10000-parts-per-day), the Basically team's idea: a faceted rotor spins inside a fixed V-trough, bricks ride the outer wall, and an exit arc lets them fall off one at a time. It replaced the flat V-channel in [RobG's design](https://hackaday.io/project/194170-colour-sorting-machine-for-lego), where the project began.

Three identical stages stack on a chain that bends 30° per hop, each stage's chute pointed at the rotor below. The top stage takes the bulk pile and is meant to double as the hopper; the middle one meters and declumps; the bottom one does the final singulation onto the ramp. Each rotor is driven from its rim: a ring gear under the rotor rides a bought turntable bearing, and a small motor under the base plate turns it through a pinion.

The gantry is all 2020 extrusion. Prop bars across the tops of the two through-deck posts carry the bottom and top stages, a T-arm off the taller post carries the middle one, and two braces run from that post down to the deck.

<style>
  .pane-multiview svg { width: 100%; height: auto; display: block; }
  .pane-multiview svg :is(polyline, path) { vector-effect: non-scaling-stroke; }
  .pane-multiview .cad-visible { stroke: var(--color-base-content); stroke-width: 1; }
  .pane-multiview .cad-hidden  { stroke: var(--color-base-content); stroke-opacity: 0.35; stroke-width: 0.5; }
</style>

<figure class="pane pane-multiview my-6" data-asset-root="/assets/exhibits/">
  <div class="grid grid-cols-2 gap-2">
    <%- helpers.inlineSvg('assets/exhibits/singulator-iso-260918.svg') %>
    <%- helpers.inlineSvg('assets/exhibits/chain-iso-260918.svg') %>
    <%- helpers.inlineSvg('assets/exhibits/chain-front-260918.svg') %>
    <%- helpers.inlineSvg('assets/exhibits/chain-top-260918.svg') %>
  </div>
</figure>

One stage with its rotor, trough and rim drive; then the chain on its posts and deck from three sides. The hopper chamber above the top stage and the bolts that hold it all together are not drawn yet.

## Scanner: Four Cameras, Four Stations

Four cameras sit at the vertices of a regular tetrahedron[^tetra-original] around the brick, every pair of lens axes 109.47° apart. A 2x2 brick and a 2x2 plate look identical from above; from any other angle, they don't.

A turret with four platforms indexes each brick through four stations: load, image, idle, dump. At the dump station a bench-mounted servo tips the platform and the brick falls into the diverter's column. The checker also lofts each lower camera's sightline as a cone and asserts that the rig's opaque parts stay outside it, apart from one documented graze at the edge of the platform frame.

<glb-pane src="/assets/exhibits/scanner-260918.glb" orbit="-85deg 45deg auto" alt="Scanner rig — tetrahedral four-camera scanning station, turret rotating through its four stations" />

## Diverter: A Hex Tower of Baskets

A hex tower of baskets, a concept from [Basically](https://basically.website/sorter-v2), first adapted to extrusion-only by Cristian Cuevas and reworked here. Four floors of 2020 hex frame surround a stack of rotating columns driven by one motor at the base. Each column carries a ramp and a trap door: the stack points the ramp at a basket, and the trap on the chosen floor decides whether the brick drops there or falls through to the floor below.

Baskets come in four widths, all sharing the same inner and outer arcs, and each floor divides its six sectors among them differently: one floor of many small categories, one of a few bulk ones, two mixed. Which sizes go where is not a design decision. It will be set by how a collection actually gets sorted. Two of the corner columns continue above the deck as the singulator's posts.

<glb-pane src="/assets/exhibits/diverter-260918.glb" orbit="0deg 75deg auto" alt="Diverter mechanism — hex tower with central rotating ramp and per-floor trap doors, three floors shown with most baskets removed" />

The exhibit shows three of the four floors with most baskets removed so the column, ramp and traps are visible.

## What the Checker Knows

Every assembly declares its claims: no two parts overlap, moving parts keep their clearance through a swept motion, a bar actually rests on its post rather than floating above it. A separate pass per printed part checks that every wall is thick enough and no overhang too steep. The top-level check alone evaluates 33,250 claims across the three units and exits green. That is the evidence behind every geometric statement on this page.

It is not evidence for anything else. Every load and stiffness number is hand statics. Whether a brick actually rides the tangential ramp off the bottom stage, or slides to the centre of the platform after a 7 mm drop, nothing models. The trap actuator, the hopper chamber, the stage seats, the brace-end joints and every bolt pattern are open. The first print will settle more than the next hundred commits.

---

A design that checks itself is not a machine. It is a machine that has not yet been contradicted.

## Credits

Written by @claude-fable-5.1 from assemblies drawn mostly by @claude-opus-4.7, with the exhibits exported from those same assemblies.

Thanks to [RobG](https://hackaday.io/AntennaRob) for several DM conversations. Lukas Weick surfaced prior builds we'd missed and championed alternate singulators. Cristian Cuevas gave detailed fabrication advice.

We started from [RobG's LEGO sorter](https://hackaday.io/project/194170-colour-sorting-machine-for-lego) and stepped, via the constraint of "extrusion + 3D-prints only," to something close to [the Basically LEGO Sorter by LegoSpencer and team](https://basically.website/sorter-v2).

Showrunner: @restlessronin.

LEGO® is a trademark of the LEGO Group of companies, which does not sponsor, authorize, or endorse this site.

[^tetra-original]: The tetrahedral arrangement is original to this build. The singulator and diverter stand on community shoulders.
