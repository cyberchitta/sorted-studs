---
layout: layouts/article
title: A LEGO Sorter Designed by Claude
ogDescription: Most LEGO sorters use one top-down camera. This one uses four, at the vertices of a tetrahedron.
publishedAt: "2026-05-25"
form: living
group: practice
showrunner: "@restlessronin"
writers:
  - "@claude-fable-5.1"
  - "@claude-opus-4.7"
tags:
  - sorted-studs
  - lego
  - cad
  - multi-view
updates:
  - date: "2026-09-19"
    version: "0.0.2"
    note: Rewritten whole for the one-machine design, with baskets that vary by floor.
---

# sorted-studs

> Work in progress: designed, not yet built. This page is rewritten whole as the design moves.

A LEGO sorting machine, vibe Caded with Claude using [cad-khana](https://github.com/cyberchitta/cad-khana): Claude writes the parts as code, writes the checks they must pass, and cad-khana runs them. 356 parts, 305 commits over six months, eleven models in the co-author trailers.

@restlessronin reads engineering drawings and CAD models but has never drafted a part himself; he sets the concept, flags what's wrong, asks for changes, calls the geometry pivots. The expertise is thin. The output isn't.

This page covers the CAD; ML and firmware come later.

## The Diverter Is the Chassis

Bulk bricks go in at the top of a three-stage singulator, come out one at a time onto a ramp, and land on a scanner turret that photographs each brick from four angles. The turret then tips the brick into the top of the diverter's central stack, which drops it into a basket.

The diverter is the chassis, a hex tower 660 mm across. The scanner turret stands on the diverter's acrylic top deck with its dump station directly over the stack's axis. The singulator chain stands on two of the diverter's corner columns, which continue up through the deck as posts.

<glb-pane src="/assets/exhibits/machine-260919.glb" orbit="140deg 65deg auto" alt="The whole machine: singulator gantry above the scanner turret, both standing on the hex-tower diverter" />

## Singulator: Three Stages on a Curve

Each stage is a cylindrical [c-channel](https://basically.website/blog/how-we-got-to-10000-parts-per-day), the Basically team's idea. It replaced the flat V-channel in [RobG's design](https://hackaday.io/project/194170-colour-sorting-machine-for-lego), where the project began.

Three identical stages step down a chain that bends 30° at each hop, each stage's chute pointed at the stage below. The top stage doubles as the hopper, its wall raised to hold about 3 kg of bricks; the middle one meters and declumps; the bottom one does the final singulation onto the ramp.

A gantry of 2020 aluminum extrusion, standing on the two through-deck posts, carries all three stages.

<style>
  .pane-multiview svg { width: 100%; height: auto; display: block; }
  .pane-multiview svg :is(polyline, path) { vector-effect: non-scaling-stroke; }
  .pane-multiview .cad-visible { stroke: var(--color-base-content); stroke-width: 1; }
  .pane-multiview .cad-hidden  { stroke: var(--color-base-content); stroke-opacity: 0.35; stroke-width: 0.5; }
</style>

<div class="not-prose bg-base-200 rounded-lg shadow-sm border border-base-300 overflow-hidden" style="margin: 2rem 0">
  <figure class="pane-multiview m-0">
    <div class="px-1 sm:px-4 py-1">
      <div class="grid grid-cols-2 gap-2 bg-base-100 rounded border border-base-300 p-2">
        <%- helpers.inlineSvg('assets/exhibits/singulator-iso-260918.svg') %>
        <%- helpers.inlineSvg('assets/exhibits/chain-iso-260918.svg') %>
        <%- helpers.inlineSvg('assets/exhibits/chain-front-260918.svg') %>
        <%- helpers.inlineSvg('assets/exhibits/chain-top-260918.svg') %>
      </div>
    </div>
    <figcaption class="px-1 sm:px-4 py-1 mt-0 mb-1" style="font-size: 0.8125rem; opacity: 0.7">One stage; then the chain on its posts and deck from three sides.</figcaption>
  </figure>
</div>

## Scanner: Four Cameras, Four Stations

Four cameras sit at the vertices of a regular tetrahedron[^tetra-original] around the brick, two above and two below, every pair of lens axes 109.47° apart. A 2x2 brick and a 2x2 plate look identical from above; from any other angle, they don't.

A turret with four platforms indexes each brick through four stations: load, image, idle, dump. The platforms are clear acrylic, so the two lower cameras shoot up through them. At the dump station a stationary servo tips the platform and the brick falls into the diverter's stack.

<glb-pane src="/assets/exhibits/scanner-260918.glb" orbit="-85deg 45deg auto" alt="Scanner rig — tetrahedral four-camera scanning station, turret rotating through its four stations" />

## Diverter: A Hex Tower of Baskets

The concept is [Basically](https://basically.website/sorter-v2)'s (LegoSpencer and team), first adapted to extrusion and 3D prints by Cristian Cuevas and reworked here.

Four floors of 2020 hex frame surround a central stack of rotating segments, one per floor, turned by one motor at the base. Each segment carries a ramp and a trap door: the stack turns to aim every ramp at a basket, and the traps pick the floor. Traps hang open and bricks fall through; a servo on each floor swings its trap up to divert one down the ramp.

Baskets come in four widths, all sharing the same inner and outer arcs, and each floor divides its six sectors among them differently: one floor of many small categories, one of a few bulk ones, two mixed.

The layout drawn, fifty-one baskets in all, is an illustration; how a collection actually gets sorted will set which sizes go where.

<glb-pane src="/assets/exhibits/diverter-260920.glb" orbit="0deg 75deg auto" alt="Diverter mechanism — hex tower with central rotating ramp and per-floor trap doors" />

Claude can redraw the tower at another floor count: the exhibit is a three-floor build, most baskets removed, so the stack, ramps and traps show.

## What the Checker Knows

Claude writes each assembly's claims alongside its parts: no two parts overlap, moving parts keep their clearance through a swept motion, a bar actually rests on its post rather than floating above it, no opaque part crosses a lower camera's sightline. cad-khana also checks printed parts for wall thickness and overhang.

The top-level check alone carries 34,226 claims across the three units, all but about 250 of them the pairwise no-overlap test, and passes.

## Credits

Written by @claude-fable-5.1, from a first version by @claude-opus-4.7, who also drew most of the assemblies the exhibits are exported from.

Thanks to [RobG](https://hackaday.io/AntennaRob) for several DM conversations. Lukas Weick surfaced prior builds we'd missed and championed alternate singulators. Cristian Cuevas gave detailed fabrication advice.

Showrunner: @restlessronin.

LEGO® is a trademark of the LEGO Group of companies, which does not sponsor, authorize, or endorse this site.

[^tetra-original]: The tetrahedral arrangement is original to this build. The singulator and diverter stand on community shoulders.
