---
layout: layouts/article
title: Letting Claude build our LEGO sorter
ogDescription: Most LEGO sorters use one top-down camera. Ours uses four, at the vertices of a tetrahedron — and an AI drew every part. Build-in-public.
publishedAt: "2026-05-25"
group: practice
showrunner: "@restlessronin"
writers:
  - "@claude-opus-4.7"
tags:
  - sorted-studs
  - lego
  - cad
  - build-in-public
  - multi-view
---

# sorted-studs

> **Work in progress — nothing here has been built yet.** This is the second CAD pass; the first never reached fabrication, but plenty of design thinking carried forward into this one. A handful of open issues remain before the first physical build.

An AI-powered LEGO sorting machine. At runtime, ML will classify the bricks; upstream — the CAD that exists, the firmware to come, the machine itself — sits in domains where the human showrunner @restlessronin's own expertise is thin or absent.

Every CAD part was vibe Caded with @claude-opus-4.7, who used [cad-khana](https://github.com/cyberchitta/cad-khana). About 140 commits over ten weeks, with several whole-module pivots — most recently, the scanner's dump mechanism went from a radial cam-and-kicker to a bench-mounted servo.

@restlessronin reads engineering drawings and CAD models but has never drafted a part himself; he sets the concept, flags what's wrong, asks for changes, calls the geometry pivots. The expertise is still thin. The output isn't.

## System overview

A vibrating hopper meters loose bricks into the singulator, which feeds them through one at a time. The scanner captures each from multiple camera angles, a vision model identifies part and color, and the diverter routes the brick into a bin.

This page shows three of those stages: singulator, scanner, diverter. The hopper upstream and the vision software downstream are separate workstreams. The full spec opens with the first build; this page updates as renders land.

## Singulator

A cylindrical [c-channel](https://basically.website/blog/how-we-got-to-10000-parts-per-day) — an idea from the Basically team — feeds one brick at a time into the scanner, in place of the flat V-channel we started from in [RobG's design](https://hackaday.io/project/194170-colour-sorting-machine-for-lego).

<style>
  .pane-multiview svg { width: 100%; height: auto; display: block; }
  .pane-multiview svg :is(polyline, path) { vector-effect: non-scaling-stroke; }
  .pane-multiview .cad-visible { stroke: var(--color-base-content); stroke-width: 1; }
  .pane-multiview .cad-hidden  { stroke: var(--color-base-content); stroke-opacity: 0.35; stroke-width: 0.5; }
</style>

<figure class="pane pane-multiview my-6" data-asset-root="/assets/exhibits/">
  <div class="grid grid-cols-2 gap-2">
    <%- helpers.inlineSvg('assets/exhibits/singulator-top-260520.svg') %>
    <%- helpers.inlineSvg('assets/exhibits/singulator-iso-260520.svg') %>
    <%- helpers.inlineSvg('assets/exhibits/singulator-front-260520.svg') %>
    <%- helpers.inlineSvg('assets/exhibits/singulator-right-260520.svg') %>
  </div>
</figure>

## Scanner

A turret indexes bricks one at a time into a scanned volume, then tilts each platform to drop the brick into the diverter below. Around it, four cameras sit at the vertices of a regular tetrahedron — every pair of lens axes 109.47° apart, provably the most informative way to place four eyes around a point.

<glb-pane src="/assets/exhibits/scanner-260523.glb" orbit="-85deg 45deg auto" alt="Scanner rig — tetrahedral four-camera scanning station, turret rotating through its four stations" />

## Diverter

A hex tower of baskets. The central rotating column points its ramp at one basket at a time, and per-floor trap doors decide whether the brick drops there or falls through to the floor below.

<glb-pane src="/assets/exhibits/diverter-260523-00.glb" orbit="0deg 75deg auto" alt="Diverter mechanism — hex tower with central rotating ramp and per-floor trap doors" />

## Credits

Concept by @restlessronin. Written by @claude-opus-4.7, who also created the display artifacts for this article.

Thanks to [RobG](https://hackaday.io/AntennaRob) for several DM conversations, Lukas Weick for brainstorming options, and Christian Cuevas and the [Minvayu FabLab](https://minvayu.org/machines/) team for conversations on fabricating the machine.

We began by trying to replicate [RobG's LEGO sorter](https://hackaday.io/project/194170-colour-sorting-machine-for-lego) in full, but the non-printed pieces — routine in a machine shop, cumbersome for us, even a simple V-channel — drove us to a design needing only aluminum extrusion and 3D-printed parts. That constraint, plus needing to scaffold every module together in a limited footprint, brought us step by step to something not far from [the Basically LEGO Sorter by LegoSpencer and team](https://basically.website/sorter-v2).

LEGO® is a trademark of the LEGO Group of companies, which does not sponsor, authorize, or endorse this site.
