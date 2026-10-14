# sorted-studs

**CAD for a LEGO sorting machine: designed, not yet built.**

The machine is meant to take a pile of loose bricks, singulate them, photograph each
one from four directions at once, and drop it into a basket by part and colour. This
repo holds the mechanical design so far: Build123d assemblies checked with
[CAD Khana](https://github.com/cyberchitta/cad-khana), and the
[build-in-public site](https://sorted-studs.cyberchitta.cc) that publishes them.
Classification, firmware and inventory come later; none of that is here yet.

Nothing has been printed. The checks are computational: interference, clearance and
printability. Fit, tolerance and print success are still unknown.

---

## The machine

Bulk bricks go in at the top of a three-stage singulator, come out one at a time onto
a ramp, and land on a scanner turret that photographs each brick from four angles. The
turret tips the brick into the diverter's central stack, which drops it into a basket.

**Diverter — the chassis.** A hex tower 660 mm across: four floors of 2020 hex frame
around a central stack of rotating segments, turned by one motor at the base. Each
segment carries a ramp and a trap door; the stack aims the ramps, a servo per floor
picks the floor. The concept is [Basically](https://basically.website/sorter-v2)'s,
first adapted to extrusion and 3D prints by Cristian Cuevas.

**Scanner — four cameras, four stations.** Four cameras at the vertices of a regular
tetrahedron around the brick, every pair of lens axes 109.47° apart. A turret of clear
acrylic platforms indexes each brick through load, image, idle and dump; the two lower
cameras shoot up through the platform. It stands on the diverter's top deck, its dump
station over the stack's axis.

**Singulator — three stages on a curve.** Three cylindrical c-channel stages (the
Basically team's idea) step down a chain that bends 30° at each hop. The top stage
doubles as the hopper; the bottom one singulates onto the ramp. A 2020 gantry carries
the chain on two of the diverter's columns, which continue up through the deck.

The project began as a port of RobG's
[Colour Sorting Machine for Lego](https://hackaday.io/project/194170-colour-sorting-machine-for-lego),
swapping its colour sensor for cameras. Keeping everything FDM-printable on aluminum
extrusion pushed the singulator and diverter toward Basically's architecture; the
tetrahedral camera station is this build's own.

---

## Repository layout

```
cad/            Build123d / CAD Khana assemblies, one directory per unit
                (m02 singulator and chain, m03 scanner, m05 diverter).
                cad/assembly.py composes all three and runs the
                cross-unit checks.
subsite/        The build-in-public eleventy site (bun-managed). Its
                exhibits are GLB models and drawings exported from cad/.
pyproject.toml  uv-managed Python environment.
```

To set up, clone [cad-khana](https://github.com/cyberchitta/cad-khana) next to this
repo (`../cad-khana`), then run `uv sync` here. Run checks with
`.venv/bin/khana check <assembly.py>`.

---

## License

[Apache License 2.0](LICENSE).

---

_sorted-studs is not affiliated with the LEGO Group. LEGO® is a trademark of the LEGO
Group, which does not sponsor, authorize, or endorse this project._
