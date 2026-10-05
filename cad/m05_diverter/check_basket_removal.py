"""
M05 hex-tower — check module: every basket comes out along its own axis.

    khana check cad/m05_diverter/check_basket_removal.py

The claim: any basket on a floor can be drawn straight out along its own
centreline, with every other basket, column and bracket in place, and
touch nothing. That is the service path the open column channels exist
for (`basket/assembly.py` docstring), and nothing else asserts it —
`single_floor.py` checks the baskets at rest only, which is how a
blocked path stood unnoticed from 2026-05-30 to 2026-09-18.

Fixture: one floor's through-columns and vertex brackets (the frame
parts that reach up into the basket's z band — the brackets' clamp
plates to z = 40). The baskets are not parts. Each distinct basket of
every `FLOOR_LAYOUTS` floor is a keep-out, the exact volume its body
sweeps from rest to `withdrawal_travel`, and every column and bracket
must stay out of it: one `<basket>_draw_out` claim per basket.

What is deliberately not asserted:

- Neighbouring baskets. Drawn along its own axis a basket moves to its
  own side of both boundary lines (the axis makes ±half_angle with
  each), so it separates from both neighbours from the first mm.
- The hex rails and radials. They sit at z ∈ [-20, 0] under the basket
  floor, tangent to it; a translation in the floor plane keeps them
  tangent. The tower's `_build_floor_static` holds the rest-pose
  contact, per floor.
- Travel past `withdrawal_travel`. By then the column envelope has
  crossed the basket edge and keeps moving away from it (the relative
  path runs parallel to the axis while the edge diverges from it), so
  no later pose can meet it again.
"""

from build123d import GeomType, Part, Pos, Rot, extrude

from cad_khana.mechanism.assembly import Assembly

from .basket.assembly import basket, withdrawal_travel
from .frame import assembly as _frame
from .params import FLOOR_LAYOUTS, basket_name


def _swept(body: Part, travel: float) -> Part:
    """The volume `body` passes through translating `travel` along +X.

    Exact, not sampled: a point of the sweep is in the body at rest, or
    on the +X segment from where that segment leaves the body through a
    face — inside that face's extrusion. A plane parallel to X extrudes
    to nothing, so it is skipped.
    """
    prisms = [
        extrude(f, amount=travel, dir=(1, 0, 0))
        for f in body.faces()
        if not (f.geom_type == GeomType.PLANE and abs(f.normal_at().X) < 1e-9)
    ]
    return body.fuse(*prisms).clean()


def build_basket_removal() -> Assembly:
    """Columns + vertex brackets, clear of each basket's draw-out path."""
    a = (
        Assembly()
        .with_subassembly("rails", _frame.build_floor_horizontals())
        .with_subassembly("columns", _frame.build_through_columns())
    )
    obstacles = [
        f"{group}_{k}"
        for k in range(6)
        for group in (
            "columns.outer_vert", "columns.inner_vert",
            "rails.outer_vertex_bracket", "rails.inner_vertex_bracket",
        )
    ]

    # Every distinct basket any floor uses, once. A basket's notches
    # follow from its centre and width (they mark the edges that land on
    # a column), so one name is one shape at one pose on every floor.
    distinct = {
        (center_deg, n_size): notches
        for layout in FLOOR_LAYOUTS.values()
        for center_deg, n_size, notches in layout
    }
    for (center_deg, n_size), notches in sorted(distinct.items()):
        half_angle = 180.0 / n_size
        body = basket(
            half_angle_deg=half_angle,
            notch_left="left" in notches,
            notch_right="right" in notches,
        )
        path = _swept(body, withdrawal_travel(half_angle))
        a = a.assert_clear_of(
            obstacles, path.moved(Rot(0, 0, center_deg)),
            name=f"{basket_name(center_deg, n_size)}_draw_out",
        )
    return a


assembly = build_basket_removal()
