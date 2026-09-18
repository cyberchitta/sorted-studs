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
plates to z = 40), and each distinct basket of every `FLOOR_LAYOUTS`
floor copied at draw-out
steps of `STEP_MM` from rest to its `withdrawal_travel` plus one step.
Each copy is asserted against every column and bracket.

What is deliberately not asserted:

- Neighbouring baskets. Drawn along its own axis a basket moves to its
  own side of both boundary lines (the axis makes ±half_angle with
  each), so it separates from both neighbours from the first mm.
- The hex rails and radials. They sit at z ∈ [-20, 0] under the basket
  floor, tangent to it; a translation in the floor plane keeps them
  tangent. `single_floor.py` already holds the rest-pose contact.
- Poses past the last step. Past `withdrawal_travel` the column
  envelope has crossed the basket edge and keeps moving away from it
  (the relative path runs parallel to the axis while the edge diverges
  from it), so no later pose can meet it again.

Sampling: `STEP_MM` is 4 mm. The thinnest crossing is a 3 mm basket wall
against a 6 mm clamp plate, travelling obliquely — a 9 mm overlap
window along the path at least — so a 4 mm step lands in any crossing.
Re-derive if either thickness or the step moves.
"""

from build123d import Pos, Rot

from cad_khana.mechanism.assembly import Assembly

from .basket.assembly import basket, withdrawal_travel
from .frame import assembly as _frame
from .params import FLOOR_LAYOUTS

STEP_MM = 4.0


def _steps(travel: float) -> list[float]:
    n = int(travel // STEP_MM) + 2   # rest, the steps, one past travel
    return [i * STEP_MM for i in range(n)]


def build_basket_removal() -> Assembly:
    """Columns + vertex brackets, and each basket at every draw-out step."""
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
        name = f"basket_{int(round(center_deg)):03d}_{n_size:02d}"
        steps = Assembly()
        for d in _steps(withdrawal_travel(half_angle)):
            steps = steps.with_part(f"d{int(d):03d}", body, location=Pos(d, 0, 0))
        a = a.with_subassembly(name, steps, location=Rot(0, 0, center_deg))
        a = a.assert_no_interference_between(name, obstacles)
    return a


assembly = build_basket_removal()
