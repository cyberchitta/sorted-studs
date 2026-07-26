"""Per-frame regression check across the m03 dump cycle — run, don't
import.

    .venv/bin/khana run cad/m03_scanner/frame_sweep.py

`animated_assembly.py` declares the motion (`build_m03_at(t)`); this
script is the orchestration that evaluates it at sampled `t`. The
family is parameterized by a float, which the CLI cannot address, so
the member list lives here:

  * `khana check .../animated_assembly.py` evaluates the degenerate
    `assembly` attribute — frame `t = 0` only.
  * this script evaluates all twelve frames into
    `outputs/animated/t_<t>/`.

**Not `sweep()`.** `cad_khana.mechanism.sweep` measures raw pairwise
overlap volumes and classifies contact phases — it is how a claim gets
*derived*. What this loop does is *hold* claims: every assertion the
tree declares (including the four phased
`assert_allowed_contact(during=JointWindow(...))` pad claims in
`assembly.py`'s `build_turret`) is re-evaluated at each pose. A sweep
evaluates no assertions, so it cannot replace this net.

Failure handling is the CLI's: `khana run` defers every `check()`
failure to its boundary, so all twelve frames run, every
`mechanism.json` on disk is current, and the roll-up names each
failure once before exiting nonzero. There is nothing to accumulate
here by hand.
"""

from cad_khana.mechanism.check import check

from .animated_assembly import build_m03_at


# Per-frame static check at sampled t — regression insurance for
# mid-motion interferences. Samples cover REST, both halves of the
# lift, the dwell peak (local_t = dwell_frac/2 = 0.35 ⇒ t = 0.0875
# in the first quarter), the dwell-end transition (local_t =
# dwell_frac = 0.7 ⇒ t = 0.175), and one transit point per quarter
# so the rotor is checked at each station.
#
# The rising half was unsampled until 2026-07-25: the pad-contact
# phase runs t ∈ (0.0292, 0.0535) and the old set jumped 0 → 0.0625
# straight over it, so of nine frames exactly one (t=0.125, on the
# falling side) was in contact at all. That was tolerable while the
# pair was blanket-suppressed and is not now that the tilt window is
# the claim — a regression net that never enters the phase it guards
# tests nothing. The three added frames enter it from both ends and
# at a second station.
SAMPLE_TS = (
    0.0,        # REST (rotor at station 0, no tilt)
    0.02917,    # tilt 7.5° — pad contact opens (0.4 mm³)
    0.04375,    # tilt 15.0° — rising, deep in contact (14.0 mm³)
    0.0625,     # tilt 24.4° — past the block, contact closed again
    0.0875,     # platform tilt peak (30°), no contact
    0.125,      # tilt 18.3° — falling side of the same window
    0.175,      # end of dwell
    0.2,        # mid-transit
    0.25,       # arrives at station 1
    0.29375,    # station 1's dwell at tilt 15.0° — the same claim
                #   on platform_idle, so the window is tested on a
                #   second cartridge and not just on platform_dump
    0.5,        # arrives at station 2
    0.75,       # arrives at station 3
)

for t in SAMPLE_TS:
    check(build_m03_at(t), out=f"outputs/animated/t_{t:.4f}")
