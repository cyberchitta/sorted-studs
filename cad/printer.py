"""The printer every printed part is designed against.

Bambu Lab X1 Carbon (owner, 2026-10-10). Build volume 256 × 256 × 256
mm, the maker's figure (confirmed 2026-10-17 against several spec
listings; one makerspace page gives 250³, unexplained). Declared once
here; `bed_fit.py` checks every inspected part against it in its
declared print orientation. It is not a slicer import — bench item 14
(`_notes/backlog/bench-validation.md`) still owns that.
"""

BUILD_VOLUME_MM: tuple[float, float, float] = (256.0, 256.0, 256.0)
