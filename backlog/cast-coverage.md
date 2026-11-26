# Cast coverage through the generic cast

Needs [the generic shape cast](generic-shape-cast.md).

- Add casts for ScaleTranslate, Pos, bevy `Transform`, mixed transform types and
  Shape2d↔Poly2d, plus `tttoi` for Shape2d.
- Keep hand-written exact formulas only where tests show they match the generic cast. Goal:
  delete most of the ~70 hand-written cast functions.
