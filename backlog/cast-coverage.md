# Cast coverage through the generic cast

The generic cast is `algorithm::shape_cast` in both crates. In 3D, pairs with a ball use
`algorithm::toi::rounded_shape_cast`, with the ball as its centre and radius: it is more
accurate, so prefer it wherever a shape is a core plus a radius.

- Add casts for ScaleTranslate, Pos, bevy `Transform`, mixed transform types and
  Shape2d↔Poly2d, plus `tttoi` for Shape2d.
- Keep hand-written exact formulas only where tests show they match the generic cast. Goal:
  delete most of the ~70 hand-written cast functions.
