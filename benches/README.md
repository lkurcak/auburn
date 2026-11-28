# auburn benchmarks

How fast auburn's basic queries are, next to [parry](https://parry.rs) and the formula a game
would write by hand: `collides`, `penetrates`, `toi` and, in 2D, `cast`, between points,
balls and boxes, at `Vec2`/`Vec3` positions and turned by a `Transform2d`/`Transform3d`.
These pairs are auburn's hot path, and they should be as fast as the formula.

```sh
cargo bench -p auburn-benches                     # all of them, about six minutes
cargo bench -p auburn-benches -- 2d/collides      # those whose id starts so
cargo bench -p auburn-benches -- --save-baseline before   # then, after a change:
cargo bench -p auburn-benches -- --baseline before        # each one's change since
```

Each benchmark's id is `<2d|3d>/<query>/<moving shape>-<other shape>/<transformation>/<who>`,
with `auburn`, `parry` or `formula` last. The time criterion gives is for one query.

## What they time

- **The pairs.** 1024 of each kind, drawn the same on every run, are queried one after
  another, so the hits and misses don't fall into a pattern the branch predictor learns. A
  point is in the other shape in about one pair in eight, and two balls or boxes overlap in
  about two in five. From a quarter to two thirds of the casts hit.
- **auburn** through its public API, as a game calls it: `a.at(&pos).collides(&b.at(&pos))`.
- **parry** 0.31 through its public query functions: `intersection_test`, `contact`, and
  `cast_shapes` for `toi` and `cast`, or a point query and a ray cast for a point, as parry
  has no point shape. They choose the algorithm at run time from the shapes, as they would
  in a game. Every answer is turned into auburn's terms, which costs a move or two.
- **The formula** in [`formula2d.rs`](formula2d.rs) and [`formula3d.rs`](formula3d.rs): each
  pair as a point against a shape at the origin, as a game would write it. A turned box works
  in its own frame. Two boxes turned different ways have none, as that takes the separating
  axis test, and nor does a 3D ball cast against a box.

Before timing anything, each benchmark checks that the formula and parry agree on every
pair, so that they answer the same question. Where auburn's answers differ from theirs, it
prints how many and the first one. Today none do.

Times depend on the machine and on what else runs on it, so there are none here: run them.
