# Exact `collides` and `penetrates`

Both run GJK, and `penetrates` EPA after it, for every pair, even those the casts have exact
formulas for: in 2D points, balls and boxes against each other, and polygons against points,
boxes and polygons; in 3D points against points, balls and boxes, and balls against balls.
Give those pairs the formulas, through the same `Form` the casts use.

## How slow

`cargo bench -p auburn-benches`, 28 Nov 2026, on a machine with 20 to 25 other threads busy:
the ratios hold, the times don't. auburn against the formula, in ns per query:

| | `collides` | `penetrates` |
|---|---|---|
| 2D, `Vec2` | 9 to 35, against 0.9 to 1.4 | 19 to 191, against 1.2 to 2.2 |
| 2D, `Transform2d` | 19 to 61 | 31 to 286 |
| 3D, `Vec3` | 21 to 96, against 1.5 to 2.0 | 39 to 1364, against 1.8 to 6.5 |
| 3D, `Transform3d` | 146 to 332 | 179 to 2384 |

parry has formulas for these: 2D `collides` takes it 1.6 to 20 ns, `penetrates` 2 to 45 (two
boxes 186).

## How wrong

EPA's answers on curved shapes are off, which the benchmarks print on every run:

- 2D: the direction of `penetrates` for a point or a ball against a ball, and a ball against
  a box's corner, is off by up to about 3e-3 rad. EPA stops at a tolerance on the distance,
  and on a circle the direction's error is about its square root.
- 3D: for balls under `Transform3d`, 4 of the benchmarks' 1024 pairs are off by up to 7%:
  two balls in pair 678 give (0.72, 1.17, −1.95) for (0.88, 1.14, −1.91). EPA stops at its cap
  of 64 iterations there ([performance](performance.md)).

When it's done, drop the note on these from `benches/README.md`.
