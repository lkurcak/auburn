# 3D casts of the basic pairs as fast as the formula

The basic pairs, points, balls and boxes that don't turn, should be as fast as the formula a
game would write by hand (Design, in `CLAUDE.md`). In 2D, `toi` and `cast` take 0.7 to 2.4
times the formula's time since #43. 3D's casts are slower.

## How slow

`cargo bench -p auburn-benches -- 3d/toi`, 28 Nov 2026, on a machine with 20 to 25 other
threads busy: the ratios hold, the times don't. auburn against the formula, in ns per query:

| Pair | 3D `toi` |
|---|---|
| point–ball | 17 / 4.5 |
| point–box | 26 / 13 |
| ball–ball | 12 / 2.7 |
| ball–box | 484 / – |
| box–box | 762 / 15 |

3D has no `cast` yet (Now has it, after this item).

## What made 2D fast

In `auburn2d/src/interactions/cast.rs`, in this order of how much each saved:

- `toi` and `cast` read a forward-only formula per pair, `touch`, that finds when they first
  touch from now on and nothing else: no exit, no `direction_and_speed` with its divisions
  and square root. `toi` drops the normal, and LLVM leaves its code out, once it is inlined.
- `#[inline(always)]` on the match on the pair's forms, on `Form::of`, on the frame helpers
  and on the formulas. LLVM kept some of them as calls in a caller as large as `cast`, and a
  call hides which pair it is, so every other pair's code stayed. What only turned boxes and
  polygons need is out of line, and so is the scaled path of a circle, `#[cold]`. `collides`
  then needed `#[inline]` on its trait impl, which LLVM stopped inlining into the caller.
- The slab test picks each axis's near side by `copysign` with the velocity, not by a select,
  which LLVM made a branch that random signs mispredict. A side the path starts on and runs
  along gives 0 × ∞ = NaN, which `f32::max` and `f32::min` skip.
- The circle's chord comes from the cross product, `vel.length_squared() * r² - (p × vel)²`,
  which stays precise far from the ball. If the square of the speed isn't a normal number,
  the cold path goes in units of the velocity's largest component.
- `is_at_rest` is a compare with 2⁻¹²⁸, not a division.

Look at what a query compiles to (`CLAUDE.md`, Benchmarks) before guessing at its cost.

## For 3D

- The same `touch` for points and balls, and the slab test for a point against a box.
- Two boxes that don't turn are the slab test on the sum of their half sizes. A ball against a
  box that doesn't turn is a ray against a rounded box, whose faces are the box's widened by
  the radius, its edges cylinders and its corners spheres: the widened box first, then the
  edge or corner the entry point is in, as 2D's corner discs.
- These would go into the GJK shape cast's place for those pairs, so `toiae` and the
  time-traveling casts want them as spans too, as 2D has `exact`.
