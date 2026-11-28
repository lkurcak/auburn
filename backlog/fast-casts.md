# The basic pairs' casts and `penetrates` as fast as the formula

The basic pairs, points, balls and boxes that don't turn, should be as fast as the formula a
game would write by hand (Design, in `CLAUDE.md`). Their casts aren't yet.

## How slow

`cargo bench -p auburn-benches`, 28 Nov 2026, on a machine with 20 to 25 other threads busy:
the ratios hold, the times don't. auburn against the formula, in ns per query:

| Pair | 2D `toi` | 2D `cast` | 3D `toi` |
|---|---|---|---|
| point–ball | 13 / 1.9 | 15 / 6.7 | 17 / 4.5 |
| point–box | 15 / 1.9 | 18 / 7.3 | 26 / 13 |
| ball–ball | 15 / 1.8 | 19 / 7.3 | 12 / 2.7 |
| ball–box | 45 / 6.3 | 49 / 11 | 484 / – |
| box–box | 19 / 4.7 | 37 / 12 | 762 / 15 |

Under `Transform2d` the 2D times are much the same.

## Where the time might go

Not profiled yet: profile first, and time each change with `--save-baseline`.

- Every cast works out the whole span, entry and exit, and `toi` drops the exit. A query
  should compute only what it answers.
- `direction_and_speed` turns `vel` into a unit direction and a speed, with a division and a
  square root, before any formula starts.
- Ball–Box2d `toi` finds the exit too, which made it about twice as slow as the hand-written
  cast it replaced in #32.
- 3D has no formula for a box against a ball or a box, so those go through GJK's shape cast.
  Two boxes that don't turn are the slab test on the sum of their half sizes. A ball against
  a box that doesn't turn is a ray against a rounded box, whose edges are cylinders and whose
  corners are spheres.

## `penetrates`

Exact since #41, and `collides` with it, but only `collides` is as fast as the formula.
`cargo bench -p auburn-benches -- penetrates`, 28 Nov 2026, on a quiet machine, auburn
against the formula in ns per query:

| Pair | 2D `Vec2` | 2D `Transform2d` | 3D `Vec3` | 3D `Transform3d` |
|---|---|---|---|---|
| point–ball | 5.7 / 1.2 | 6.0 / 1.2 | 1.4 / 1.4 | 7.2 / 1.4 |
| point–box | 7.3 / 1.2 | 6.8 / 1.8 | 6.3 / 1.4 | 7.9 / 7.2 |
| ball–ball | 5.8 / 1.5 | 6.9 / 1.4 | 6.3 / 1.6 | 7.0 / 1.6 |
| ball–box | 9.5 / 2.0 | 16 / 6.2 | 23 / 6.5 | 30 / 9.4 |
| box–box | 10 / 1.4 | – | 7.3 / 1.8 | – |

What's known so far:

- `collides` took 4 times the formula's time until the match on the pair's forms was
  `#[inline(always)]`: with `#[inline]`, LLVM kept it as a call. `exact`, the casts' match,
  is only `#[inline]`.
- 2D point–ball `penetrates` compiles to the formula's arithmetic (objdump on a crate that
  calls it), but the benchmark loop calls it out of line and copies the `Option<Vec2>` back
  through memory. 3D point–ball at `Vec3` is as fast as the formula, the others aren't.
- `#[inline]` on `Collider::penetrates` and on the trait impls changed nothing in 2D and made
  3D point–ball four times slower.
