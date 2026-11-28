# The basic pairs' casts as fast as the formula

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
