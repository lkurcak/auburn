# `penetrates` for the basic pairs as fast as the formula

The basic pairs, points, balls and boxes that don't turn, should be as fast as the formula a
game would write by hand (Design, in `CLAUDE.md`). `penetrates` has been exact since #42, and
`collides` with it, but only `collides` is as fast as the formula.

## How slow

`cargo bench -p auburn-benches -- penetrates`, auburn against the formula in ns per query, on
a quiet machine. 2D after the fast 2D casts (28 Nov 2026), 3D as #42 left it:

| Pair | 2D `Vec2` | 2D `Transform2d` | 3D `Vec3` | 3D `Transform3d` |
|---|---|---|---|---|
| point–ball | 5.7 / 1.2 | 5.9 / 1.2 | 1.4 / 1.4 | 7.2 / 1.4 |
| point–box | 5.7 / 1.1 | 6.4 / 1.7 | 6.3 / 1.4 | 7.9 / 7.2 |
| ball–ball | 5.9 / 1.4 | 7.0 / 1.4 | 6.3 / 1.6 | 7.0 / 1.6 |
| ball–box | 6.3 / 2.0 | 8.0 / 6.2 | 23 / 6.5 | 30 / 9.4 |
| box–box | 6.2 / 1.4 | – | 7.3 / 1.8 | – |

## What's known so far

- `collides` took 4 times the formula's time until the match on the pair's forms was
  `#[inline(always)]`: with `#[inline]`, LLVM kept it as a call. Once `Form::of` was
  `#[inline(always)]` too, for the casts, it took twice the formula's time again until its
  trait impl got `#[inline]`: the bigger body kept LLVM from inlining it into the caller.
- 2D's ways out for the basic pairs are `#[inline(always)]` since the fast 2D casts, which use
  them for shapes that overlap already: point–box, ball–box and box–box got faster.
- 2D point–ball `penetrates` compiles to the formula's arithmetic (objdump on a crate that
  calls it), but the benchmark loop calls it out of line and copies the `Option<Vec2>` back
  through memory. 3D point–ball at `Vec3` is as fast as the formula, the others aren't.
- `#[inline]` on `Collider::penetrates` and on the trait impls changed nothing in 2D and made
  3D point–ball four times slower. That was before the 2D changes above: try it again, and
  look at what the benchmark loop compiles to (`CLAUDE.md`, Benchmarks).
