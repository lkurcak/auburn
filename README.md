# auburn

[![CI](https://github.com/lkurcak/auburn/actions/workflows/ci.yml/badge.svg)](https://github.com/lkurcak/auburn/actions/workflows/ci.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

Fast and simple collision queries for games that do their own movement and physics: whether
two shapes overlap, the shortest way out, and when and where a moving shape hits another.
auburn has no world and runs no simulation. You call it from your own update, so it fits any
game, from a bullet hell to a platformer.

**[Try auburn2d's queries in your browser](https://lkurcak.github.io/auburn/)**: drag shapes
around and see what each query answers.

## Crates

- [`auburn2d`](./auburn2d) — 2D collision detection and physics
- [`auburn3d`](./auburn3d) — 3D collision detection and physics
- [`playground`](./playground) — try auburn2d's queries by hand, on the desktop and in the browser
- [`benches`](./benches) — how fast the basic queries are, next to parry and the formula a game writes by hand

The previous `auburn` crate has been deprecated and split into `auburn2d` and `auburn3d`. Use those crates directly.

## License

This project is licensed under the MIT License.
