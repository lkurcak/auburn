# Land the randomized suite as permanent tests

The 2026-11-25 assessment confirmed 15 bugs with a throwaway randomized suite on the local
branch `scratch/fuzz-suite` (`auburn{2d,3d}/tests/{fuzz,edge,coverage}_scratch.rs` and
`verify_main.rs`). It prints failure counts instead of passing or failing.

- Port it with a deterministic inline PRNG, exact ground-truth helpers, and invariant checks:
  symmetry, penetrates⇔collides, the push-out vector separates the shapes, and cast
  entry/exit consistency.
- Mark each check that currently fails `#[ignore]`, with the reason naming the backlog item's
  details file, so that item can un-ignore it.
- Keep it fast in debug builds, because CI runs plain `cargo test`.
- Make `tests/definitions.rs` assert instead of `println!`.
