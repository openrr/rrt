# rrt [![Build Status](https://travis-ci.org/OTL/rrt.svg?branch=master)](https://travis-ci.org/OTL/rrt)  [![crates.io](https://img.shields.io/crates/v/rrt.svg)](https://crates.io/crates/rrt)

RRT (Rapidly-exploring Random Tree) library in Rust

## Using this crate

In your `Cargo.tml`, add below.

```ini
[dependencies]
rrt = "0.1.0"
```

## Examples

There are an example to solve collision avoid problem.

```bash
cargo run --release --example collision_avoid
```

Below is the simplest example.
It search the path from [-1.2, 0.0] to [1.2, 0.0] avoiding [-1, -1] - [1, 1] region.
There are only one function `dual_rrt_connect`, which takes `start`, `goal`,
`is free function`, `random generation function`, `unit length of extend`, `max repeat num`.

```rust
extern crate rand;
extern crate rrt;
fn main() {
  use rand::distributions::{IndependentSample, Range};
  let result = rrt::dual_rrt_connect(&[-1.2, 0.0],
                                     &[1.2, 0.0],
                                     |p: &[f64]| !(p[0].abs() < 1.0 && p[1].abs() < 1.0),
                                     || {
                                        let between = Range::new(-2.0, 2.0);
                                        let mut rng = rand::thread_rng();
                                        vec![between.ind_sample(&mut rng),
                                             between.ind_sample(&mut rng)]
                                     },
                                     0.2,
                                     1000)
              .unwrap();
  println!("{:?}", result);
  assert!(result.len() >= 4);
}
```
