/*
  Copyright 2017 Takashi Ogura

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
*/

#![doc = include_str!("../README.md")]
#![warn(missing_docs)]

use kdtree::distance::squared_euclidean;
use num_traits::float::Float;
use num_traits::identities::Zero;
use rand::distributions::{Distribution, Uniform};
use std::fmt::Debug;

// #[derive(Debug)]
// enum ExtendStatus {
//     Reached(usize),
//     Advanced(usize),
//     Trapped,
// }

/// Trait to express a weight/cost for a node in the tree
pub trait Weight: Float + Zero {}

impl Weight for f64 {}
impl Weight for f32 {}

/// Node that contains user data
#[derive(Debug, Clone)]
struct Node<T, W: Weight> {
    parent_index: Option<usize>,
    data: T,
    weight: W,
}

impl<T, W: Weight> Node<T, W> {
    fn new(data: T, weight: W) -> Self {
        Node {
            parent_index: None,
            data,
            weight,
        }
    }
}

/// RRT
#[derive(Debug)]
pub struct Tree<N, W>
where
    N: Float + Zero + Debug,
    W: Weight,
{
    kdtree: kdtree::KdTree<N, usize, Vec<N>>,
    vertices: Vec<Node<Vec<N>, W>>,
}

impl<N, W> Tree<N, W>
where
    N: Float + Zero + Debug,
    W: Weight,
{
    fn new(dim: usize) -> Self {
        Tree {
            kdtree: kdtree::KdTree::new(dim),
            vertices: Vec::new(),
        }
    }

    // Add a vertex to the tree
    fn add_vertex(&mut self, q: &[N], weight: W) -> usize {
        let index = self.vertices.len();
        self.kdtree.add(q.to_vec(), index).unwrap();
        self.vertices.push(Node::new(q.to_vec(), weight));
        index
    }

    //
    fn add_edge(&mut self, q1_index: usize, q2_index: usize) {
        self.vertices[q2_index].parent_index = Some(q1_index);
    }

    fn remove_edge(&mut self, q_index: usize) {
        self.vertices[q_index].parent_index = None;
    }

    //
    fn get_nearest_index(&self, q: &[N]) -> usize {
        *self.kdtree.nearest(q, 1, &squared_euclidean).unwrap()[0].1
    }

    //
    // fn extend<FF>(&mut self, q_target: &[N], extend_length: N, is_free: &mut FF) -> ExtendStatus
    // where
    //     FF: FnMut(&[N]) -> bool,
    // {
    //     assert!(extend_length > N::zero());
    //     let nearest_index = self.get_nearest_index(q_target);
    //     let nearest_q = &self.vertices[nearest_index].data;
    //     let diff_dist = squared_euclidean(q_target, nearest_q).sqrt();
    //     let q_new = if diff_dist < extend_length {
    //         q_target.to_vec()
    //     } else {
    //         nearest_q
    //             .iter()
    //             .zip(q_target)
    //             .map(|(near, target)| *near + (*target - *near) * extend_length / diff_dist)
    //             .collect::<Vec<_>>()
    //     };
    //     debug!("q_new={q_new:?}");
    //     if is_free(&q_new) {
    //         let new_index = self.add_vertex(&q_new);
    //         self.add_edge(nearest_index, new_index);
    //         if squared_euclidean(&q_new, q_target).sqrt() < extend_length {
    //             return ExtendStatus::Reached(new_index);
    //         }
    //         debug!("target = {q_target:?}");
    //         debug!("advanced to {q_target:?}");
    //         return ExtendStatus::Advanced(new_index);
    //     }
    //     ExtendStatus::Trapped
    // }

    //
    // fn connect<FF>(&mut self, q_target: &[N], extend_length: N, is_free: &mut FF) -> ExtendStatus
    // where
    //     FF: FnMut(&[N]) -> bool,
    // {
    //     loop {
    //         debug!("connecting...{q_target:?}");
    //         match self.extend(q_target, extend_length, is_free) {
    //             ExtendStatus::Trapped => return ExtendStatus::Trapped,
    //             ExtendStatus::Reached(index) => return ExtendStatus::Reached(index),
    //             ExtendStatus::Advanced(_) => {}
    //         };
    //     }
    // }

    fn get_until_root(&self, index: usize) -> Vec<Vec<N>> {
        let mut nodes = Vec::new();
        let mut cur_index = index;
        while let Some(parent_index) = self.vertices[cur_index].parent_index {
            cur_index = parent_index;
            nodes.push(self.vertices[cur_index].data.clone())
        }
        nodes
    }

    // Get indices of nerest nodes within a radius
    fn get_nearest_neighbours(&self, q_new: &[N], extend_length: N) -> Vec<usize> {
        self.kdtree
            .within(q_new, extend_length.powi(2), &squared_euclidean)
            .unwrap_or(vec![])
            .iter()
            .map(|(_, index)| **index)
            .collect()
    }
}

/// RRT* error
#[derive(Debug, derive_more::Error, derive_more::Display)]
pub enum RRTStarError {
    /// Failed to find a path within the maximum number of iterations
    #[display(fmt = "Failed to find a path within the maximum number of iterations")]
    MaxItersReached,
}

// pub type RRTStarResult<N> = Result<Vec<Vec<N>>, RRTStarError>;
/// This is the return type for rrtstar
pub type RRTStarResult<N, W> = Result<Tree<N, W>, RRTStarError>;

/// search the path from start to goal which is free, using random_sample function
/// https://erc-bpgc.github.io/handbook/automation/PathPlanners/Sampling_Based_Algorithms/RRT_Star/
pub fn rrtstar<N>(
    start: &[N],
    goal: &[N],
    mut is_free: impl FnMut(&[N]) -> bool,
    random_sample: impl Fn() -> Vec<N>,
    extend_length: N,
    max_iters: usize,
) -> RRTStarResult<N, f32>
// ) -> Result<Vec<Vec<N>>, RRTStarError>
where
    // FF: FnMut(&[N]) -> bool,
    // FR: Fn() -> Vec<N>,
    N: Float + Debug,
    // W: Weight,
{
    assert_eq!(start.len(), goal.len());
    let mut tree = Tree::<N, f32>::new(start.len());
    tree.add_vertex(start, 0.0);

    // Path finding loop
    for _ in 0..max_iters {
        // 1. Random sample
        let q_rand = random_sample();
        // 2. Nearest neighbour
        let nearest_index = tree.get_nearest_index(&q_rand);
        let q_nearest = &tree.vertices[nearest_index].data;
        // 3. Steer to get new point
        let diff_dist = squared_euclidean(q_rand.as_slice(), q_nearest.as_slice()).sqrt();
        let q_new = if diff_dist < extend_length {
            q_rand.to_vec()
        } else {
            q_nearest
                .iter()
                .zip(q_rand)
                .map(|(near, target)| *near + (target - *near) * extend_length / diff_dist)
                .collect::<Vec<_>>()
        };

        // 4. Check if the new point is free
        if !is_free(&q_new) {
            continue;
        }

        // 5. Connect to the new point
        // 5.1. Find nearest neighbours
        let nearest = tree.get_nearest_neighbours(&q_new, extend_length);
        // 5.2. Insert the new point to the tree
        let parent_weight = tree.vertices[nearest_index].weight;
        let edge_weight = <f32 as num_traits::cast::NumCast>::from::<N>(extend_length)
            .expect("N implements Float, same as W");
        let cost_min = parent_weight + edge_weight;
        // + num_traits::cast::NumCast::from::<N>(extend_length)
        // .expect("N implements Float, same as W");

        let new_index = tree.add_vertex(&q_new, cost_min);
        // 5.3. Connect to lowest cost path
        let min_index = std::iter::once(&new_index)
            .chain(nearest.iter())
            .min_by(|&a, &b| {
                tree.vertices[*a]
                    .weight
                    .partial_cmp(&tree.vertices[*b].weight)
                    .unwrap()
            })
            .expect("iterator not empty");
        tree.add_edge(*min_index, new_index);

        // 5.4. Rewire
        for &near_index in nearest.iter() {
            let near_weight = tree.vertices[near_index].weight;
            let new_potential_cost = cost_min
                + <f32 as num_traits::cast::NumCast>::from(
                    squared_euclidean(&q_new, &tree.vertices[near_index].data).sqrt(),
                )
                .expect("N implements Float, same as W");

            if new_potential_cost < near_weight {
                tree.remove_edge(near_index);
                tree.add_edge(new_index, near_index);
                tree.vertices[near_index].weight = new_potential_cost;
            }
        }

        // 6. Check if the goal is reached
        if squared_euclidean(&q_new, goal).sqrt() < extend_length {
            let goal_index = tree.add_vertex(goal, 0.0);
            tree.add_edge(new_index, goal_index);
            // let mut path = tree.get_until_root(goal_index);
            // path.push(goal.to_vec());
            // return Ok(path);
            return Ok(tree);
        }
    }

    Err(RRTStarError::MaxItersReached)
}

/// select random two points, and try to connect.
pub fn smooth_path<FF, N>(
    path: &mut Vec<Vec<N>>,
    mut is_free: FF,
    extend_length: N,
    num_max_try: usize,
) where
    FF: FnMut(&[N]) -> bool,
    N: Float + Debug,
{
    if path.len() < 3 {
        return;
    }
    let mut rng = rand::thread_rng();
    for _ in 0..num_max_try {
        let range1 = Uniform::new(0, path.len() - 2);
        let ind1 = range1.sample(&mut rng);
        let range2 = Uniform::new(ind1 + 2, path.len());
        let ind2 = range2.sample(&mut rng);
        let mut base_point = path[ind1].clone();
        let point2 = path[ind2].clone();
        let mut is_searching = true;
        while is_searching {
            let diff_dist = squared_euclidean(&base_point, &point2).sqrt();
            if diff_dist < extend_length {
                // reached!
                // remove path[ind1+1] ... path[ind2-1]
                let remove_index = ind1 + 1;
                for _ in 0..(ind2 - ind1 - 1) {
                    path.remove(remove_index);
                }
                if path.len() == 2 {
                    return;
                }
                is_searching = false;
            } else {
                let check_point = base_point
                    .iter()
                    .zip(point2.iter())
                    .map(|(near, target)| *near + (*target - *near) * extend_length / diff_dist)
                    .collect::<Vec<_>>();
                if !is_free(&check_point) {
                    // trapped
                    is_searching = false;
                } else {
                    // continue to extend
                    base_point = check_point;
                }
            }
        }
    }
}

#[test]
fn it_works() {
    use rand::distributions::{Distribution, Uniform};
    let mut result = rrtstar(
        &[-1.2, 0.0],
        &[1.2, 0.0],
        |p: &[f64]| !(p[0].abs() < 1.0 && p[1].abs() < 1.0),
        || {
            let between = Uniform::new(-2.0, 2.0);
            let mut rng = rand::thread_rng();
            vec![between.sample(&mut rng), between.sample(&mut rng)]
        },
        0.2,
        1000,
    )
    .unwrap();
    println!("{result:?}");
    // assert!(result.len() >= 4);
    // smooth_path(
    //     &mut result,
    //     |p: &[f64]| !(p[0].abs() < 1.0 && p[1].abs() < 1.0),
    //     0.2,
    //     100,
    // );
    // println!("{result:?}");
    // assert!(result.len() >= 3);
}
