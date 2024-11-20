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
use rand::{
    distributions::{Distribution, Uniform},
    RngCore,
};
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
pub struct Node<T, W: Weight> {
    pub parent_index: Option<usize>,
    pub data: T,
    pub weight: W,
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
    /// kdtree data structure to store the nodes
    /// for fast nearest neighbour search
    pub kdtree: kdtree::KdTree<N, usize, Vec<N>>,
    /// Vertices of the tree
    pub vertices: Vec<Node<Vec<N>, W>>,
    /// The goal index
    pub goal_index: Option<usize>,
}

// impl default for Tree
impl<N, W> Default for Tree<N, W>
where
    N: Float + Zero + Debug,
    W: Weight,
{
    fn default() -> Self {
        Tree {
            kdtree: kdtree::KdTree::new(2),
            vertices: Vec::new(),
            goal_index: None,
        }
    }
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
            goal_index: None,
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

    /// Get the path from the root to the node
    pub fn get_until_root(&self, index: usize) -> Vec<Vec<N>> {
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
    mut is_collision_free: impl FnMut(&[N]) -> bool,
    mut random_sample: impl FnMut() -> Vec<N>,
    extend_length: N,
    max_iters: usize,
    neighbourhood_radius: N,
    stop_when_reach_goal: bool,
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

    let mut goal_reached = false;

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
        if !is_collision_free(&q_new) {
            continue;
        }

        // 5. Connect to the new point
        // 5.1. Find nearest neighbours
        let nearest = tree.get_nearest_neighbours(&q_new, neighbourhood_radius);
        // 5.2. Insert the new point to the tree
        let parent_weight = tree.vertices[nearest_index].weight;
        let edge_weight = <f32 as num_traits::cast::NumCast>::from::<N>(extend_length)
            .expect("N implements Float, same as W");
        let cost_min = parent_weight + edge_weight;

        let new_index = tree.add_vertex(&q_new, cost_min);
        // 5.3. Connect to lowest cost path
        let min_index = std::iter::once(&nearest_index)
            .chain(nearest.iter())
            .min_by(|&a, &b| {
                let a_potential_weight = tree.vertices[*a].weight
                    + <f32 as num_traits::cast::NumCast>::from(
                        squared_euclidean(&q_new, &tree.vertices[*a].data).sqrt(),
                    )
                    .expect("N implements Float, same as W");

                let b_potential_weight = tree.vertices[*b].weight
                    + <f32 as num_traits::cast::NumCast>::from(
                        squared_euclidean(&q_new, &tree.vertices[*b].data).sqrt(),
                    )
                    .expect("N implements Float, same as W");

                a_potential_weight
                    .partial_cmp(&b_potential_weight)
                    .expect("Weight W of two nodes should be comparable")
            })
            .expect("iterator shouldn't be empty");

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
        if !goal_reached && squared_euclidean(&q_new, goal).sqrt() < extend_length {
            let goal_weight = tree.vertices[new_index].weight
                + <f32 as num_traits::cast::NumCast>::from(squared_euclidean(&q_new, goal).sqrt())
                    .expect("N implements Float, same as W");
            // println!("goal {:?} reached with weight {}", goal, goal_weight);
            let goal_index = tree.add_vertex(goal, goal_weight);
            tree.add_edge(new_index, goal_index);

            tree.goal_index = Some(goal_index);

            goal_reached = true;

            if stop_when_reach_goal {
                return Ok(tree);
            }
        }
    }

    if !stop_when_reach_goal {
        return Ok(tree);
    } else {
        Err(RRTStarError::MaxItersReached)
    }
}

/// select random two points, and try to connect.
pub fn smooth_path<FF, N>(
    path: &mut Vec<Vec<N>>,
    mut is_free: FF,
    extend_length: N,
    num_max_try: usize,
    mut rng: &mut dyn RngCore,
) where
    FF: FnMut(&[N]) -> bool,
    N: Float + Debug,
{
    if path.len() < 3 {
        return;
    }
    // let mut rng = rand::thread_rng();
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
