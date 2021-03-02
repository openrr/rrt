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
//! # RRT
//!
//! RRT (Rapidly-exploring Random Tree) library implemented by rust.
//! Only Dual RRT Connect is supported.
//!
//! ## Examples
//!
//! ```
//! use rand::distributions::{Distribution, Uniform};
//! let result = rrt::dual_rrt_connect(
//!     &[-1.2, 0.0],
//!     &[1.2, 0.0],
//!     |p: &[f64]| !(p[0].abs() < 1.0 && p[1].abs() < 1.0),
//!     || {
//!         let between = Uniform::new(-2.0, 2.0);
//!         let mut rng = rand::thread_rng();
//!         vec![between.sample(&mut rng), between.sample(&mut rng)]
//!     },
//!     0.2,
//!     1000,
//! )
//! .unwrap();
//! println!("{:?}", result);
//! assert!(result.len() >= 4);
//! ```

use kdtree::distance::squared_euclidean;
use log::info;
use num_traits::float::Float;
use num_traits::identities::Zero;
use rand::distributions::{Distribution, Uniform};
use std::fmt::Debug;
use std::mem;

pub enum ExtendStatus {
    Reached(usize),
    Advanced(usize),
    Trapped,
}

/// Node that contains user data
#[derive(Debug, Clone)]
pub struct Node<T> {
    parent_id: Option<usize>,
    id: usize,
    data: T,
}

impl<T> Node<T> {
    pub fn new(data: T, id: usize) -> Self {
        Node {
            parent_id: None,
            id,
            data,
        }
    }
}

/// RRT
#[derive(Debug)]
pub struct Tree<N>
where
    N: Float + Zero + Debug,
{
    pub dim: usize,
    pub kdtree: kdtree::KdTree<N, usize, Vec<N>>,
    pub vertices: Vec<Node<Vec<N>>>,
    pub name: String,
}

impl<N> Tree<N>
where
    N: Float + Zero + Debug,
{
    pub fn new(name: &str, dim: usize) -> Self {
        Tree {
            dim,
            kdtree: kdtree::KdTree::new(dim),
            vertices: Vec::new(),
            name: name.to_string(),
        }
    }
    pub fn add_vertex(&mut self, q: &[N]) -> usize {
        let id = self.vertices.len();
        self.kdtree.add(q.to_vec(), id).unwrap();
        self.vertices.push(Node::new(q.to_vec(), id));
        id
    }
    pub fn add_edge(&mut self, q1_id: usize, q2_id: usize) {
        self.vertices[q2_id].parent_id = Some(q1_id);
    }
    pub fn get_nearest_id(&self, q: &[N]) -> usize {
        *self.kdtree.nearest(q, 1, &squared_euclidean).unwrap()[0].1
    }
    pub fn extend<FF>(&mut self, q_target: &[N], extend_length: N, is_free: &mut FF) -> ExtendStatus
    where
        FF: FnMut(&[N]) -> bool,
    {
        assert!(extend_length > N::zero());
        let nearest_id = self.get_nearest_id(q_target);
        let nearest_q = self.vertices[nearest_id].data.clone();
        let diff_dist = squared_euclidean(q_target, &nearest_q).sqrt();
        let q_new = if diff_dist < extend_length {
            q_target.to_vec()
        } else {
            nearest_q
                .into_iter()
                .zip(q_target.iter())
                .map(|(near, target)| near + (*target - near) * extend_length / diff_dist)
                .collect::<Vec<_>>()
        };
        info!("q_new={:?}", q_new);
        if is_free(&q_new) {
            let new_id = self.add_vertex(&q_new);
            self.add_edge(nearest_id, new_id);
            if squared_euclidean(&q_new, q_target).sqrt() < extend_length {
                return ExtendStatus::Reached(new_id);
            }
            info!("target = {:?}", q_target);
            info!("advaneced to {:?}", q_target);
            return ExtendStatus::Advanced(new_id);
        }
        ExtendStatus::Trapped
    }
    pub fn connect<FF>(
        &mut self,
        q_target: &[N],
        extend_length: N,
        is_free: &mut FF,
    ) -> ExtendStatus
    where
        FF: FnMut(&[N]) -> bool,
    {
        loop {
            info!("connecting...{:?}", q_target);
            match self.extend(q_target, extend_length, is_free) {
                ExtendStatus::Trapped => return ExtendStatus::Trapped,
                ExtendStatus::Reached(id) => return ExtendStatus::Reached(id),
                ExtendStatus::Advanced(_) => {}
            };
        }
    }
    pub fn get_until_root(&self, id: usize) -> Vec<Vec<N>> {
        let mut nodes = Vec::new();
        let mut cur_id = id;
        while let Some(parent_id) = self.vertices[cur_id].parent_id {
            cur_id = parent_id;
            nodes.push(self.vertices[cur_id].data.clone())
        }
        nodes
    }
}

/// search the path from start to goal which is free, using random_sample function
pub fn dual_rrt_connect<FF, FR, N>(
    start: &[N],
    goal: &[N],
    mut is_free: FF,
    random_sample: FR,
    extend_length: N,
    num_max_try: usize,
) -> Result<Vec<Vec<N>>, String>
where
    FF: FnMut(&[N]) -> bool,
    FR: Fn() -> Vec<N>,
    N: Float + Debug,
{
    assert_eq!(start.len(), goal.len());
    let mut tree_a = Tree::new("start", start.len());
    let mut tree_b = Tree::new("goal", start.len());
    tree_a.add_vertex(start);
    tree_b.add_vertex(goal);
    for _ in 0..num_max_try {
        info!("tree_a = {:?}", tree_a.vertices.len());
        info!("tree_b = {:?}", tree_b.vertices.len());
        let q_rand = random_sample();
        let extend_status = tree_a.extend(&q_rand, extend_length, &mut is_free);
        match extend_status {
            ExtendStatus::Trapped => {}
            ExtendStatus::Advanced(new_id) | ExtendStatus::Reached(new_id) => {
                let q_new = tree_a.vertices[new_id].data.clone();
                if let ExtendStatus::Reached(reach_id) =
                    tree_b.connect(&q_new, extend_length, &mut is_free)
                {
                    let mut a_all = tree_a.get_until_root(new_id);
                    let mut b_all = tree_b.get_until_root(reach_id);
                    a_all.reverse();
                    a_all.append(&mut b_all);
                    if tree_b.name == "start" {
                        a_all.reverse();
                    }
                    return Ok(a_all);
                }
            }
        }
        mem::swap(&mut tree_a, &mut tree_b);
    }
    Err("failed".to_string())
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
    let mut result = dual_rrt_connect(
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
    println!("{:?}", result);
    assert!(result.len() >= 4);
    smooth_path(
        &mut result,
        |p: &[f64]| !(p[0].abs() < 1.0 && p[1].abs() < 1.0),
        0.2,
        100,
    );
    println!("{:?}", result);
    assert!(result.len() >= 3);
}
