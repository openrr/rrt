#![cfg_attr(feature="clippy", feature(plugin))]
#![cfg_attr(feature="clippy", plugin(clippy))]

extern crate rand;
extern crate kdtree;
#[macro_use]
extern crate log;

use kdtree::distance::squared_euclidean;
use std::mem;

pub enum ExtendStatus {
    Reached(usize),
    Advanced(usize),
    Trapped,
}

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
            id: id,
            data: data,
        }
    }
}

#[derive(Debug)]
pub struct Tree {
    pub dim: usize,
    pub kdtree: kdtree::KdTree<usize, Vec<f64>>,
    pub vertices: Vec<Node<Vec<f64>>>,
    pub name: String,
}

impl Tree {
    pub fn new(name: &str, dim: usize) -> Self {
        Tree {
            dim: dim,
            kdtree: kdtree::KdTree::new(dim),
            vertices: Vec::new(),
            name: name.to_string(),
        }
    }
    pub fn add_vertex(&mut self, q: &[f64]) -> usize {
        let id = self.vertices.len();
        self.kdtree.add(q.to_vec(), id).unwrap();
        self.vertices.push(Node::new(q.to_vec(), id));
        id
    }
    pub fn add_edge(&mut self, q1_id: usize, q2_id: usize) {
        self.vertices[q2_id].parent_id = Some(q1_id);
    }
    pub fn get_nearest_id(&self, q: &[f64]) -> usize {
        *self.kdtree.nearest(q, 1, &squared_euclidean).unwrap()[0].1
    }
    pub fn extend<FF>(&mut self,
                      q_target: &[f64],
                      extend_length: f64,
                      is_feasible: &FF)
                      -> ExtendStatus
        where FF: Fn(&[f64]) -> bool
    {
        assert!(extend_length > 0.0);
        let nearest_id = self.get_nearest_id(q_target);
        let nearest_q = self.vertices[nearest_id].data.clone();
        let diff_dist = squared_euclidean(q_target, &nearest_q).sqrt();
        let q_new = if diff_dist < extend_length {
            q_target.to_vec()
        } else {
            nearest_q
                .iter()
                .zip(q_target.iter())
                .map(|(near, target)| near + (target - near) * extend_length / diff_dist)
                .collect::<Vec<_>>()
        };
        info!("q_new={:?}", q_new);
        if is_feasible(&q_new) {
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
    pub fn connect<FF>(&mut self,
                       q_target: &[f64],
                       extend_length: f64,
                       is_feasible: &FF)
                       -> ExtendStatus
        where FF: Fn(&[f64]) -> bool
    {
        loop {
            info!("connecting...{:?}", q_target);
            match self.extend(q_target, extend_length, is_feasible) {
                ExtendStatus::Trapped => return ExtendStatus::Trapped,
                ExtendStatus::Reached(id) => return ExtendStatus::Reached(id),
                ExtendStatus::Advanced(_) => {}
            };
        }
    }
    pub fn get_until_root(&self, id: usize) -> Vec<Vec<f64>> {
        let mut nodes = Vec::new();
        let mut cur_id = id;
        while let Some(parent_id) = self.vertices[cur_id].parent_id {
            cur_id = parent_id;
            nodes.push(self.vertices[cur_id].data.clone())
        }
        nodes
    }
}

pub fn dual_rrt_connect<FF, FR>(start: &[f64],
                                goal: &[f64],
                                is_feasible: FF,
                                random_sample: FR,
                                extend_length: f64,
                                num_max_try: usize)
                                -> Result<Vec<Vec<f64>>, String>
    where FF: Fn(&[f64]) -> bool,
          FR: Fn() -> Vec<f64>
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
        let extend_status = tree_a.extend(&q_rand, extend_length, &is_feasible);
        match extend_status {
            ExtendStatus::Trapped => {}
            ExtendStatus::Advanced(new_id) |
            ExtendStatus::Reached(new_id) => {
                let q_new = tree_a.vertices[new_id].data.clone();
                if let ExtendStatus::Reached(reach_id) =
                    tree_b.connect(&q_new, extend_length, &is_feasible) {
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

#[test]
fn it_works() {
    extern crate env_logger;
    use rand::distributions::{IndependentSample, Range};

    pub struct BoxProblem {}
    impl BoxProblem {
        fn is_feasible(&self, point: &[f64]) -> bool {
            !(point[0].abs() < 1.0 && point[1].abs() < 1.0)
        }
        fn random_sample(&self) -> Vec<f64> {
            let between = Range::new(-2.0, 2.0);
            let mut rng = rand::thread_rng();
            vec![between.ind_sample(&mut rng), between.ind_sample(&mut rng)]
        }
    }

    let p = BoxProblem {};
    let start = [-1.2, 0.0];
    let goal = [1.2, 0.0];
    assert!(p.is_feasible(&start));
    assert!(p.is_feasible(&goal));
    let result = dual_rrt_connect(&start,
                                  &goal,
                                  |x: &[f64]| p.is_feasible(x),
                                  || p.random_sample(),
                                  0.5,
                                  1000)
            .unwrap();
    println!("{:?}", result);
    assert!(result.len() >= 4);
}
