#![cfg_attr(feature="clippy", feature(plugin))]
#![cfg_attr(feature="clippy", plugin(clippy))]
extern crate kiss3d;
extern crate nalgebra as na;
extern crate ncollide;
extern crate rand;
extern crate rrt;

use kiss3d::window::Window;
use kiss3d::light::Light;
use na::{Isometry3, Vector3};
use ncollide::shape::{Cuboid, Ball};
use ncollide::query;
use ncollide::ncollide_geometry::query::Proximity;

use rand::distributions::{IndependentSample, Range};

use rrt::dual_rrt_connect;

struct CollisionProblem {
    obstacle: Cuboid<na::Vector3<f32>>,
    ball: Ball<f32>,
}

impl CollisionProblem {
    fn is_feasible(&self, point: &[f64]) -> bool {
        let cuboid_pos = Isometry3::new(Vector3::new(0.0f32, 0.0, 0.0), na::zero());
        let ball_pos =
            Isometry3::new(Vector3::new(point[0] as f32, point[1] as f32, point[2] as f32),
                           na::zero());
        let prediction = 0.1;
        let ctct = query::proximity(&ball_pos,
                                    &self.ball,
                                    &cuboid_pos,
                                    &self.obstacle,
                                    prediction);
        ctct == Proximity::Disjoint
    }
    fn random_sample(&self) -> Vec<f64> {
        let between = Range::new(-4.0, 4.0);
        let mut rng = rand::thread_rng();
        vec![between.ind_sample(&mut rng),
             between.ind_sample(&mut rng),
             between.ind_sample(&mut rng)]
    }
}

fn main() {

    let mut window = Window::new("rrt test");
    window.set_light(Light::StickToCamera);

    let p = CollisionProblem {
        obstacle: Cuboid::new(Vector3::new(0.05f32, 0.25, 0.15)),
        ball: Ball::new(0.05f32),
    };
    let mut c1 = window.add_cube(p.obstacle.half_extents()[0] * 2.0,
                                 p.obstacle.half_extents()[1] * 2.0,
                                 p.obstacle.half_extents()[2] * 2.0);
    c1.set_color(1.0, 0.0, 0.0);

    let mut cs = window.add_cube(0.05, 0.05, 0.05);
    cs.set_color(0.0, 0.0, 1.0);
    let mut cg = window.add_cube(0.05, 0.05, 0.05);
    cg.set_color(0.0, 1.0, 0.0);

    let mut c2 = window.add_sphere(p.ball.radius());
    c2.set_color(0.0, 1.0, 1.0);
    let start = [0.2f64, 0.2, 0.2];
    let goal = [-0.2f64, -0.2, -0.2];
    let poss = Isometry3::new(Vector3::new(start[0] as f32, start[1] as f32, start[2] as f32),
                              na::zero());
    let posg = Isometry3::new(Vector3::new(goal[0] as f32, goal[1] as f32, goal[2] as f32),
                              na::zero());

    cs.set_local_transformation(poss);
    cg.set_local_transformation(posg);
    let mut path = vec![];
    let mut index = 0;
    while window.render() {
        if index == path.len() {
            path = dual_rrt_connect(&start,
                                    &goal,
                                    |x: &[f64]| p.is_feasible(x),
                                    || p.random_sample(),
                                    0.05,
                                    1000)
                    .unwrap();
            index = 0;
        }
        let point = &path[index % path.len()];
        let pos = Isometry3::new(Vector3::new(point[0] as f32, point[1] as f32, point[2] as f32),
                                 na::zero());
        c2.set_local_transformation(pos);
        index += 1;
    }
}
