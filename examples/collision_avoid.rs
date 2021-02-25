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

use nalgebra as na;

use kiss3d::light::Light;
use kiss3d::window::Window;
use na::{Isometry3, Vector3};
use ncollide3d::query;
use ncollide3d::query::Proximity;
use ncollide3d::shape::{Ball, Cuboid};

use rand::distributions::{Distribution, Uniform};

struct CollisionProblem {
    obstacle: Cuboid<f32>,
    ball: Ball<f32>,
}

impl CollisionProblem {
    fn is_feasible(&self, point: &[f64]) -> bool {
        let cuboid_pos = Isometry3::new(Vector3::new(0.0f32, 0.0, 0.0), na::zero());
        let ball_pos = Isometry3::new(
            Vector3::new(point[0] as f32, point[1] as f32, point[2] as f32),
            na::zero(),
        );
        let prediction = 0.1;
        let ctct = query::proximity(
            &ball_pos,
            &self.ball,
            &cuboid_pos,
            &self.obstacle,
            prediction,
        );
        ctct == Proximity::Disjoint
    }
    fn random_sample(&self) -> Vec<f64> {
        let between = Uniform::new(-4.0, 4.0);
        let mut rng = rand::thread_rng();
        vec![
            between.sample(&mut rng),
            between.sample(&mut rng),
            between.sample(&mut rng),
        ]
    }
}

fn main() {
    let mut window = Window::new("rrt test");
    window.set_light(Light::StickToCamera);

    let p = CollisionProblem {
        obstacle: Cuboid::new(Vector3::new(0.05f32, 0.25, 0.15)),
        ball: Ball::new(0.05f32),
    };
    let mut c1 = window.add_cube(
        p.obstacle.half_extents[0] * 2.0,
        p.obstacle.half_extents[1] * 2.0,
        p.obstacle.half_extents[2] * 2.0,
    );
    c1.set_color(1.0, 0.0, 0.0);

    let mut cs = window.add_cube(0.05, 0.05, 0.05);
    cs.set_color(0.0, 0.0, 1.0);
    let mut cg = window.add_cube(0.05, 0.05, 0.05);
    cg.set_color(0.0, 1.0, 0.0);

    let mut c2 = window.add_sphere(p.ball.radius);
    c2.set_color(0.0, 1.0, 1.0);
    let start = [0.2f64, 0.2, 0.2];
    let goal = [-0.2f64, -0.2, -0.2];
    let poss = Isometry3::new(
        Vector3::new(start[0] as f32, start[1] as f32, start[2] as f32),
        na::zero(),
    );
    let posg = Isometry3::new(
        Vector3::new(goal[0] as f32, goal[1] as f32, goal[2] as f32),
        na::zero(),
    );

    cs.set_local_transformation(poss);
    cg.set_local_transformation(posg);
    let mut path = vec![];
    let mut index = 0;
    while window.render() {
        if index == path.len() {
            path = rrt::dual_rrt_connect(
                &start,
                &goal,
                |x: &[f64]| p.is_feasible(x),
                || p.random_sample(),
                0.05,
                1000,
            )
            .unwrap();
            rrt::smooth_path(&mut path, |x: &[f64]| p.is_feasible(x), 0.05, 100);
            index = 0;
        }
        let point = &path[index % path.len()];
        let pos = Isometry3::new(
            Vector3::new(point[0] as f32, point[1] as f32, point[2] as f32),
            na::zero(),
        );
        c2.set_local_transformation(pos);
        index += 1;
    }
}
