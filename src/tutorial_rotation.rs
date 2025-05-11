// Tutorial: Rotation
// Destroy the asteroid. The target is in a random
// location given by the "target()" function.
//
// You can get the angle between a vector and the x-axis: vec2(a, b).angle()
// And compare an angle with your ship's heading: angle_diff(heading(), angle)
//
// If angle_diff returns a positive number you need to turn left, or if negative then right.
// The turn function takes a speed argument, where positive speeds result in turning left
// and negative speeds will turn right.
use crate::control::*;
use oort_api::prelude::{maths_rs::{num::Base, *}, *};

pub struct Ship {
    helm: Helm,
}

impl Ship {
    pub fn new() -> Ship {
        Ship {
            helm: Helm::new(),
        }
    }

    pub fn tick(&mut self) {
        turn_and_shoot_at(&mut self.helm, target(), Vec2::zero(), Vec2::zero(), 1000.0);
    }
}
