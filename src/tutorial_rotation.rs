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
use oort_api::prelude::{maths_rs::*, *};

pub struct Ship {}

impl Ship {
    pub fn new() -> Ship {
        Ship {}
    }

    pub fn tick(&mut self) {
        if !aim_at_vec(target()) {
            return;
        }
        // We're close to the target. See if the line projected from our ship
        // comes within one meter. If it does, fire.
        let aim_to_target =
            (closest_point_on_ray(target(), position(), vec2(cos(heading()), sin(heading())))
                - target())
            .length();
        debug!("aim_to_target: {:.2}", aim_to_target);
        if aim_to_target < 10. {
            fire(0);
        }
    }
}
