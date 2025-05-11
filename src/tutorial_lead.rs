use crate::control::*;
use oort_api::prelude::{maths_rs::{num::Base, *}, *};

pub struct Ship {
    helm: Helm,
}

const BULLET_SPEED: f64 = 1000.0; // m/s

impl Ship {
    pub fn new() -> Ship {
        Ship {
            helm: Helm::new(),
        }
    }

    pub fn tick(&mut self) {
        turn_and_shoot_at(&mut self.helm, target(), target_velocity(), Vec2::zero(), 1000.0);
    }
}