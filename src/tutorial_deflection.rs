use crate::control::*;
use oort_api::prelude::{maths_rs::{num::Base, *}, *};

pub struct Ship {
    helm: Helm,
    v_prev: Vec2,
}

const BULLET_SPEED: f64 = 1000.0; // m/s

impl Ship {
    pub fn new() -> Ship {
        Ship {
            helm: Helm::new(),
            v_prev: Vec2::zero(),
        }
    }

    pub fn tick(&mut self) {
        let target_acceleration = (target_velocity() - self.v_prev) / TICK_LENGTH;
        self.v_prev = target_velocity();
        turn_and_shoot_at(&mut self.helm, target(), target_velocity(), target_acceleration, 1000.0);
    }
}