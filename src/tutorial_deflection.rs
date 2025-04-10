use crate::control::*;
use maths_rs::*;
use oort_api::prelude::*;

pub struct Ship {
    last_v: Vec2,
}

const BULLET_SPEED: f64 = 1000.0; // m/s

impl Ship {
    pub fn new() -> Ship {
        Ship {
            last_v: vec2(0., 0.),
        }
    }

    pub fn tick(&mut self) {
        let target_acc = (target_velocity() - self.last_v) / TICK_LENGTH;
        self.last_v = target_velocity();

        if let Some(err) = aim_at_entity(target(), target_velocity(), target_acc, BULLET_SPEED) {
            if err < 0.4 / 360. * 2. * PI {
                fire(0)
            }
        }
    }
}
