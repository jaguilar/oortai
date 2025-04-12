use crate::control::*;
use maths_rs::*;
use oort_api::prelude::*;

pub struct Ship {}

const BULLET_SPEED: f64 = 1000.0; // m/s

impl Ship {
    pub fn new() -> Ship {
        Ship {}
    }

    pub fn tick(&mut self) {
        draw_line(position(), target(), 0x00ff00);

        if let Some(a) = lead3(target(), target_velocity(), vec2(0., 0.), BULLET_SPEED) {
            draw_line(position(), a, 0xff0000);
            if aim_at_pos(a, BULLET_SPEED) {
                fire(0);
            }
        }
    }
}
