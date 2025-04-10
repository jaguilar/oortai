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

        if let Some(a) = lead2(target(), target_velocity(), vec2(0., 0.), BULLET_SPEED) {
            let lead_vec = vec2(sin(a), cos(a));
            draw_line(position(), lead_vec, 0xff0000);
            if aim_at_vec(lead_vec) {
                fire(0);
            }
        }
    }
}
