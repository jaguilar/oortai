use std::cmp::min_by;

use maths_rs::*;
use oort_api::prelude::*;

// Returns the position of an entity with the given position, velocity,
// acceleration and acceleration after t seconds have elapsed.
pub fn pos_after(pos: Vec2, vel: Vec2, acc: Vec2, t: f64) -> Vec2 {
    pos + t * (vel + 0.5 * acc * t)
}

// Runs Newton's method on a function and its derivative to find a rational
// root near x0.
//
// f: The function of which to find the root. May return an Option if the answer
//    is not real or otherwise unacceptable (e.g. it's not physically possible).
// df: The derivative of the function.
// x0: The initial guess for the root.
// tol: The tolerance for the root. If None, uses the default tolerance.
// range: The range of x values to search. If we find a flat spot in the
//    function we will resume the search from a random value in this range.
//    Defaults to 0, 2*x0.
fn newtons_method<T: (Fn(f64) -> Option<f64>), U: (Fn(f64) -> f64)>(
    f: &T,
    fp: &U,
    x0: f64,
    tol: Option<f64>,
    range: Option<(f64, f64)>,
) -> Option<f64> {
    let mut x = x0;

    for i in 0..100 {
        if let Some(fx) = f(x) {
            if fx.abs() < tol.unwrap_or(1e-10) {
                return Some(x);
            }

            let fpx = fp(x);
            if fpx.abs() > 1e-10 {
                x -= fx / fpx;
                continue;
            }
        }

        // If f reports None, or if we are in a flat spot (dfx <= tol), then
        // we randomly select a value in the range and try again.
        let rrange = range.unwrap_or((0., 2. * x0));
        x = rand(rrange.0, rrange.1);
        continue;
    }
    None
}

pub fn lead3(e_pos: Vec2, e_vel: Vec2, e_acc: Vec2, b_spd: f64) -> Option<Vec2> {
    let es = e_pos - position();
    let ev = e_vel - velocity();
    let ea = e_acc;

    // Zero at times when the bullet distance from the ship and the enemy
    // distance from the ship are the same.
    let f = |t: f64| -> Option<f64> {
        if t <= 0. {
            return None;
        }
        Some(t * b_spd - (es + t * (ev + t * 0.5 * ea)).length())
    };

    // The derivative of the above function.
    let fp = |t: f64| {
        b_spd - (ea * t + ev).length()
    };

    let x0 = es.length() / b_spd;
    newtons_method(&f, &fp, x0, Some(TICK_LENGTH / 10.), None)
        .and_then(|t: f64| Some(pos_after(e_pos, e_vel, e_acc, t)))
}

// Aims at the entity whose pos, vel, and acc are given. Aim for a gun with the
// given bullet speed.
//
// Returns the absolute angle between our heading and the heading to the
// aim point
pub fn aim_at_entity(pos: Vec2, vel: Vec2, acc: Vec2, bspd: f64) -> Option<Vec2> {
    let a = lead3(pos, vel, acc, bspd)?;
    aim_at_pos(a, bspd);
    Some(a)
}

// Instructs the ship to turn toward target. Returns the distance between the
// target and the line projected from the current heading, in meters.
pub fn aim_at_pos(target: Vec2, bspd: f64) -> bool {
    draw_line(position(), target, 0xff0000);
    draw_diamond(target, 10.0, 0xff0000);

    // We impart our own velocity to the bullet, so it will be displaced from
    // its intended position by our own velocity. Therefore, we must shoot it
    // at the target's current position plus that displacement negated.
    let travel_time = (target - position()).length() / bspd;
    let displacement = travel_time * velocity();
    aim_at_heading(((target - displacement) - position()).angle())
}

pub fn point_at(target: Vec2) {
    aim_at_heading((target - position()).angle());
}

fn aim_at_heading(target_heading: f64) -> bool {
    let error = angle_diff(target_heading, heading());
    let max_accel = if error < 0. {
        -max_angular_acceleration()
    } else {
        max_angular_acceleration()
    };

    // Calculate the maximum acceptable angular velocity (in magnitude) for this
    // tick. This is given by sqrt(2)*sqrt(a*s), where a is the acceleration per
    // unit time and s is the distance remaining at the beginning of the tick.
    let max_vel = copysign(sqrt(2.) * sqrt(max_accel * error), -error);
    let vel_err = max_vel - angular_velocity();
    let req_torque = clamp(
        vel_err / TICK_LENGTH,
        -max_angular_acceleration(),
        max_angular_acceleration(),
    );

    let close = (360. * abs(error) / (2. * PI)) < 1.;

    // Accelerate in the direction of vel_err by vel_err or
    // max_angular_acceleration, whichever is less.
    torque(req_torque);

    close
}
