use std::cmp::min_by;

use maths_rs::*;
use oort_api::prelude::*;

// Calculates the lead angle needed to hit a target given the position and
// velocity of the target and the shooting ship. The angle is returned in
// radians, where zero means "fire directly at the enemy" and positive
// radians indicate to turn clockwise. If there are no solutions, returns
// Nothing. If there are two solutions, picks the solution that has the
// shortest travel time.
pub fn lead(e_pos: Vec2, e_vel: Vec2, b_spd: f64) -> Option<f64> {
    // This implementation uses the law of sines to calculate the angle from
    // our ship to the position where the bullet and the enemy collide.
    // An explanation can be found here:
    // https://www.youtube.com/watch?v=F0eJXk8ZuTk

    // Convert the enemy's speed vector to the reference frame of our ship.
    let e_vel_rel = e_vel - velocity();

    // Get the enemy's look angle to us (angle between enemy's travel direction
    // relative to us and vector from enemy to us).
    let e_to_me = position() - e_pos;
    let look_angle = angle_diff(e_vel_rel.angle(), e_to_me.angle());

    // Compute each possible solution.
    let e_spd = e_vel_rel.length();
    let inner = e_spd * sin(look_angle) / b_spd;
    if inner < -1. || inner > 1. {
        return None;
    }

    let asind = asin(inner);
    let sol1 = asind;
    let sol2 = PI - asind;
    if abs(sol1) < abs(sol2) {
        return Some(sol1);
    } else {
        return Some(sol2);
    }
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

    for _ in 0..100 {
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
    let e_rpos = e_pos - position();
    let e_rvel = e_vel - velocity();

    let esx = e_rpos.x;
    let esy = e_rpos.y;
    let evx = e_rvel.x;
    let evy = e_rvel.y;
    let eax = e_acc.x;
    let eay = e_acc.y;

    let f = |t: f64| -> Option<f64> {
        if t <= 0. {
            return None;
        }
        Some(
            -esx.powi(2) - esy.powi(2)
                + t.powi(4) * (-0.25 * eax.powi(2) - 0.25 * eay.powi(2))
                + t.powi(3) * (-1.0 * eax * evx - 1.0 * eay * evy)
                + t.powi(2)
                    * (b_spd.powi(2)
                        - 1.0 * eax * esx
                        - 1.0 * eay * esy
                        - evx.powi(2)
                        - evy.powi(2))
                + t * (-2. * esx * evx - 2. * esy * evy),
        )
    };
    let fp = |t: f64| {
        t.powi(3) * (-eax.powi(2) - eay.powi(2))
            + t.powi(2) * 3. * (-eax * evx - eay * evy)
            + t * 2.
                * (b_spd.powi(2) - 1.0 * eax * esx - 1.0 * eay * esy - evx.powi(2) - evy.powi(2))
            + (-2. * esx * evx - 2. * esy * evy)
    };

    let x0 = e_rpos.length() / b_spd;
    newtons_method(&f, &fp, x0, Some(1e-5), None)
        .and_then(|t: f64| Some(position() + e_rpos + e_rvel * t + 0.5 * e_acc * t.powi(2)))
}

pub fn lead2(e_pos: Vec2, e_vel: Vec2, e_acc: Vec2, b_spd: f64) -> Option<f64> {
    let e_rpos = e_pos - position();
    let e_rvel = e_vel - velocity();

    match lead2_raw(
        b_spd, e_rpos.x, e_rpos.y, e_rvel.x, e_rvel.y, e_acc.x, e_acc.y,
    ) {
        (None, None) => None,
        (Some(th), None) => Some(th),
        (None, Some(th)) => Some(th),
        (Some(th1), Some(th2)) => {
            // When we have two workable solutions, we pick the one that
            // is closer to the current heading to the enemy. It's a hack to
            // try to estimate which hits soonest without doing all the math.
            let ehdg = e_rpos.angle();
            if abs(angle_diff(th1, ehdg)) < abs(angle_diff(th2, ehdg)) {
                Some(th1)
            } else {
                Some(th2)
            }
        }
    }
}

fn lead2_raw(
    bspd: f64,
    esx: f64,
    esy: f64,
    evx: f64,
    evy: f64,
    eax: f64,
    eay: f64,
) -> (Option<f64>, Option<f64>) {
    let inner = 2.82842712474619
        * (0.5 * bspd.powi(2) * esx.powi(2) + 0.5 * bspd.powi(2) * esy.powi(2)
            - 0.125 * eax.powi(2) * esy.powi(2)
            + 0.25 * eax * eay * esx * esy
            + 0.5 * eax * esx * esy * evy
            - 0.5 * eax * esy.powi(2) * evx
            - 0.125 * eay.powi(2) * esx.powi(2)
            - 0.5 * eay * esx.powi(2) * evy
            + 0.5 * eay * esx * esy * evx
            - 0.5 * esx.powi(2) * evy.powi(2)
            + esx * esy * evx * evy
            - 0.5 * esy.powi(2) * evx.powi(2))
        .sqrt();
    let denom = 2.0 * bspd * esx - eax * esy + eay * esx + 2.0 * esx * evy - 2.0 * esy * evx;
    let out1 = -2.0 * ((2.0 * bspd * esy - inner) / denom).atan();
    let out2 = -2.0 * ((2.0 * bspd * esy + inner) / denom).atan();
    (
        if f64::is_nan(out1) { None } else { Some(out1) },
        if f64::is_nan(out2) { None } else { Some(out2) },
    )
}

// Aims at the entity whose pos, vel, and acc are given. Aim for a gun with the
// given bullet speed.
//
// Returns the absolute angle between our heading and the heading to the
// aim point
pub fn aim_at_entity(pos: Vec2, vel: Vec2, acc: Vec2, bspd: f64) -> Option<f64> {
    let a = lead3(pos, vel, acc, bspd)?;
    draw_line(position(), a - position(), 0xff0000);
    draw_diamond(a, 10.0, 0xff0000);
    aim_at_vec(a);
    return Some(angle_diff((a - position()).angle(), heading()).abs());
}

// Instructs the ship to turn toward target. Returns the distance between the
// target and the line projected from the current heading, in meters.
pub fn aim_at_vec(target: Vec2) -> bool {
    aim_at_heading((target - position()).angle())
}

pub fn aim_at_heading(target_heading: f64) -> bool {
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
