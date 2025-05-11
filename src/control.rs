use maths_rs::*;
use oort_api::prelude::*;

pub struct Helm {
    // The last position we were instructed to turn to. When setting the helm,
    // if there is a difference between the last position we were instructed to
    // turn to and the current one, that the position will continue changing
    // at the same rate during the time we are turning. As such, if it will take
    // us time to turn to the target, we will adjust the effective desired
    // final heading by the rate of change in the target heading.
    last_heading: f64,
    last_heading_tick: u32,
}

fn to_unit(h: f64) -> Vec2 {
    Vec2::new(h.cos(), h.sin())
}

impl Helm {
    pub fn new() -> Helm {
        Helm {
            last_heading: 0.,
            last_heading_tick: 0,
        }
    }

    // Attempts to point the ship such that a bullet fired at bspd will 
    // pass through the point pos.
    pub fn aim(&mut self, pos: Vec2, bspd: f64) -> f64 {
        let bullet_vec = (pos - position_next()).normalize() * bspd;
        let shoot_vec = bullet_vec - velocity();
        draw_line(position(), shoot_vec, 0xFF00FF);
        self.turn(shoot_vec.angle())
    }

    // Turns as quickly as possible toward the given heading, adjusting for
    // the expected change in heading during the time it takes us to actually
    // perform the turn.
    pub fn turn(&mut self, ht: f64) -> f64 {
        {
            // Try to make it so that next turn we are facing exactly at the
            // desired heading.
            let hn = heading() + TICK_LENGTH * angular_velocity();
            let diff = ht - hn;

            if abs(diff) < TICK_LENGTH.powi(2) * max_angular_acceleration() {
                // The angular acceleration should be exactly the same as the
                // difference between the desired heading and the actual.
                // Note that this is the angular acceleration we want *to occur*.
                // Therefore, the angular acceleration to apply needs to be
                // divided by the tick length.
                let vel_chg = diff / TICK_LENGTH;
                let acc = vel_chg / TICK_LENGTH;
                let ph = hn + vel_chg * TICK_LENGTH;
                debug!("maa={:.4} v={:.4} vn={:.4}", max_angular_acceleration(), angular_velocity(), angular_velocity() + acc * TICK_LENGTH);
                debug!("Turn h={:.4} ht={:.4} hn={:.4} diff={:.4} acc={:.4} ph={:.4}", heading(), ht, hn, diff, acc, ph);
                torque(acc);
                return 0.0;
            } else {
                debug!("diff ({:.4}) > max_acc {:.4}", diff, TICK_LENGTH.powi(2) * max_angular_acceleration());
            }
        }

        // The other algorithm uses normal physics. See the calcs.ipynb file
        // for details. The possible solutions for the time to stop accelerating
        // are
        // (-v - sqrt(-4*a*s + 2*v**2)/2)/a, 
        // (-v + sqrt(-4*a*s + 2*v**2)/2)/a
        // where:
        // * v is initial velocity.
        // * h is target heading
        // * s is the heading error.
        // * a is the angular acceleration.
        //
        // Note that we will assume that the acceleration always opposes the 
        // heading error. This means a * s will always be negative, and
        // therefore the expression inside sqrt will always be positive.
        // Nevertheless it is possible for both solutions to be negative if 
        // the starting velocity is high, acceleration and error are both low.
        // In this case, we just decelerate.

        let s = angle_diff(ht, heading());
        let a = if s < 0. {
            max_angular_acceleration()
        } else {
            -max_angular_acceleration()
        };
        let v = angular_velocity();
        
        let sqrt_term = sqrt(-4. * a * s + 2. * v * v) / 2.;
        let t1 = (-v - sqrt_term) / a;
        let t2 = (-v + sqrt_term) / a;
        let t = if t1 < 0. {
            t2
        } else if t2 < 0. {
            t1
        } else if abs(t1) < abs(t2) {
            t1
        } else {
            t2
        };

        

        let rtorque = if t < 0. {
            -a
        } else if t > TICK_LENGTH {
            a
        } else {
            // If we can accelerate for some fraction of a tick, we need
            // to compute the effective velocity over the course of that tick.
            // The average velocity will be half of the portion of the tick
            // that we accelerate. Note that this is a nod to the discrete
            // nature of the simulation. If we were in a continuous universe
            // we'd just keep accelerating until the exact moment that
            // we have to stop.
            a * (t / TICK_LENGTH) / 2.
        };
        debug!("err={:.4} v={:.4} at={:.4}, rtorque={:.4} nv:{:.4}", s, v, t, rtorque, v + rtorque * TICK_LENGTH);
        torque(rtorque);

        // Compute where we'll be facing next turn compared to where we want
        // to be facing.
        let hnext = heading() + TICK_LENGTH * (angular_velocity() + rtorque * TICK_LENGTH);
        let next_err =  angle_diff(hnext, ht);

        draw_line(position(), position() + to_unit(hnext) * 1000., 0xFF00FF);
        draw_line(position(), position() + to_unit(ht) * 1000., 0xCC00CC);
        debug!("next_err={:.4}", next_err);
        next_err
    }
}

// Returns the position of an entity with the given position, velocity,
// acceleration and acceleration after t seconds have elapsed.
pub fn pos_after(pos: Vec2, vel: Vec2, acc: Vec2, t: f64) -> Vec2 {
    pos + t * (vel + 0.5 * acc * t)
}

// Returns the position of our ship next turn. This isn't exact because it does
// not account for acceleration.
pub fn position_next() -> Vec2 {
    velocity() * TICK_LENGTH + position()
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

// Given a bullet's speed and the current turn position, velocity, and
// acceleration of an entity, returns the earliest position where a bullet can 
// intersect with the trajectory of that entity if fired *next turn*. This 
// function takes into account our current position and velocity but does not
// take into account our acceleration.
pub fn lead(e_pos: Vec2, e_vel: Vec2, e_acc: Vec2, b_spd: f64) -> Option<Vec2> {
    let pos_next = position() + velocity() * TICK_LENGTH;

    let es = e_pos - pos_next;
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

pub fn turn_and_shoot_at(helm: &mut Helm, pos: Vec2, vel: Vec2, acc: Vec2, bspd: f64) {
    if let Some(aimpoint) = lead(pos, vel, acc, bspd) {
        if abs(helm.aim(aimpoint, bspd)) * 360. / (2. * PI) < 1.  {
            fire(0);
        }
    }
}