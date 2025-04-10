
use crate::control::*;
use oort_api::prelude::{
    maths_rs::{
        mat::*,
        num::{Number, NumberOps, SignedNumber},
        *,
    },
    *,
};

struct KalmanFilter {
    // The state of the enemy contact.
    // pos_x, pos_y, vel_x, vel_y
    state: Vec4f,

    // The state covariance matrix: represents the uncertainty in the state.
    // Also called P in the literature.
    state_covariance: Mat4f,

    // The process covariance matrix is fixed per timestep for a given ship
    // class. Also called Q in the literature.
    process_covariance: Mat4f,
}

fn add<T: Number>(mut a: Mat4<T>, b: &Mat4<T>) -> Mat4<T> {
    for i in 0..16 {
        a[i] += b[i];
    }
    a
}

fn neg<T: SignedNumber>(mut a: Mat4<T>) -> Mat4<T> {
    for i in 0..16 {
        a[i] = -a[i];
    }
    a
}

// Returns measurement noise covariance matrix.
fn calculate_measurement_covariance(snr: f64) -> Mat4f {
    let error_factor = 10.0f32.powf(-snr as f32 / 10.0);
    const DISTANCE_NOISE_FACTOR: f32 = 1e4;
    let pos_var = (DISTANCE_NOISE_FACTOR * error_factor).powi(2);
    const VELOCITY_NOISE_FACTOR: f32 = 1e2;
    let vel_var = (VELOCITY_NOISE_FACTOR * error_factor).powi(2);
    debug!("snr: {:2}, err fact: {:2} pos_var = {:.2}, vel_var = {:.2}", snr, error_factor, pos_var, vel_var);
    #[rustfmt::skip]
    return Mat4f::new(
        pos_var, 0., 0., 0.,
        0., vel_var, 0., 0.,
        0., 0., pos_var, 0.,
        0., 0., 0., vel_var,
    );
}

fn transition_matrix() -> Mat4f {
    #[rustfmt::skip]
    let f = Mat4f::new(
        1.0, TICK_LENGTH as f32, 0., 0.,
        0., 1., 0., 0.,
        0., 0., 1., TICK_LENGTH as f32,
        0., 0., 0., 1.,
    );
    f
}

impl KalmanFilter {
    pub fn new(class: Class, pos: Vec2, vel: Vec2, snr: f64) -> KalmanFilter {
        let max_accel = class.default_stats().max_forward_acceleration;
        let accel_var = (max_accel * 2. / 12.).powi(2) as f32;
        let tick2 = TICK_LENGTH.powi(2) as f32;
        let tick3 = TICK_LENGTH.powi(3) as f32;
        let tick4 = TICK_LENGTH.powi(4) as f32;

        #[rustfmt::skip]
        let process_covariance = Mat4f::new(
            0.25f32 * tick4 * accel_var, 0.5 * tick3 * accel_var, 0., 0.,
            0.5f32 * tick3 * accel_var, tick2 * accel_var, 0., 0.,
            0., 0., 0.25f32 * tick4 * accel_var, 0.5 * tick3 * accel_var,
            0., 0., 0.5f32 * tick3 * accel_var, tick2 * accel_var,
        );

        // Use the RSSI to calculate the initial noise on the position and
        // velocity.
        KalmanFilter {
            state: Vec4f::new(pos.x as f32, vel.x as f32, pos.y as f32, vel.y as f32),
            state_covariance: add(process_covariance, &calculate_measurement_covariance(snr)),
            process_covariance,
        }
    }

    pub fn pos(&self) -> Vec2 {
        Vec2::new(self.state[0] as f64, self.state[2] as f64)
    }

    pub fn vel(&self) -> Vec2 {
        Vec2::new(self.state[1] as f64, self.state[3] as f64)
    }

    pub fn acc(&self) -> Vec2 {
        Vec2::new(
            self.state_covariance[0] as f64,
            self.state_covariance[2] as f64,
        )
    }

    // Updates the predicted contact state and covariance.
    pub fn predict(&mut self) {
        #[rustfmt::skip]
        let f = transition_matrix();
        self.state = f * self.state;
        self.state_covariance = add(
            f * self.state_covariance * f.transpose(),
            &self.process_covariance,
        );
    }

    // Updates the filter with a new observation.
    pub fn update(&mut self, pos: Vec2, vel: Vec2, snr: f64) {
        // Below we will omit the H vector since our measurement model is 1:1
        // with our measurement vector (i.e. it is the identity matrix).
        let measurement_covariance = calculate_measurement_covariance(snr);
        let kalman_gain =
            self.state_covariance * add(self.state_covariance, &measurement_covariance).inverse();

        let measurement = Vec4f::new(pos.x as f32, vel.x as f32, pos.y as f32, vel.y as f32);
        self.state = self.state + kalman_gain * (measurement - self.state);

        self.state_covariance = (add(Mat4f::identity(), &neg(kalman_gain))) * self.state_covariance;
    }
}

// A Contact stores all the information we know about one enemy contact.
// We get information about the contact from the radar and periodically update
// it by scanning where we expect it to be.
pub struct Contact {
    // The ship class of the contact.
    class: Class,

    // We assign a unique id to each contact.
    id: u32,

    // The physics of the contact.
    filter: KalmanFilter,

    // The velocity we saw last time we updated.
    vel_last_update: Vec2,

    // The estimated acceleration over the course of the last update window.
    acc: Vec2,

    // When was the last time we saw this contact on the radar?
    last_seen_tick: u32,

    // How many times have we tried to track this contact and failed to find it
    // where we expected. Resets every time we successfully track. If it climbs
    // too high we'll probably delete it from the contact database.
    tracking_miss_count: u32,
}

impl Contact {
    pub fn new(class: Class, id: u32, pos: Vec2, vel: Vec2, snr: f64) -> Contact {
        Contact {
            class,
            id,
            filter: KalmanFilter::new(class, pos, vel, snr),
            vel_last_update: vel,
            acc: vec2(0., 0.),
            last_seen_tick: current_tick(),
            tracking_miss_count: 0,
        }
    }

    pub fn since_update(&self) -> f64 {
        (current_tick() - self.last_seen_tick) as f64 * TICK_LENGTH
    }

    pub fn pos(&self) -> Vec2 {
        self.filter.pos() + 0.5 * self.acc * self.since_update().powi(2)
    }

    pub fn vel(&self) -> Vec2 {
        self.filter.vel() + self.acc * self.since_update()
    }

    pub fn class(&self) -> Class {
        self.class
    }

    pub fn id(&self) -> u32 {
        self.id
    }

    pub fn acc(&self) -> Vec2 {
        self.acc
    }

    // Reports the furthest away a scan can be and still match this contact.
    // For contacts we've scanned recently with good signal this will be
    // approximately the assumed ship size. As time goes since our last scan
    // the allowed range will increase. 
    pub fn max_distance_for_match(&self) -> f64 {
        let assumed_ship_size = 50.0;
        // Length is sqrt of sides squared, and the variance is already
        // stddev squared.
        let len_sq = self.filter.state_covariance[0] + self.filter.state_covariance[2];
        if len_sq > 0. {
            // We have a non-zero variance. Use it.
            len_sq.sqrt() as f64 * 3. + assumed_ship_size
        } else {
            assumed_ship_size
        }
    }


    pub fn tick(&mut self) {
        // Update the filter with the current position and velocity.
        if self.id == 0 {
            debug!(
                "Contact {}: pos: {:?}\nvel: {:?}\nacc: {:?}",
                self.id,
                self.pos(),
                self.vel(),
                self.acc,
            );
            debug!("Match range before: {:.2}", self.max_distance_for_match());
        }
        self.filter.predict();
        if self.id == 0 {
            debug!("Match range after: {:.2}", self.max_distance_for_match());
        }
    }

    pub fn update(&mut self, pos: Vec2, vel: Vec2, snr: f64) {
        // Update the filter with the new position and velocity.
        self.filter.update(pos, vel, snr);
        self.acc = (self.vel() - self.vel_last_update) / self.since_update();
        self.vel_last_update = self.vel();
        self.last_seen_tick = current_tick();
        self.tracking_miss_count = 0;
    }

    // Marks that we missed tracking for this contact. Returns the number of
    // tracking misses since the last update.
    pub fn add_miss(&mut self) -> u32 {
        self.tracking_miss_count += 1;
        self.tracking_miss_count
    }

    pub fn draw(&self) {
        // Draw the contact.
        let pos = self.pos();
        draw_diamond(pos, self.max_distance_for_match(), 0xff0000);
        draw_line(pos, pos + self.vel(), 0xfff000);
        draw_line(pos, pos + self.acc, 0xffff00);
        draw_text!(pos, 0x00ff00, "ID: {}{:?}", self.id, self.class);
    }
}

pub struct Contacts {
    contacts: Vec<Contact>,
    next_id: u32,
}

impl Contacts {
    pub fn new() -> Contacts {
        Contacts {
            contacts: Vec::new(),
            next_id: 0,
        }
    }

    pub fn contacts(&self) -> &Vec<Contact> {
        &self.contacts
    }

    pub fn tick(&mut self) {
        // Update all the contacts.
        for contact in &mut self.contacts {
            contact.tick();
        }
    }

    pub fn recv_contact(&mut self, contact: &ScanResult) {
        // Find the contact that is most likely to be the same contact as the
        // scan result. It needs to have the same class and be within the
        // volume we consider 99% likely to contain the contact (3 std dev).
        let dist_from_scan = |a: &Contact| (a.pos() - contact.position).length() as f64;
        if let Some(best_contact) = self
            .contacts
            .iter_mut()
            .filter(|c| c.class == contact.class)
            .min_by_key(|a| dist_from_scan(a) as i32)
            .filter(|a| {
                let ok = dist_from_scan(a) < a.max_distance_for_match();
                if !ok {
                    debug!(
                        "Contact {} is too far from scan result: {:.2} meters, max: {:.2}",
                        a.id,
                        dist_from_scan(a),
                        a.max_distance_for_match()
                    );
                }
                ok
            })
            .as_mut()
        {
            // We have a contact that is likely the same as the scan result.
            // Update it.
            debug!(
                "Updating contact {:.2} meters from max {:.2}",
                (best_contact.pos() - contact.position).length(),
                best_contact.max_distance_for_match()
            );
            best_contact.update(contact.position, contact.velocity, contact.snr);
            debug!("Max dist after update: {:.2}", best_contact.max_distance_for_match());
        } else {
            self.contacts.push(Contact::new(
                contact.class,
                self.next_id,
                contact.position,
                contact.velocity,
                contact.snr,
            ));
            self.next_id += 1;
        }
    }

    pub fn cleanup(&mut self) {
        // Remove contacts that we haven't seen in a while.
        self.contacts.retain(|c| c.since_update() < 0.5);
    }

    pub fn draw(&self) {
        for contact in &self.contacts {
            contact.draw();
        }
    }
}
