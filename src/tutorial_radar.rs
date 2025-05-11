use crate::{
    contacts::*,
    control::*,
    radar::*,
};
use maths_rs::*;
use oort_api::prelude::*;

pub struct Ship {
    contacts: Contacts,
    radar: Radar,
    helm: Helm,
}

const BULLET_SPEED: f64 = 1000.0; // m/s

impl Ship {
    pub fn new() -> Ship {
        Ship {
            contacts: Contacts::new(),
            radar: Radar::new(),
            helm: Helm::new(),
        }
    }

    pub fn tick(&mut self) {
        let beam_width = 2. * PI / 16.;
        set_radar_width(beam_width);
        set_radar_max_distance(5000.);

        self.contacts.tick();
        self.radar.tick(&mut self.contacts);
        self.contacts.draw();

        let dist = |a: &Contact| (position() - a.pos()).length();
        if let Some(contact) = self.contacts.iter_mut().min_by(|a, b| a.id.cmp(&b.id)) {
            turn_and_shoot_at(
                &mut self.helm,
                contact.pos(),
                contact.vel(),
                contact.acc(),
                BULLET_SPEED,
            );
        }
    }
}
