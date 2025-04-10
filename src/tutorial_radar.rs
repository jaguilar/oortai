use crate::{contacts::Contacts, control::*};
use maths_rs::*;
use oort_api::prelude::*;

pub struct Ship {
  contacts: Contacts
}

const BULLET_SPEED: f64 = 1000.0; // m/s

impl Ship {
    pub fn new() -> Ship {
        Ship {
            contacts: Contacts::new(),
        }
    }

    pub fn tick(&mut self) {
        let beam_width = 2. * PI / 16.; 
        set_radar_width(beam_width);
        set_radar_max_distance(5000.);

        self.contacts.tick();
        if let Some(contact) = scan() {
            self.contacts.recv_contact(&contact);
        }
        self.contacts.draw();
        self.contacts.cleanup();

        set_radar_heading(radar_heading() + beam_width);

        if let Some(contact) = self.contacts.contacts().iter().by_ref().next() {
            if let Some(angle) = aim_at_entity(contact.pos(), contact.vel(), contact.acc(), BULLET_SPEED) {
                if angle < 2. / 360. * 2.* PI {
                    fire(0);
                }
            }
        }
    }
}
