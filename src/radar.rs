use std::{ops::Mul, rc::Rc};

use crate::{contacts::Contacts, control::*};
use oort_api::{
    ClassStats,
    prelude::{maths_rs::*, *},
};

pub struct Radar {
    update_contact_id: Option<u32>,
    scan_heading: f64,
    pub scan_beam_width: f64,
}

fn ship_dim(c: Class) -> f64 {
    match c {
        Class::Fighter => 20.,
        Class::Frigate => 240.,
        Class::Cruiser => 480.,
        Class::Asteroid => 100.,
        Class::Torpedo => 16.,
        Class::Target => 40.,
        Class::Missile => 3.,
        Class::Unknown => 50.,
    }
}

impl Radar {
    pub fn new() -> Radar {
        Radar {
            update_contact_id: None,
            scan_heading: 0.,
            scan_beam_width: 2. * PI / 16.,
        }
    }

    pub fn set_scan_beam_width(&mut self, scan_beam_width: f64) {
        let min_width = match class() {
            Class::Cruiser | Class::Frigate => 1. / 3600. * 2. * PI,
            _ => 1. / 720. * 2. * PI,
        };
        self.scan_beam_width = clamp(scan_beam_width, min_width, PI / 2.);
    }

    pub fn tick(&mut self, contacts: &mut Contacts) {
        {
            match (self.update_contact_id, scan()) {
                (Some(id), scan_result) => {
                    contacts.update(id, scan_result);
                }
                (None, Some(scan_result)) => {
                    contacts.recv_contact(scan_result);
                }
                (None, None) => {}
            }
            self.update_contact_id = None;
        }
        if let Some(update_contact) = contacts.contact_to_update() {
            // Figure out where we and the enemy will be next turn.
            let pos_e = pos_after(
                update_contact.pos(),
                update_contact.vel(),
                update_contact.acc(),
                TICK_LENGTH,
            );
            let pos_me = position_next();


            let rel = pos_e - pos_me;
            let dist = rel.length();
            let heading = rel.angle();
            let width = clamp(
                update_contact.pos_stddev() * 4.,
                2. * ship_dim(update_contact.class()),
                1000.,
            );

            // The beam forms a triangle. The adjacent edge is the distance
            // to the contact position. The opposite edge is half the width.
            // opposite/adjacent = tan(angle) / 2
            // (width/2) / distance = tan(angle) / 2
            // width/distance = tan(angle)
            // angle = atan(width/distance)
            let beam_width = 2. * atan(width / dist);
            set_radar_heading(heading);
            set_radar_width(beam_width);
            set_radar_max_distance(dist + width / 2.);
            set_radar_min_distance(dist - width / 2.);
            self.update_contact_id = Some(update_contact.id);
        } else {
            set_radar_heading(self.scan_heading + self.scan_beam_width);
            set_radar_width(self.scan_beam_width);
            set_radar_min_distance(0.);
            set_radar_max_distance(100000.);
            self.scan_heading += self.scan_beam_width;
        }
    }
}
