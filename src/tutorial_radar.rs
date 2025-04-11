use crate::{
    contacts::{Contact, Contacts},
    control::*,
};
use maths_rs::*;
use oort_api::prelude::*;

pub struct Ship<'a> {
    contacts: Contacts,
    update_contact: Option<&'a mut Contact>,
    scan_heading: f64,
}

const BULLET_SPEED: f64 = 1000.0; // m/s

impl<'a> Ship<'a> {
    pub fn new() -> Ship<'a> {
        Ship {
            contacts: Contacts::new(),
            update_contact: None,
            scan_heading: 0.,
        }
    }

    pub fn tick(&'a mut self) {
        let beam_width = 2. * PI / 16.;
        set_radar_width(beam_width);
        set_radar_max_distance(5000.);

        self.contacts.tick();
        match (&mut self.update_contact, scan()) {
            (Some(update_contact), Some(scan_result))
                if update_contact.class() == scan_result.class =>
            {
                update_contact.update(scan_result);
            }
            (Some(update_contact), _) => {
                debug!("missed updating contact: {:?}", update_contact);
                update_contact.add_miss();
            }
            (None, Some(scan_result)) => {
                self.contacts.recv_contact(scan_result);
            }
            (None, None) => {}
        }
        self.update_contact = None;
        self.contacts.draw();
        self.contacts.cleanup();

        if let Some(contact) = self.contacts.contacts().iter().next() {
            if let Some(angle) =
                aim_at_entity(contact.pos(), contact.vel(), contact.acc(), BULLET_SPEED)
            {
                draw_line(position(), position() + vec2(cos(heading()), sin(heading())) * 1000., 0x00ffff);
                if angle < 2. / 360. * 2. * PI {
                    fire(0);
                }
            }
        }

        if let Some(update_contact) = self.contacts.contact_to_update() {
            // Set the radar so it just scans where the contact will be this
            // turn. Note that we need to point the radar where the contact
            // will be *next* tick.
            debug!("update contact: {:?}", update_contact);
            let pos = update_contact.pos()
                + TICK_LENGTH * (update_contact.vel() + TICK_LENGTH * update_contact.acc());
            let rel = pos - position();
            let dist = rel.length();
            let heading = rel.angle();
            let width = max(10., update_contact.pos_stddev() * 4.);
            debug!("update beam width: {:.2}", width);

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
            self.update_contact = Some(update_contact);
        } else {
            set_radar_heading(self.scan_heading + beam_width);
            set_radar_width(beam_width);
            set_radar_min_distance(0.);
            set_radar_max_distance(radar_max_distance());
            self.scan_heading += beam_width;
        }
    }
}
