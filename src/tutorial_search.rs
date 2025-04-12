use crate::{contacts::*, control::*, radar::*};
use maths_rs::*;
use oort_api::prelude::*;

pub struct Ship {
    contacts: Contacts,
    radar: Radar,
}

const BULLET_SPEED: f64 = 1000.0; // m/s

impl Ship {
    pub fn new() -> Ship {
        Ship {
            contacts: Contacts::new(),
            radar: Radar::new(),
        }
    }

    pub fn tick(&mut self) {
        self.radar.scan_beam_width = 2. * PI / 128.;

        self.contacts.tick();
        self.radar.tick(&mut self.contacts);
        self.contacts.draw();

        if let Some(contact) = self.contacts.iter_mut().min_by(|a, b| a.id.cmp(&b.id)) {
            // Accelerate toward the contact.
            let vec_to = contact.pos() - position();
            accelerate(vec_to * max_forward_acceleration());

            let bullet_travel = vec_to.length() / BULLET_SPEED;
            if bullet_travel > 2. {
              if abs(angle_diff(heading(), vec_to.angle())) < 1. * 360. / (2. * PI) {
                // If we are facing the contact, boost.
                activate_ability(Ability::Boost);
              }
              activate_ability(Ability::Boost);
              point_at(contact.pos());
            } else if let Some(aim_pos) =
                aim_at_entity(contact.pos(), contact.vel(), contact.acc(), BULLET_SPEED)
            {
                draw_line(
                    position(),
                    position() + vec2(cos(heading()), sin(heading())) * 1000.,
                    0x00ffff,
                );
                if reload_ticks(0) <= 0 {
                    let err = abs(angle_diff((aim_pos - position()).angle(), heading()));
                    if err < 2. / 360. * 2. * PI {
                        let hit_time =
                            current_time() + (aim_pos - position()).length() / BULLET_SPEED;
                        contact.record_prediction(hit_time, aim_pos);
                        fire(0);
                        debug!("firing!");
                    } else {
                        debug!("aim error: tracking {:.2}", err * 360. / (2. * PI));
                    }
                } else {
                    debug!("reload: {}", reload_ticks(0));
                }
            } else {
                debug!("aim error: no firing solution");
            }
        }
    }
}
