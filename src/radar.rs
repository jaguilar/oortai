/*

use crate::control::*;
use oort_api::prelude::{maths_rs::*, *};

// Contains information about a contact, including when it was last seen,
// and what it's position and velocity were at that time.
struct Contact {
    last_seen_tick: u32,
    scan_result: ScanResult,
    acceleration: Vec2,
}

impl Contact {
    // Returns the expected current position of the contact assuming it
    // continued on its prior course (inclusive of its measured acceleration).
    fn pos_now(&self) -> Vec2 {
        pos_after(
            self.scan_result.position,
            self.scan_result.velocity,
            self.acceleration,
            ((current_tick() - self.last_seen_tick) as f64) * TICK_LENGTH,
        )
    }
}

// The state of
enum Mode {
    Scan,
    Track,
}

// The state of the radar. The radar runs in two modes: scan and track.
// In scan mode, we circle the radar around looking for new contacts.
// In trakc mode, we point the radar where we expect a contact to be and update
// its data. Each contact update takes only one tick, so we can lock up a decent
// handful of contacts and still have time to scan for new ones.
pub struct Radar {
    contacts: Vec<Contact>,
    mode: Mode,
    start_tick: u32,
    scan_idx: usize,
}

impl Radar {
    pub fn new() -> Radar {
        Radar {
            contacts: Vec::new(),
            mode: Mode::Scan,
            start_tick: current_tick(),
            track_idx: 0,
        }
    }

    pub fn tick(&mut self) {
        // Start by scanning whatever the radar is pointing at.
        if let Some(nc) = scan() {
            // In tracking mode, this had better be the contact we started with.
            // In scanning mode, we're happy to see anything.

            // See if any of our contacts should be where we hit on the scan.
            if let Some(oc) = &mut self
                .contacts
                .iter_mut()
                .filter(|c| {
                    let dist = (c.pos_now() - nc.position).length();
                    nc.class == c.scan_result.class && dist < 250.
                })
                .next()
            {
                // We will only update if the snr is within ten decibels of
                // the old snr. This is just a placeholder value.
                if oc.scan_result.snr < nc.snr + 10. {
                    oc.last_seen_tick = current_tick();
                    oc.acceleration = (nc.velocity - oc.scan_result.velocity) / TICK_LENGTH;
                    oc.scan_result = nc;
                } else {
                    debug!("Did not update contact because SNR was too low.");
                }
            } else {
                // There is no old version of this contact. Create a new one.
                self.contacts.push(Contact {
                    last_seen_tick: current_tick(),
                    scan_result: nc,
                    acceleration: vec2(0., 0.),
                });
            }
            if self.mode == Mode::Track {
                ++self.track_idx;
            }
        } else {
            // No contact was found in this scan. If we were scanning for a
            // particular contact, prune it from the database.
            self.contacts.swap_remove(self.track_idx);
        }

        // The mode only controls how we'll point the radar for the next tick.
        // In scan mode, we sweep the radar all around us, intentionally trying
        // to avoid known contacts.
        // In track mode, we point the radar where we expect each contact to be,
        // in sequence, with filter bands set such that we're trying *not* to
        // catch any other contacts in the scan.
        // If the enemy contact is moving very quickly, we may lose it at this
        // juncture, but only if it deviates significantly

        match self.mode {
            Mode::Scan => {}
            Mode::Track => {}
        }
    }
}
*/
