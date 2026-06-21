// This is the grid so that not every frame - based computation will 
// have to look at every body, only those in its own cell
use std::collections::HashMap;
use bevy::prelude::*;
use bevy::math::{DVec3, IVec3};

use crate::sim::clock::SimClock;
use crate::sim::orbit::{Orbit, Body, Maneuvers};
use crate::sim::integrate::StateVec;
use crate::sim::soi::soi_radius;

// performance knob, change based on size of system / # of entities 
const DEFAULT_CELL: f64 = 256.0;

// A sibling body as an SOI, in its parent frames local coordinates.
#[derive(Clone, Copy)]
pub struct BodyEntry {
    pub entity: Entity,
    pub pos: DVec3,
    pub vel: DVec3,
    pub soi: f64,
    pub mu: f64,
}

#[derive(Resource)]
pub struct Broadphase {
    bodies: HashMap<(Entity, IVec3), Vec<BodyEntry>>,      // SOI-fattened: for reparenting
    ships:  HashMap<(Entity, IVec3), Vec<(Entity, DVec3)>>, // points: for target acquisition
    pub cell: f64,
}

impl Default for Broadphase {
    fn default() -> Self {
        Self { bodies: HashMap::new(), ships: HashMap::new(), cell: DEFAULT_CELL }
    }
}

pub(crate) fn cell_of(p: DVec3, cell: f64) -> IVec3 {
    (p / cell).floor().as_ivec3()
}

impl Broadphase {
    // Sibling bodies whose SOI might contain a ship at local_pos in frame.
    pub fn soi_candidates(&self, frame: Entity, local_pos: DVec3) -> &[BodyEntry] {
        self.bodies
            .get(&(frame, cell_of(local_pos, self.cell)))
            .map_or(&[][..], |v| v.as_slice())
    }

    // Ships within radius of pos in frame
    pub fn ships_near(&self, frame: Entity, pos: DVec3, radius: f64, exclude: Entity) -> Vec<Entity> {
        let low = cell_of(pos - DVec3::splat(radius), self.cell);
        let high = cell_of(pos + DVec3::splat(radius), self.cell);
        let mut out = Vec::new();
        for x in low.x..=high.x {
            for y in low.y..=high.y {
                for z in low.z..=high.z {
                    if let Some(v) = self.ships.get(&(frame, IVec3::new(x, y, z))) {
                        for &(e, p) in v {
                            if e != exclude && (p - pos).length() <= radius {
                                out.push(e);
                            }
                        }
                    }
                }
            }
        }
        out
    }
} 

pub fn build_broadphase(
    clock: Res<SimClock>,
    mut bp: ResMut<Broadphase>,
    bodies: Query<(Entity, &Orbit, &Body), Without<Maneuvers>>,
    rail_ships: Query<(Entity, &Orbit), With<Maneuvers>>,
    powered_ships: Query<(Entity, &StateVec), With<Maneuvers>>,
) {
    let t = clock.t;
    let cell = bp.cell;
    bp.bodies.clear();
    bp.ships.clear();

    // bodies → every cell their SOI sphere overlaps, keyed by their parent frame
    for (e, orbit, body) in &bodies {
        let (pos, vel) = orbit.elements.state_vectors_at(t);
        let soi = soi_radius(orbit.elements.a, body.mu, orbit.elements.mu);
        let entry = BodyEntry { entity: e, pos, vel, soi, mu: body.mu };
        let low = cell_of(pos - DVec3::splat(soi), cell);
        let high = cell_of(pos + DVec3::splat(soi), cell);
        for x in low.x..=high.x {
            for y in low.y..=high.y {
                for z in low.z..=high.z {
                    bp.bodies.entry((orbit.parent, IVec3::new(x, y, z))).or_default().push(entry);
                }
            }
        }
    }

    // ships → single cell, keyed by frame (Orbit.parent or StateVec.frame)
    for (e, orbit) in &rail_ships {
        let pos = orbit.elements.state_vectors_at(t).0;
        bp.ships.entry((orbit.parent, cell_of(pos, cell))).or_default().push((e, pos));
    }
    for (e, sv) in &powered_ships {
        bp.ships.entry((sv.frame, cell_of(sv.pos, cell))).or_default().push((e, sv.pos));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn point_in_soi_lands_in_an_inserted_cell() {
        let cell = 256.0;
        let pos = DVec3::new(123.0, -50.0, 900.0);
        let soi = 400.0;
        let lo = cell_of(pos - DVec3::splat(soi), cell);
        let hi = cell_of(pos + DVec3::splat(soi), cell);
        for k in 0..2000 {
            let a = k as f64 * 0.137;
            let dir = DVec3::new(a.sin(), (a * 1.7).cos(), (a * 0.3).sin()).normalize_or_zero();
            let q = pos + dir * (soi * 0.999); // just inside the SOI sphere
            let c = cell_of(q, cell);
            assert!(
                (lo.x..=hi.x).contains(&c.x)
                    && (lo.y..=hi.y).contains(&c.y)
                    && (lo.z..=hi.z).contains(&c.z),
                "q cell {c:?} fell outside inserted range [{lo:?},{hi:?}]"
            );
        }
    }
}
