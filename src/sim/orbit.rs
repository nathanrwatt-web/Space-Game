use bevy::prelude::*;
use bevy::math::DVec3;
use std::collections::HashMap;

use crate::sim::clock::SimClock;
use crate::world_pos::WorldPos;
use crate::math::orbital_elements::OrbitalElements;
use crate::sim::integrate::StateVec;
use std::collections::VecDeque;
use serde::{Serialize, Deserialize};

#[derive(Component)] 
pub struct Orbit {
    pub(crate) elements: OrbitalElements, // math of the specific orbit 
    pub(crate) parent: Entity, // id of what it orbits 
}

impl Orbit {
    pub fn from_statevec(sv: &StateVec, mu: f64, t: f64) -> Self {
        Self {
            elements: OrbitalElements::from_state(sv.pos, sv.vel, mu, t),
            parent: sv.frame,
        }
    }

    // builds a stable coast orbit from a statevec (pos, vel)
    pub fn park_from_statevec(sv: &StateVec, mu: f64, body_radius: f64, t: f64) -> Self {
        let natural = Self::from_statevec(sv, mu, t);
        let el = natural.elements;
        let periapsis = el.a * (1.0 - el.e);
        let stable = el.a.is_finite() && el.e.is_finite() && el.e < 1.0 && periapsis > body_radius;
        if stable {
            return natural;
        }

        // circularize at the current radius, keeping the ship's heading if it has one
        let r = sv.pos.length();
        if !(r > 0.0) {
            return natural; // at the body centre; nothing sensible to do
        }
        let r_hat = sv.pos / r;
        let v_circ = (mu / r).sqrt();
        let tang = sv.vel - sv.vel.dot(r_hat) * r_hat; // tangential part of current velocity
        let dir = if tang.length() > 1e-6 {
            tang.normalize()
        } else {
            // no tangential motion: pick an arbitrary perpendicular for a sane plane
            let axis = if r_hat.z.abs() < 0.9 { DVec3::Z } else { DVec3::X };
            r_hat.cross(axis).normalize()
        };
        Self {
            elements: OrbitalElements::from_state(sv.pos, dir * v_circ, mu, t),
            parent: sv.frame,
        }
    }
}

#[derive(Component, Default, Clone, Serialize, Deserialize)]
pub struct Burn {
    pub execute_at: f64, // time to fire
    pub dv: DVec3, // velocity change
}

#[derive(Component, Default, Clone, Serialize, Deserialize)]
pub struct Maneuvers {
    pub queue: VecDeque<Burn>,
}

#[derive(Component)]
pub struct Body {
    pub mu: f64,
    pub radius: f64,
}

// Operational shell radius for a body, the sphere ships move on
pub const SHELL_FACTOR: f64 = 4.0;

pub fn shell_radius(body: &Body) -> f64 {
    body.radius * SHELL_FACTOR
}

#[allow(clippy::complexity)]
pub fn propagate_orbits(
    clock: Res<SimClock>,
    orbiters: Query<(Entity, &Orbit)>, // every orbiting object
    powered: Query<(Entity, &StateVec)>,
    roots: Query<(Entity, &WorldPos), (Without<Orbit>, Without<StateVec>)>,
    mut writeback: Query<(Entity, &mut WorldPos), Or<(With<Orbit>, With<StateVec>)>>,
) {
    let t = clock.t;
    
    // offsets of each item to be used later 
    let mut locals: HashMap<Entity, (DVec3, DVec3, Entity)> = orbiters
        .iter()
        // map each element to new (position, velocity, parent)
        .map(|(e, o)| {
            let (pos, vel) = o.elements.state_vectors_at(t);
            (e, (pos, vel , o.parent))
        })
        .collect();

    // calculate orbits for powered ship 
    for (e, sv) in &powered {
        locals.insert(e, (sv.pos, sv.vel, sv.frame));
    }

    // position of each entity wihthout orbit
    let root_pos: HashMap<Entity, DVec3> = roots
        .iter()
        // w.0 = DVec3, position 
        .map(|(e,w)| (e, w.0))
        .collect();
    
    // checking for previous computation
    let mut cache: HashMap<Entity, (DVec3, DVec3)> = HashMap::new();

    // for each entity and its world position update its world position 
    for (e, mut wp) in &mut writeback {
        wp.0 = absolute_state(e, &locals, &root_pos, &mut cache).0;
    }
}

pub fn draw_orbits(
    mut gizmos: Gizmos,
    camera: Single<&WorldPos, With<Camera>>,
    orbits: Query<&Orbit>,
    bodies: Query<&WorldPos, Without<Camera>>,
) {
    const SEGMENTS: usize = 128;
    let cam = **camera; // worldpos of camera 

    for orbit in &orbits {
        let Ok(parent_wp) = bodies.get(orbit.parent) else { continue; };
        let parent_pos = parent_wp.0;
        let elements = &orbit.elements;
        let color = Color::srgb(0.35, 0.35, 0.4);
    
        // if circle or elipse 
        if elements.e < 1.0 {
            let period = elements.period();
            let points = (0..SEGMENTS).map(|i| {
                let t = period * i as f64 / SEGMENTS as f64;
                let world = parent_pos + elements.offset_at(t);
                WorldPos(world).to_render_space(cam)
            });
            gizmos.lineloop(points, color);
        } else { 
            // open hyperbola, go between asymptotes 
            let nu_max = (-1.0 / elements.e).acos() * 0.98;
            let points = (0..=SEGMENTS).map(|i| {
                let nu = -nu_max + 2.0 * nu_max * i as f64 / SEGMENTS as f64;
                WorldPos(parent_pos + elements.point_at_true_anomaly(nu)).to_render_space(cam)
            });
            gizmos.linestrip(points, color);
        }
    }
}

// for the current time, while there is still a scheduled burn, do it
pub fn execute_maneuvers(
    clock: Res<SimClock>,
    mut ships: Query<(&mut Orbit, &mut Maneuvers)>,
) {
    for (mut orbit, mut maneuvers) in &mut ships {
        while maneuvers.queue.front().is_some_and(|b| b.execute_at <= clock.t) {
            let burn = maneuvers.queue.pop_front().unwrap();
            orbit.elements = orbit.elements.with_burn(burn.execute_at, burn.dv);
        }
    }
}

// absoulte (pos, vel) by summing local vectors up the parent chain
pub(crate) fn absolute_state(
    e: Entity, 
    locals: &HashMap<Entity, (DVec3, DVec3, Entity)>, // pos, vel, parent 
    roots: &HashMap<Entity, DVec3>, 
    cache: &mut HashMap<Entity, (DVec3, DVec3)>,
) -> (DVec3, DVec3) {

    // check to see if the entity has been computed
    if let Some(&s) = cache.get(&e) { return s; }

    let state = match locals.get(&e) {
        Some(&(local_position, local_velocity, parent)) => {
            let (parent_position, parent_velocity) = absolute_state(parent, locals, roots, cache);
            (parent_position + local_position, parent_velocity + local_velocity)
        },
        None => { 
            (roots.get(&e).copied().unwrap_or(DVec3::ZERO), DVec3::ZERO)
        },
    };

    cache.insert(e, state);
    state
}

#[cfg(test)]
mod tests {
    use super::*;

    const MU: f64 = 1.267e17;

    #[test]
    fn park_from_rest_gives_stable_circular_orbit() {
        // after Hold the velocity is ~0 (no real orbit); park must circularize so the
        // ship exits to a clean, flyable orbit instead of a degenerate plunge.
        let (body_radius, r) = (7.0e6, 1.0e7);
        let sv = StateVec { pos: DVec3::new(r, 0.0, 0.0), vel: DVec3::ZERO, frame: Entity::PLACEHOLDER };
        let el = Orbit::park_from_statevec(&sv, MU, body_radius, 0.0).elements;

        assert!(el.e < 1e-3, "should be ~circular, e = {}", el.e);
        assert!(el.a * (1.0 - el.e) > body_radius, "periapsis below the body");
        for k in 0..8 {
            let p = el.offset_at(el.period() * k as f64 / 8.0);
            assert!(p.is_finite() && (p.length() - r).abs() / r < 1e-3, "bad point {p:?}");
        }
    }

    #[test]
    fn park_keeps_a_good_orbit() {
        // a healthy bound orbit must be preserved (velocity not snapped to circular)
        let r = 1.0e7;
        let vc = (MU / r).sqrt();
        let sv = StateVec {
            pos: DVec3::new(r, 0.0, 0.0),
            vel: DVec3::new(0.0, vc * 0.95, 0.0),
            frame: Entity::PLACEHOLDER,
        };
        let el = Orbit::park_from_statevec(&sv, MU, 1.0e6, 0.0).elements;
        let (_, v) = el.state_vectors_at(0.0);
        assert!((v.length() - vc * 0.95).abs() / (vc * 0.95) < 1e-6, "good orbit was altered");
    }
}
