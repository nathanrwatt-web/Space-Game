use bevy::prelude::*;
use bevy::math::DVec3;
use std::collections::{HashMap, HashSet};

use crate::sim::clock::SimClock;
use crate::sim::entity::SimEntity;
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
        if r <= 0.0 {
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

#[derive(Resource)]
pub struct OrbitPropagationCache {
    pub dirty: bool,
    order: Vec<Entity>,
    states: HashMap<Entity, (DVec3, DVec3)>,
}

impl Default for OrbitPropagationCache {
    fn default() -> Self {
        Self {
            dirty: true,
            order: Vec::new(),
            states: HashMap::new(),
        }
    }
}

// Operational shell radius for a body, the sphere ships move on
pub const SHELL_FACTOR: f64 = 4.0;

pub fn shell_radius(body: &Body) -> f64 {
    body.radius * SHELL_FACTOR
}

#[allow(clippy::type_complexity)]
pub fn propagate_orbits(
    clock: Res<SimClock>,
    mut cache: ResMut<OrbitPropagationCache>,
    orbiters: Query<(Entity, &Orbit), With<SimEntity>>,
    powered: Query<(Entity, &StateVec), With<SimEntity>>,
    roots: Query<(Entity, &WorldPos), (With<SimEntity>, Without<Orbit>, Without<StateVec>)>,
    mut writeback: Query<&mut WorldPos, (With<SimEntity>, Or<(With<Orbit>, With<StateVec>)>)>,
) {
    let t = clock.t;

    if cache.dirty || cache.order.is_empty() {
        rebuild_order(&mut cache, &orbiters, &roots);
    }

    cache.states.clear();
    for (entity, wp) in &roots {
        cache.states.insert(entity, (wp.0, DVec3::ZERO));
    }

    for idx in 0..cache.order.len() {
        let entity = cache.order[idx];
        let Ok((_, orbit)) = orbiters.get(entity) else { continue };
        let (local_pos, local_vel) = orbit.elements.state_vectors_at(t);
        let (parent_pos, parent_vel) = cache
            .states
            .get(&orbit.parent)
            .copied()
            .unwrap_or((DVec3::ZERO, DVec3::ZERO));
        let abs_pos = parent_pos + local_pos;
        let abs_vel = parent_vel + local_vel;
        cache.states.insert(entity, (abs_pos, abs_vel));
        if let Ok(mut wp) = writeback.get_mut(entity) {
            wp.0 = abs_pos;
        }
    }

    for (entity, sv) in &powered {
        let (parent_pos, parent_vel) = cache
            .states
            .get(&sv.frame)
            .copied()
            .unwrap_or((DVec3::ZERO, DVec3::ZERO));
        let abs_pos = parent_pos + sv.pos;
        let abs_vel = parent_vel + sv.vel;
        cache.states.insert(entity, (abs_pos, abs_vel));
        if let Ok(mut wp) = writeback.get_mut(entity) {
            wp.0 = abs_pos;
        }
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

#[allow(clippy::type_complexity)]
fn rebuild_order(
    cache: &mut OrbitPropagationCache,
    orbiters: &Query<(Entity, &Orbit), With<SimEntity>>,
    roots: &Query<(Entity, &WorldPos), (With<SimEntity>, Without<Orbit>, Without<StateVec>)>,
) {
    let mut children: HashMap<Entity, Vec<Entity>> = HashMap::new();
    for (entity, orbit) in orbiters.iter() {
        children.entry(orbit.parent).or_default().push(entity);
    }

    cache.order.clear();
    let mut seen = HashSet::new();
    for (root, _) in roots.iter() {
        push_children(root, &children, &mut cache.order, &mut seen);
    }
    for (entity, _) in orbiters.iter() {
        if seen.insert(entity) {
            cache.order.push(entity);
            push_children(entity, &children, &mut cache.order, &mut seen);
        }
    }
    cache.dirty = false;
}

fn push_children(
    parent: Entity,
    children: &HashMap<Entity, Vec<Entity>>,
    order: &mut Vec<Entity>,
    seen: &mut HashSet<Entity>,
) {
    let Some(entries) = children.get(&parent) else { return };
    for &child in entries {
        if seen.insert(child) {
            order.push(child);
            push_children(child, children, order, seen);
        }
    }
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
