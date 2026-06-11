use bevy::prelude::*;
use bevy::math::{Isometry3d, DVec3};
use crate::math::orbital_elements::OrbitalElements;
use crate::sim::orbit::{Orbit, Body, Maneuvers, absolute_state};
use crate::world_pos::WorldPos;
use crate::sim::clock::SimClock;
use std::collections::HashMap;

// Laplace sphere of influence 

pub fn soi_radius(a: f64, mu_body: f64, mu_parent: f64) -> f64 {
    a * (mu_body / mu_parent).powf(0.4)
}

// update the sphere of influence for ships 
pub fn update_soi(
    clock: Res<SimClock>,
    mut ships: Query<(Entity, &mut Orbit), With<Maneuvers>>,
    bodies: Query<(Entity, &Orbit, &Body), Without<Maneuvers>>,
    roots: Query<(Entity, &WorldPos), Without<Orbit>>,
) {
    let t = clock.t;

    // current state
    let mut locals: HashMap<Entity, (DVec3, DVec3, Entity)> = HashMap::new();
    for (entity, orbit, _) in &bodies {
        let (pos, vel) = orbit.elements.state_vectors_at(t);
        locals.insert(entity, (pos, vel, orbit.parent));
    }

    for (entity, orbit) in ships.iter() {
        let (pos, vel) = orbit.elements.state_vectors_at(t);
        locals.insert(entity, (pos, vel, orbit.parent));
    }

    let root_pos: HashMap<Entity, DVec3> = roots
        .iter()
        .map(|(e, wp)| (e, wp.0))
        .collect();

    let mut cache = HashMap::new();
    for (ship_e, mut orbit) in ships.iter_mut() {
        let (r_ship, v_ship) = absolute_state(ship_e, &locals, &root_pos, &mut cache);
        let parent = orbit.parent;

        // ASCEND: outside current parent's SOI?
        if let Ok((_, p_orbit, p_body)) = bodies.get(parent) {
            let r_soi_p = soi_radius(p_orbit.elements.a, p_body.mu, p_orbit.elements.mu);
            let (r_p, _) = absolute_state(parent, &locals, &root_pos, &mut cache);
            if (r_ship - r_p).length() > r_soi_p {
                let gp = p_orbit.parent;
                let gp_mu = p_orbit.elements.mu;       // = grandparent's G·M (invariant)
                let (r_gp, v_gp) = absolute_state(gp, &locals, &root_pos, &mut cache);
                orbit.elements = OrbitalElements::from_state(r_ship - r_gp, v_ship - v_gp, gp_mu, t);
                orbit.parent = gp;
                continue;
            }
        }

        // DESCEND: inside a sibling's SOI?
        for (b_e, b_orbit, b_body) in &bodies {
            if b_orbit.parent != parent || b_e == ship_e { continue; }
            let r_soi_b = soi_radius(b_orbit.elements.a, b_body.mu, b_orbit.elements.mu);
            let (r_b, v_b) = absolute_state(b_e, &locals, &root_pos, &mut cache);
            if (r_ship - r_b).length() < r_soi_b {
                orbit.elements = OrbitalElements::from_state(r_ship - r_b, v_ship - v_b, b_body.mu, t);
                orbit.parent = b_e;
                break;
            }
        }
    }
}

pub fn draw_soi(
    mut gizmos: Gizmos, 
    camera: Single<&WorldPos, With<Camera>>,
    bodies: Query<(&Orbit, &Body, &WorldPos)>,
) {
    let cam = **camera;
    for (orbit, body, wp) in &bodies {
        let r = soi_radius(orbit.elements.a, body.mu, orbit.elements.mu);
        let center = wp.to_render_space(cam);
        gizmos.sphere(Isometry3d::from_translation(center), r as f32, Color::srgb(0.2, 0.5, 0.3));
    }
}
