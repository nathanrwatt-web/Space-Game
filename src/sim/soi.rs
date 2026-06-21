use bevy::prelude::*;
use bevy::math::{Isometry3d};
use crate::math::orbital_elements::OrbitalElements;
use crate::sim::orbit::{Orbit, Body, Maneuvers};
use crate::world_pos::WorldPos;
use crate::sim::clock::SimClock;
use crate::sim::broadphase::Broadphase;

// Laplace sphere of influence 
pub fn soi_radius(a: f64, mu_body: f64, mu_parent: f64) -> f64 {
    a * (mu_body / mu_parent).powf(0.4)
}

// update the sphere of influence for ships 
pub fn update_soi(
    clock: Res<SimClock>,
    bp: Res<Broadphase>,
    mut ships: Query<(Entity, &mut Orbit), With<Maneuvers>>,
    bodies: Query<(&Orbit, &Body), Without<Maneuvers>>,
) {
    let t = clock.t;
    for (_ship_e, mut orbit) in &mut ships {
        let parent = orbit.parent;
        let (ship_local, ship_vel) = orbit.elements.state_vectors_at(t); // relative to parent

        // ASCEND: outside the parent's SOI?
        if let Ok((p_orbit, p_body)) = bodies.get(parent) {
            let r_soi_p = soi_radius(p_orbit.elements.a, p_body.mu, p_orbit.elements.mu);
            if ship_local.length() > r_soi_p {
                let (p_local, p_vel) = p_orbit.elements.state_vectors_at(t); // parent rel. grandparent
                orbit.elements = OrbitalElements::from_state(
                    p_local + ship_local, p_vel + ship_vel, p_orbit.elements.mu, t);
                orbit.parent = p_orbit.parent;
                continue;
            }
        }

        // DESCEND: inside a sibling's SOI?
        for c in bp.soi_candidates(parent, ship_local) {
            if (ship_local - c.pos).length() < c.soi {
                orbit.elements = OrbitalElements::from_state(
                    ship_local - c.pos, ship_vel - c.vel, c.mu, t);
                orbit.parent = c.entity;
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
