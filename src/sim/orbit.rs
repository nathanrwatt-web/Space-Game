use bevy::prelude::*;
use bevy::math::DVec3;
use std::collections::HashMap;

use crate::sim::clock::SimClock;
use crate::world_pos::WorldPos;
use crate::math::orbital_elements::OrbitalElements;
use std::collections::VecDeque;

#[derive(Component)] 
pub struct Orbit {
    pub(crate) elements: OrbitalElements, // math of the specific orbit 
    pub(crate) parent: Entity, // id of what it orbits 
}
// note: we keep track of the paretn ourself since the bevy ChildOf works in f32

#[derive(Component, Default)]
pub struct Burn {
    pub execute_at: f64, // time to fire 
    pub dv: DVec3, // velocity change 
}

#[derive(Component, Default)]
pub struct Maneuvers {
    pub queue: VecDeque<Burn>,
}

#[derive(Component)]
pub struct Body {
    pub mu: f64,
    pub radius: f64,
}

pub fn propagate_orbits(
    clock: Res<SimClock>,
    orbiters: Query<(Entity, &Orbit)>, // every orbiting object
    roots: Query<(Entity, &WorldPos), Without<Orbit>>, // bodies which don't orbit the star
    mut writeback: Query<(Entity, &mut WorldPos), With<Orbit>>,
) {
    let t = clock.t;
    
    // offsets of each item to be used later 
    let locals: HashMap<Entity, (DVec3, DVec3, Entity)> = orbiters
        .iter()
        // map each element to new (position, velocity, parent)
        .map(|(e, o)| {
            let (pos, vel) = o.elements.state_vectors_at(t);
            (e, (pos, vel , o.parent))
        })
        .collect();

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
