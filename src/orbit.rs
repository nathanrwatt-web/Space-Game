use bevy::prelude::*;
use bevy::math::DVec3;
use std::collections::HashMap;

use crate::clock::SimClock;
use crate::world_pos::WorldPos;
use crate::orbital_elements::OrbitalElements;
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

pub fn propagate_orbits(
    clock: Res<SimClock>,
    orbiters: Query<(Entity, &Orbit)>, // every orbiting object
    roots: Query<(Entity, &WorldPos), Without<Orbit>>, // bodies which don't orbit the star
    mut writeback: Query<(Entity, &mut WorldPos), With<Orbit>>,
) {
    let t = clock.t;
    
    // offsets of each item to be used later 
    let offsets: HashMap<Entity, (DVec3, Entity)> = orbiters
        .iter()
        // map each element to the offset given by its orbit 
        .map(|(e, o)| (e, (o.elements.offset_at(t), o.parent))) 
        .collect();

    // position of each entity wihthout orbit
    let root_pos: HashMap<Entity, DVec3> = roots
        .iter()
        // w.0 = DVec3, position 
        .map(|(e,w)| (e, w.0))
        .collect();
    
    // checking for previous computation
    let mut world: HashMap<Entity, DVec3> = HashMap::new();
    for e in offsets.keys().copied().collect::<Vec<_>>() {
        resolve(e, &offsets, &root_pos, &mut world);
    }

    // for each entity and its world position, 
    // if the total offset has been calculated by reolse, update position 
    for (e, mut wp) in &mut writeback {
        if let Some(&p) = world.get(&e) {
            wp.0 = p;
        }
    }
    // note: .copied() is used to transfer the references to keys to 
    // an ownable iter() of the values 
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
        let period = orbit.elements.period();
    
        // calculate 128 points through the period 
        let points = (0..SEGMENTS).map(|i| {
            let t = period * i as f64 / SEGMENTS as f64;
            let world = parent_pos + orbit.elements.offset_at(t);
            WorldPos(world).to_render_space(cam)
        });

        gizmos.lineloop(points, Color::srgb(0.35, 0.35, 0.4));
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

// recursive solver to get offset 
fn resolve(
    e: Entity, 
    offsets: &HashMap<Entity, (DVec3, Entity)>,
    root_pos: &HashMap<Entity, DVec3>,
    world: &mut HashMap<Entity, DVec3>,
) -> DVec3 {

    // check to see if the entity has been computed
    if let Some(&p) = world.get(&e) { return p; }

    // the position is offset + parent offset + ... 
    let pos = match offsets.get(&e) {
        Some(&(offset, parent)) => {
            resolve(parent, offsets, root_pos, world) + offset
        },
        None => {
            root_pos.get(&e).copied().unwrap_or(DVec3::ZERO)
        },
    };

    world.insert(e, pos);
    pos
}
