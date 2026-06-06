use bevy::prelude::*;
use bevy::math::DVec3;
use std::collections::HashMap;

use crate::clock::SimClock;
use crate::world_pos::WorldPos;
use crate::orbital_elements::OrbitalElements;


#[derive(Component)] 
pub struct Orbit {
    pub elements: OrbitalElements, // math of the specific orbit 
    pub parent: Entity, // id of what it orbits 
}
// note: we keep track of the paretn ourself since the bevy ChildOf works in f32

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
