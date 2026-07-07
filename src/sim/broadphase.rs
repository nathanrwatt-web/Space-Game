use std::collections::HashMap;

use bevy::math::DVec3;
use bevy::prelude::*;

use crate::sim::clock::SimClock;
use crate::sim::orbit::{Body, Maneuvers, Orbit};
use crate::sim::soi::soi_radius;

// A sibling body in its parent frame's local coordinates.
#[derive(Clone, Copy, Debug)]
pub struct BodyEntry {
    pub entity: Entity,
    pub pos: DVec3,
    pub vel: DVec3,
    pub soi: f64,
    pub mu: f64,
}

#[derive(Resource, Default)]
pub struct FrameSpaceCache {
    siblings: HashMap<Entity, Vec<BodyEntry>>,
}

// returbn children with the same parent frame
impl FrameSpaceCache {
    pub fn siblings(&self, frame: Entity) -> &[BodyEntry] {
        self.siblings
            .get(&frame)
            .map_or(&[][..], |entries| entries.as_slice())
    }
}

// insert all bodies into parents cache, grouping siblings 
pub fn build_frame_cache(
    clock: Res<SimClock>,
    mut cache: ResMut<FrameSpaceCache>,
    bodies: Query<(Entity, &Orbit, &Body), Without<Maneuvers>>,
) {
    let t = clock.t;
    cache.siblings.clear();

    for (entity, orbit, body) in &bodies {
        let (pos, vel) = orbit.elements.state_vectors_at(t);
        let entry = BodyEntry {
            entity,
            pos,
            vel,
            soi: soi_radius(orbit.elements.a, body.mu, orbit.elements.mu),
            mu: body.mu,
        };
        cache.siblings.entry(orbit.parent).or_default().push(entry);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn missing_frame_returns_empty_slice() {
        let cache = FrameSpaceCache::default();
        assert!(cache.siblings(Entity::PLACEHOLDER).is_empty());
    }
}
