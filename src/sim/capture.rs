use bevy::prelude::*;
use crate::sim::clock::SimClock;
use crate::sim::orbit::Orbit;
use crate::math::orbital_elements::OrbitalElements;


// precomputed capture
#[derive(Component)]
pub struct ScheduledCapture {
    pub execute_at: f64,
    pub parent: Entity, 
    pub elements: OrbitalElements,
}

pub fn execute_capture(
    clock: Res<SimClock>,
    mut commands: Commands, 
    mut ships: Query<(Entity, &mut Orbit, &ScheduledCapture)>,
) {
    for (entity, mut orbit, cap) in &mut ships {
        if clock.t >= cap.execute_at {
            orbit.parent = cap.parent;
            orbit.elements = cap.elements;
            commands.entity(entity).remove::<ScheduledCapture>();
        }
    }
}
