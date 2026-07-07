use bevy::prelude::*;
use serde::{Deserialize, Serialize};

// Marker for entities that belong to the loaded simulation world.
#[derive(Component, Default)]
pub struct SimEntity;

#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum SimulationTier {
    Background,
    Local,
    #[default]
    Rendered,
}
