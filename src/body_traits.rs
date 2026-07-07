use bevy::prelude::*;
use serde::{Deserialize, Serialize};

// to be used when a body is Focusable by the camera
#[derive(Component, Default, Serialize, Deserialize)]
pub struct Focusable {}
