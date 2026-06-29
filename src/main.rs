mod world_pos;
mod math;
mod sim;
mod camera;
mod body_traits;
mod debug;
mod edit;
mod log_capture;
mod game_state;
mod worlds;
mod menu;
mod ship_control;
mod app;

use app::{WorldPlugin, SimPlugin, CameraPlugin, EditHandlePlugin, UiPlugin, ShipControlPlugin};
use debug::DebugPlugin;
use log_capture::capture_layer;
use bevy::{
    diagnostic::FrameTimeDiagnosticsPlugin,
    log::LogPlugin,
    prelude::*,
};

fn main() {
   App::new()
       .add_plugins(DefaultPlugins
           .set(WindowPlugin { primary_window: Some(Window { title: "Orbital".into(), ..default() }),..default() })
           .set(LogPlugin { custom_layer: capture_layer, ..default() }))
       .add_plugins(MeshPickingPlugin)
       // frame-time diagnostics: collect FPS/frame-time for the debug FPS overlay (not logged)
       .add_plugins(FrameTimeDiagnosticsPlugin::default())
       // gameplay split into cohesive plugins; ordering lives in app::GameSet
       .add_plugins((
               WorldPlugin,
               SimPlugin,
               CameraPlugin,
               EditHandlePlugin,
               UiPlugin,
               DebugPlugin,
               ShipControlPlugin))
       .run();
}
