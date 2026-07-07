mod app;
mod assets;
mod body_traits;
mod camera;
mod debug;
mod edit;
mod game_state;
mod log_capture;
mod math;
mod menu;
mod ship_control;
mod sim;
mod world_pos;
mod worlds;

use app::{CameraPlugin, EditHandlePlugin, ShipControlPlugin, SimPlugin, UiPlugin, WorldPlugin};
use assets::AssetsPlugin;
use bevy::{diagnostic::FrameTimeDiagnosticsPlugin, log::LogPlugin, prelude::*};
use debug::DebugPlugin;
use log_capture::capture_layer;

fn main() {
    App::new()
        .add_plugins(
            DefaultPlugins
                .set(WindowPlugin {
                    primary_window: Some(Window {
                        title: "Orbital".into(),
                        ..default()
                    }),
                    ..default()
                })
                .set(LogPlugin {
                    custom_layer: capture_layer,
                    ..default()
                }),
        )
        .add_plugins(MeshPickingPlugin)
        // frame-time diagnostics: collect FPS/frame-time for the debug FPS overlay (not logged)
        .add_plugins(FrameTimeDiagnosticsPlugin::default())
        // gameplay split into cohesive plugins; ordering lives in app::GameSet
        .add_plugins((
            AssetsPlugin,
            WorldPlugin,
            SimPlugin,
            CameraPlugin,
            EditHandlePlugin,
            UiPlugin,
            DebugPlugin,
            ShipControlPlugin,
        ))
        .run();
}
