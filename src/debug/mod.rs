// Debug tooling, gathered into one module: the F1 inspector panel, a toggleable free-flight
// debug camera (F2) with reference gizmos, and debug-only sim input shortcuts. All of it runs
// inside AppMode::Run.

mod debug_camera;
mod debug_sim;
mod debug_ui;

pub use debug_camera::{
    DebugCamera, debug_cam_active, debug_cam_inactive, debug_camera, draw_debug_grid,
    draw_origin_axes, draw_selection_highlight, toggle_debug_cam,
};
pub use debug_sim::{debug_burn_key, debug_guidance_keys, debug_toggle_powered};
pub use debug_ui::{
    DebugUi, MissionReadout, debug_is_open, debug_ui, fps_overlay, toggle_debug_ui,
};

use bevy::prelude::*;
use bevy_egui::EguiPrimaryContextPass;

use crate::game_state::AppMode;

// ===== debug inspector + free-flight camera + sim shortcuts =====
pub struct DebugPlugin;
impl Plugin for DebugPlugin {
    fn build(&self, app: &mut App) {
        app
            // F1 inspector state + free-flight camera state
            .init_resource::<DebugUi>()
            .init_resource::<DebugCamera>()
            // toggles (any non-menu input) + debug sim shortcuts (Run only)
            .add_systems(
                Update,
                (
                    toggle_debug_ui,
                    toggle_debug_cam,
                    debug_burn_key.run_if(in_state(AppMode::Run)),
                    debug_toggle_powered.run_if(in_state(AppMode::Run)),
                    debug_guidance_keys.run_if(in_state(AppMode::Run)),
                ),
            )
            // free-flight camera drives the persistent camera entity while active (Run only)
            .add_systems(Update, debug_camera.run_if(in_state(AppMode::Run)))
            // reference gizmos: adaptive grid + infinite axes + selection highlight, only while the
            // debug camera is active, after it moves
            .add_systems(
                Update,
                (draw_debug_grid, draw_origin_axes, draw_selection_highlight)
                    .after(debug_camera)
                    .run_if(in_state(AppMode::Run))
                    .run_if(debug_cam_active),
            )
            // F1 debug overlay: toolbar + entity panel + tool windows, plus the top-right FPS readout
            .add_systems(
                EguiPrimaryContextPass,
                (
                    debug_ui
                        .run_if(in_state(AppMode::Run))
                        .run_if(debug_is_open),
                    fps_overlay
                        .run_if(in_state(AppMode::Run))
                        .run_if(debug_is_open),
                ),
            );
    }
}
