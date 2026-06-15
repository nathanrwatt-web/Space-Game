// Free fly camera for the level editor. Drives the one persistent camera entity while in edit 

use bevy::prelude::*;
use bevy::math::{DVec3, DQuat};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy_egui::input::EguiWantsInput;

use crate::world_pos::WorldPos;
use crate::camera::OrbitCam;

const LOOK_SENS: f64 = 0.003;

// fly state (the camera entity itself carries OrbitCam, which we ignore here)
#[derive(Resource)]
pub struct EditorCamera {
    pub pos: DVec3,
    pub yaw: f64,   // around +Y
    pub pitch: f64, // around local X
    pub speed: f64, // units / second
}

impl Default for EditorCamera {
    fn default() -> Self {
        Self { pos: DVec3::new(0.0, 2000.0, 6000.0), yaw: 0.0, pitch: -0.3, speed: 2000.0 }
    }
}

#[allow(clippy::too_many_arguments)]
pub fn fly_camera(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    egui_wants: Res<EguiWantsInput>,
    time: Res<Time>,
    mut editor: ResMut<EditorCamera>,
    cam: Single<(&mut WorldPos, &mut Transform), With<OrbitCam>>,
) {
    let dt = time.delta_secs() as f64;

    // look: hold right mouse + drag (ignored when egui has the pointer)
    if mouse.pressed(MouseButton::Right) && !egui_wants.wants_any_pointer_input() {
        editor.yaw -= motion.delta.x as f64 * LOOK_SENS;
        editor.pitch -= motion.delta.y as f64 * LOOK_SENS;
        let lim = std::f64::consts::FRAC_PI_2 - 0.01;
        editor.pitch = editor.pitch.clamp(-lim, lim);
    }

    
    let rot = DQuat::from_rotation_y(editor.yaw) * DQuat::from_rotation_x(editor.pitch);
    let forward = rot * DVec3::NEG_Z;
    let right = rot * DVec3::X;

    // scroll changes move speed
    let notches = scroll.delta.y as f64;
    if notches != 0.0 {
        editor.speed = (editor.speed * 1.2_f64.powf(notches)).clamp(10.0, 1.0e9);
    }

    // WASD + Q/E move, ignored when egui wants the keyboard
    if !egui_wants.wants_any_keyboard_input() {
        let mut dir = DVec3::ZERO;
        if keys.pressed(KeyCode::KeyW) { dir += forward; }
        if keys.pressed(KeyCode::KeyS) { dir -= forward; }
        if keys.pressed(KeyCode::KeyD) { dir += right; }
        if keys.pressed(KeyCode::KeyA) { dir -= right; }
        if keys.pressed(KeyCode::KeyE) { dir += DVec3::Y; }
        if keys.pressed(KeyCode::KeyQ) { dir -= DVec3::Y; }
        let boost = if keys.pressed(KeyCode::ShiftLeft) { 5.0 } else { 1.0 };
        let speed = editor.speed; // copy out: can't read + write editor through ResMut at once
        if dir != DVec3::ZERO {
            editor.pos += dir.normalize() * speed * boost * dt;
        }
    }

    let (mut cam_wp, mut transform) = cam.into_inner();
    cam_wp.0 = editor.pos;
    transform.rotation = rot.as_quat();
}
