// Blender-style orbit camera for the level editor. Drives the one persistent camera entity
// while in Edit: MMB orbits the pivot, Shift+MMB pans, scroll zooms toward the cursor, F frames
// the selection. All motion eases toward a target for a smooth, professional feel.

use bevy::prelude::*;
use bevy::math::{DVec3, DQuat};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::window::PrimaryWindow;
use bevy_egui::input::EguiWantsInput;

use crate::world_pos::WorldPos;
use crate::camera::OrbitCam;
use crate::game_state::Appearance;
use super::{EditorFocus, EditorSelection};

const ORBIT_SENS: f64 = 0.005; // radians per pixel
const ZOOM_STEP: f64 = 0.88;   // distance multiplier per scroll notch
const SMOOTH: f64 = 16.0;      // higher = snappier easing
const WASD_SENS: f64 = 1.5;    // movement 

// orbit state: the camera sits `distance` from `pivot`, rotated by yaw then pitch.
// `_t` fields are the (input-driven) targets; the un-suffixed fields are the eased,
// currently-rendered values.
#[derive(Resource)]
pub struct EditorCamera {
    pub pivot: DVec3,
    pub yaw: f64,
    pub pitch: f64,
    pub distance: f64,
    pub pivot_t: DVec3,
    pub yaw_t: f64,
    pub pitch_t: f64,
    pub distance_t: f64,
}

impl Default for EditorCamera {
    fn default() -> Self {
        let (pivot, yaw, pitch, distance) = (DVec3::ZERO, 0.0, -0.5, 6000.0);
        Self { pivot, yaw, pitch, distance, pivot_t: pivot, yaw_t: yaw, pitch_t: pitch, distance_t: distance }
    }
}

impl EditorCamera {
    fn rot(yaw: f64, pitch: f64) -> DQuat {
        DQuat::from_rotation_y(yaw) * DQuat::from_rotation_x(pitch)
    }
    // set the targets to frame a body of the given radius
    fn frame(&mut self, target: DVec3, radius: f64) {
        self.pivot_t = target;
        self.distance_t = (radius * 4.0).max(100.0);
    }
}

#[allow(clippy::too_many_arguments)]
pub fn editor_camera(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    egui_wants: Res<EguiWantsInput>,
    time: Res<Time>,
    window: Single<&Window, With<PrimaryWindow>>,
    selection: Res<EditorSelection>,
    mut focus: ResMut<EditorFocus>,
    bodies: Query<(&WorldPos, &Appearance), Without<OrbitCam>>, // disjoint from the camera's &mut WorldPos
    mut editor: ResMut<EditorCamera>,
    cam: Single<(&Camera, &GlobalTransform, &mut WorldPos, &mut Transform), With<OrbitCam>>,
) {
    let dt = time.delta_secs() as f64;
    // check if using the ui 
    let pointer_free = !egui_wants.wants_any_pointer_input();
    let kbd_free = !egui_wants.wants_any_keyboard_input();
    let (camera, gxf, mut cam_wp, mut transform) = cam.into_inner();

    // current orientation drives screen-aligned pan and the zoom ray
    let rot = EditorCamera::rot(editor.yaw, editor.pitch);


    let right = rot * DVec3::X;
    let cam_pos = editor.pivot +  rot * (DVec3::Z * editor.distance); // dist from pivot + movement

    // Right click to drag 
    if mouse.pressed(MouseButton::Right) && pointer_free {
        let d = motion.delta; // change in mouse motion 
        // update based on change of mouse pos 
        editor.yaw_t -= d.x as f64 * ORBIT_SENS;
        editor.pitch_t -= d.y as f64 * ORBIT_SENS;

        // clamp the pitch to avoid flipping 
        let limit = std::f64::consts::FRAC_PI_2 - 0.01;
        editor.pitch_t = editor.pitch_t.clamp(-limit, limit);
    }

    if kbd_free && pointer_free {
        // scale by distance 
        let pan_speed = WASD_SENS * editor.distance * dt;

        let forward_flat = {
            let f = rot * -DVec3::Z;
            let flat = DVec3::new(f.x, 0.0, f.z);
            flat.try_normalize().unwrap_or(DVec3::ZERO)
        };
        
        // WASDEQ keys pressed  
        if keys.pressed(KeyCode::KeyW) { editor.pivot_t += forward_flat * pan_speed; }
        if keys.pressed(KeyCode::KeyS) { editor.pivot_t -= forward_flat * pan_speed; }
        if keys.pressed(KeyCode::KeyA) { editor.pivot_t -= right * pan_speed; }
        if keys.pressed(KeyCode::KeyD) { editor.pivot_t += right * pan_speed; }
        if keys.pressed(KeyCode::KeyE) { editor.pivot_t += DVec3::Y * pan_speed; }
        if keys.pressed(KeyCode::KeyQ) { editor.pivot_t -= DVec3::Y * pan_speed; }
    }

    // move around by the mosue wheel 
    let notches = scroll.delta.y as f64;
    if notches != 0.0 && pointer_free {
        let factor = ZOOM_STEP.powf(notches);
        
        if let Some(hit) = cursor_ground_hit(camera, gxf, &window, cam_pos) {
            editor.pivot_t = editor.pivot_t.lerp(hit, 1.0 - factor);
        }

        editor.distance_t = (editor.distance_t * factor).clamp(1.0, 1.0e9);
    }

    // framing 
    let frame_req = focus.0.take().or_else(|| {
        (kbd_free && keys.just_pressed(KeyCode::KeyF))
            .then_some(selection.0)
            .flatten()
    });

    if let Some(e) = frame_req && let Ok((wp, appearance)) = bodies.get(e) {
        // Choose a framing radius based on the body's appearance.
        let radius = match appearance {
            Appearance::Sphere { radius, .. } => *radius as f64,
            _ => 50.0,
        };
        editor.frame(wp.0, radius);
    }

    // update camera variables 
    let a = 1.0 - (-SMOOTH * dt).exp();

    editor.pivot = editor.pivot.lerp(editor.pivot_t, a);
    editor.yaw += (editor.yaw_t - editor.yaw) * a;
    editor.pitch   += (editor.pitch_t  - editor.pitch)  * a;
    editor.distance += (editor.distance_t - editor.distance) * a;

    // rebuild from new variables 
    let rot = EditorCamera::rot(editor.yaw, editor.pitch);
    cam_wp.0 = editor.pivot + rot * (DVec3::Z * editor.distance);
    transform.rotation = rot.as_quat();
}

// world point where the cursor ray meets the orbital plane (world z = 0), in f64.
// The camera renders at ~render-space origin, so the plane sits at render z = -cam_pos.z.
fn cursor_ground_hit(camera: &Camera, gxf: &GlobalTransform, window: &Window, cam_pos: DVec3) -> Option<DVec3> {
    let cursor = window.cursor_position()?;
    let ray = camera.viewport_to_world(gxf, cursor).ok()?;
    let o = DVec3::new(ray.origin.x as f64, ray.origin.y as f64, ray.origin.z as f64);
    let d = DVec3::new(ray.direction.x as f64, ray.direction.y as f64, ray.direction.z as f64);
    if d.z.abs() < 1e-9 { return None; }
    let t = (-cam_pos.z - o.z) / d.z;
    if t <= 0.0 { return None; }
    Some(o + d * t + cam_pos)
}
