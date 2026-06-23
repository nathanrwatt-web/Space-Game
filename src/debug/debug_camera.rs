// Blender-style free-flight orbit camera, available as a toggleable debug tool while running a
// world (F2). When active it drives the one persistent camera entity: RMB orbits the pivot,
// WASDEQ pans, scroll zooms toward the cursor, F frames the debug selection. All motion eases
// toward a target for a smooth, professional feel. Also draws editor-style reference gizmos:
// an adaptive ground grid, infinite RGB origin axes, and a highlight on the selected body.

use bevy::prelude::*;
use bevy::math::{DVec3, DQuat, Isometry3d};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::window::PrimaryWindow;
use bevy_egui::input::EguiWantsInput;

use crate::world_pos::WorldPos;
use crate::camera::OrbitCam;
use crate::game_state::Appearance;
use super::DebugUi;

const ORBIT_SENS: f64 = 0.005; // radians per pixel
const ZOOM_STEP: f64 = 0.88;   // distance multiplier per scroll notch
const SMOOTH: f64 = 16.0;      // higher = snappier easing
const WASD_SENS: f64 = 1.5;    // movement
const CELLS: u32 = 40;         // cells per side of each grid (lines = CELLS + 1)

// orbit state: the camera sits `distance` from `pivot`, rotated by yaw then pitch.
// `_t` fields are the (input-driven) targets; the un-suffixed fields are the eased,
// currently-rendered values. `active` gates the whole tool (toggled with F2).
#[derive(Resource, Default)]
pub struct DebugCamera {
    pub active: bool,
    pub pivot: DVec3,
    pub yaw: f64,
    pub pitch: f64,
    pub distance: f64,
    pub pivot_t: DVec3,
    pub yaw_t: f64,
    pub pitch_t: f64,
    pub distance_t: f64,
}

impl DebugCamera {
    fn rot(yaw: f64, pitch: f64) -> DQuat {
        DQuat::from_rotation_y(yaw) * DQuat::from_rotation_x(pitch)
    }
    // set the targets to frame a body of the given radius
    fn frame(&mut self, target: DVec3, radius: f64) {
        self.pivot_t = target;
        self.distance_t = (radius * 4.0).max(100.0);
    }
}

// F2 toggles the free-flight camera. On activation, seed the orbit state from the current
// orbit-camera pose (pivot at the focus point, matching distance + viewing angles) so the
// view doesn't jump as control hands over.
pub fn toggle_debug_cam(
    keys: Res<ButtonInput<KeyCode>>,
    mut debug_cam: ResMut<DebugCamera>,
    cam: Single<(&WorldPos, &OrbitCam), With<Camera>>,
) {
    if !keys.just_pressed(KeyCode::F2) {
        return;
    }
    debug_cam.active = !debug_cam.active;
    if !debug_cam.active {
        return;
    }
    let (wp, orbit) = *cam;
    let pivot = orbit.focus_point;
    let offset = wp.0 - pivot;
    let distance = offset.length().max(1.0);
    let dir = offset / distance;
    // invert rot(yaw,pitch) * Z = (cos p · sin y, -sin p, cos p · cos y)
    let pitch = (-dir.y).asin();
    let yaw = dir.x.atan2(dir.z);
    *debug_cam = DebugCamera {
        active: true,
        pivot, yaw, pitch, distance,
        pivot_t: pivot, yaw_t: yaw, pitch_t: pitch, distance_t: distance,
    };
}

// while the debug camera is active, the normal orbit camera must not also drive the entity
pub fn debug_cam_active(debug_cam: Res<DebugCamera>) -> bool { debug_cam.active }
pub fn debug_cam_inactive(debug_cam: Res<DebugCamera>) -> bool { !debug_cam.active }

#[allow(clippy::too_many_arguments)]
pub fn debug_camera(
    keys: Res<ButtonInput<KeyCode>>,
    mouse: Res<ButtonInput<MouseButton>>,
    motion: Res<AccumulatedMouseMotion>,
    scroll: Res<AccumulatedMouseScroll>,
    egui_wants: Res<EguiWantsInput>,
    time: Res<Time>,
    window: Single<&Window, With<PrimaryWindow>>,
    debug: Res<DebugUi>,
    mut editor: ResMut<DebugCamera>,
    cam: Single<(&Camera, &GlobalTransform, &mut WorldPos, &mut Transform), With<OrbitCam>>,
    bodies: Query<(&WorldPos, &Appearance), Without<OrbitCam>>, // disjoint from the camera's &mut WorldPos
) {
    if !editor.active {
        return;
    }
    let dt = time.delta_secs() as f64;
    // check if using the ui
    let pointer_free = !egui_wants.wants_any_pointer_input();
    let kbd_free = !egui_wants.wants_any_keyboard_input();
    let (camera, gxf, mut cam_wp, mut transform) = cam.into_inner();

    // current orientation drives screen-aligned pan and the zoom ray
    let rot = DebugCamera::rot(editor.yaw, editor.pitch);


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

    // framing — F frames the debug-selected body
    let frame_req = (kbd_free && keys.just_pressed(KeyCode::KeyF))
        .then_some(debug.selected)
        .flatten();

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
    let rot = DebugCamera::rot(editor.yaw, editor.pitch);
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

// ===== reference gizmos (grid / axes / selection highlight) =====

// Snap a positive value down to the nearest 1 / 2 / 5 × 10^k step (grid "nice numbers").
pub(crate) fn nice(x: f64) -> f64 {
    if x <= 0.0 || !x.is_finite() {
        return 1.0;
    }
    let p = 10f64.powf(x.log10().floor());
    let m = x / p;
    let n = if m < 2.0 { 1.0 } else if m < 5.0 { 2.0 } else { 5.0 };
    n * p
}

// Adaptive grid: a fine grid at the LOD spacing plus a brighter 10× major grid, both centered
// under the camera and snapped to their spacing so the floor reads as infinite.
pub fn draw_debug_grid(
    mut gizmos: Gizmos,
    cam: Single<&WorldPos, With<Camera>>,
    editor: Res<DebugCamera>,
) {
    let cam_pos = cam.0;
    let s = nice(editor.distance.max(1.0) / 10.0);

    // fade the fine grid out as we zoom out (it gets dense on screen); major stays solid
    let fade = (2.0 - editor.distance / (s * CELLS as f64)).clamp(0.0, 1.0) as f32;
    draw_grid(&mut gizmos, cam_pos, s, Color::srgba(0.35, 0.35, 0.42, 0.5 * fade));
    draw_grid(&mut gizmos, cam_pos, s * 10.0, Color::srgba(0.5, 0.5, 0.58, 0.85));
}

fn draw_grid(gizmos: &mut Gizmos, cam_pos: DVec3, s: f64, color: Color) {
    let cx = (cam_pos.x / s).round() * s;
    let cy = (cam_pos.y / s).round() * s;
    let center = (DVec3::new(cx, cy, 0.0) - cam_pos).as_vec3();
    gizmos.grid(
        Isometry3d::from_translation(center),
        UVec2::splat(CELLS),
        Vec2::splat(s as f32),
        color,
    );
}

// Infinite RGB axes through the world origin (X red, Y green, Z blue). The half-length scales
// with the view distance so the lines always overrun the screen and read as infinite.
pub fn draw_origin_axes(
    mut gizmos: Gizmos,
    cam: Single<&WorldPos, With<Camera>>,
    editor: Res<DebugCamera>,
) {
    let origin = (-cam.0).as_vec3(); // world origin in render space
    let l = (editor.distance * 1000.0) as f32;
    gizmos.line(origin - Vec3::X * l, origin + Vec3::X * l, Color::srgb(0.9, 0.25, 0.25));
    gizmos.line(origin - Vec3::Y * l, origin + Vec3::Y * l, Color::srgb(0.3, 0.85, 0.3));
    gizmos.line(origin - Vec3::Z * l, origin + Vec3::Z * l, Color::srgb(0.35, 0.5, 1.0));
}

// Selected-body highlight: an orange wire sphere plus a small orientation triad.
pub fn draw_selection_highlight(
    mut gizmos: Gizmos,
    cam: Single<&WorldPos, With<Camera>>,
    debug: Res<DebugUi>,
    bodies: Query<(&WorldPos, &Appearance)>,
) {
    let Some(sel) = debug.selected else { return; };
    let Ok((wp, appearance)) = bodies.get(sel) else { return; };
    let radius = match appearance {
        Appearance::Sphere { radius, .. } => *radius,
        _ => 50.0,
    };
    let center = (wp.0 - cam.0).as_vec3();
    let accent = Color::srgb(1.0, 0.6, 0.1);
    gizmos.sphere(Isometry3d::from_translation(center), radius * 1.15, accent);

    let len = radius * 2.0;
    gizmos.line(center, center + Vec3::X * len, Color::srgb(0.9, 0.25, 0.25));
    gizmos.line(center, center + Vec3::Y * len, Color::srgb(0.3, 0.85, 0.3));
    gizmos.line(center, center + Vec3::Z * len, Color::srgb(0.35, 0.5, 1.0));
}

#[cfg(test)]
mod tests {
    use super::nice;

    #[test]
    fn nice_snaps_to_1_2_5() {
        assert_eq!(nice(73.0), 50.0);
        assert_eq!(nice(0.0042), 0.002);
        assert_eq!(nice(1.0), 1.0);
        assert_eq!(nice(3.0), 2.0);
        assert_eq!(nice(9.9), 5.0);
        assert_eq!(nice(150.0), 100.0);
    }
}
