// Editor reference gizmos: an adaptive ground grid on the orbital (z=0) plane, infinite RGB
// origin axes, and a highlight on the selected body. All drawn in camera-relative render space.

use bevy::prelude::*;
use bevy::math::{DVec3, Isometry3d};

use crate::world_pos::WorldPos;
use crate::game_state::Appearance;
use super::camera::EditorCamera;
use super::EditorSelection;

const CELLS: u32 = 40; // cells per side of each grid (lines = CELLS + 1)

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
pub fn draw_editor_grid(
    mut gizmos: Gizmos,
    cam: Single<&WorldPos, With<Camera>>,
    editor: Res<EditorCamera>,
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
    editor: Res<EditorCamera>,
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
    selection: Res<EditorSelection>,
    bodies: Query<(&WorldPos, &Appearance)>,
) {
    let Some(sel) = selection.0 else { return; };
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
