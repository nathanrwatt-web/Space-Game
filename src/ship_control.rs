// Player ship commands. RTS-style movement: select a ship with
// debug.selected and then right click. 

use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;

use crate::world_pos::WorldPos;
use crate::debug_ui::DebugUi;
use crate::sim::clock::SimClock;
use crate::sim::orbit::{Orbit, Body, Maneuvers, shell_radius};
use crate::sim::integrate::{StateVec, ThrustCommand};
use crate::sim::guidance::Guidance;

pub fn move_order(
    mouse: Res<ButtonInput<MouseButton>>,
    egui_wants: Res<EguiWantsInput>,
    debug: Res<DebugUi>,
    clock: Res<SimClock>,
    window: Single<&Window>,
    camera: Single<(&Camera, &GlobalTransform, &WorldPos), With<Camera>>,
    coast: Query<&Orbit, With<Maneuvers>>,
    powered: Query<&StateVec, With<Maneuvers>>,
    bodies: Query<(Entity, &WorldPos, &Body)>,
    mut guidance_q: Query<&mut Guidance>,
    mut commands: Commands,
) {
    if !mouse.just_pressed(MouseButton::Right) || egui_wants.wants_any_pointer_input() { return; }
    let Some(e) = debug.selected else { return };

    let frame = if let Ok(o) = coast.get(e) { o.parent} // ship on rails 
    else if let Ok(sv) = powered.get(e) { sv.frame } // powered ship 
    else { return; }; // selected isn't a ship 

    let Ok((_, frame_wp, frame_body)) = bodies.get(frame) else { return };
    let shell = shell_radius(frame_body);

    // ray from camera to cursor position
    let (cam, cam_tf, cam_wp) = *camera;
    let Some(cursor) = window.cursor_position() else { return };
    let Ok(ray) = cam.viewport_to_world(cam_tf, cursor) else { return };

    // Depth-resolve the front-most body under the cursor, in case of parent behind moon 
    let mut nearest: Option<(f32, Entity)> = None;
    for (be, wp, b) in &bodies {
        if let Some(t) = ray_sphere_enter(ray, wp.to_render_space(*cam_wp), b.radius as f32)
            && nearest.is_none_or(|(bt, _)| t < bt)
        {
            nearest = Some((t, be));
        }
    }
    // a DIFFERENT body is the front-most hit ⇒ transfer/engage order, left to intercept_transfer
    if let Some((_, hit)) = nearest && hit != frame { return; }

    let center = frame_wp.to_render_space(*cam_wp);
    let Some(hit) = ray_sphere(ray, center, shell as f32) else { return };
    // offset from the frame body to the hit == the frame-local target (render and world
    // differ only by the camera translation, which cancels in the subtraction)
    let target = (hit - center).as_dvec3();

    if let Ok(o) = coast.get(e) {
        // engage powered, seeded from the current orbit, with the move order
        commands.entity(e).remove::<Orbit>().insert((
            StateVec::from_orbit(o, clock.t),
            ThrustCommand::default(),
            Guidance::MoveTo { target },
        ));
    } else if let Ok(mut g) = guidance_q.get_mut(e) {
        *g = Guidance::MoveTo { target };
    }
}

// Nearest forward hit of the ray with the sphere; on a miss, the silhouette point closest
// to the ray (so a click in empty space still yields a point on the shell).
fn ray_sphere(ray: Ray3d, center: Vec3, radius: f32) -> Option<Vec3> {
    let oc = ray.origin - center;
    let b = oc.dot(*ray.direction);
    let c = oc.length_squared() - radius * radius;
    let disc = b * b - c;
    if disc >= 0.0 {
        let s = disc.sqrt();
        let t = if -b - s > 0.0 { -b - s } else { -b + s };
        if t > 0.0 {
            return Some(ray.origin + *ray.direction * t);
        }
    }
    // miss (or sphere behind us): project the ray's closest approach onto the sphere
    let closest = ray.origin + *ray.direction * b.max(0.0);
    let dir = (closest - center).normalize_or_zero();
    (dir != Vec3::ZERO).then(|| center + dir * radius)
}

// Distance to the nearest forward intersection of the ray with the sphere, or None if the
// ray misses or the sphere lies entirely behind the camera. Lets us depth-sort bodies so the
// front-most one under the cursor wins (if the camera is INSIDE a body, returns the far exit).
fn ray_sphere_enter(ray: Ray3d, center: Vec3, radius: f32) -> Option<f32> {
    let oc = ray.origin - center;
    let b = oc.dot(*ray.direction);
    let c = oc.length_squared() - radius * radius;
    let disc = b * b - c;
    if disc < 0.0 { return None; }
    let s = disc.sqrt();
    let (t_near, t_far) = (-b - s, -b + s);
    if t_near > 0.0 { Some(t_near) }        // camera outside the body: entry point
    else if t_far > 0.0 { Some(t_far) }     // camera inside the body: exit point
    else { None }                           // body entirely behind the camera
}
