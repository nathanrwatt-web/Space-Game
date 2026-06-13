use bevy::prelude::*;
use bevy::math::primitives::InfinitePlane3d;
use bevy_egui::input::EguiWantsInput;

use crate::camera::OrbitCam;
use crate::sim::clock::SimClock;
use crate::sim::orbit::Orbit;
use crate::world_pos::WorldPos;

// Run is a live sim, edit is paused and bodies can be moved 
#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AppMode {
    #[default]
    Run,
    Edit,
}

// Tab flips between modes
pub fn toggle_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<State<AppMode>>,
    mut next: ResMut<NextState<AppMode>>,
) {
    if keys.just_pressed(KeyCode::Tab) {
        next.set(match mode.get() {     // Resources are singular mode.get returns AppMode. 
            AppMode::Run => AppMode::Edit,
            AppMode::Edit => AppMode::Run,
        });
    }
}

// In Edit mode, dragging a body rescales + re-phases its orbit (keeping e/i/lan/arg_pe)
// so it passes through the cursor. Children follow automatically since their 
// position is updated as pos_parent + pos_relative 
pub fn reshape_on_drag(
    drag: On<Pointer<Drag>>,
    mode: Res<State<AppMode>>,
    egui_wants: Res<EguiWantsInput>,
    clock: Res<SimClock>,
    mut orbits: Query<&mut Orbit>,
    positions: Query<&WorldPos>,
    cam: Single<(&Camera, &GlobalTransform, &WorldPos), With<OrbitCam>>,
) {
    if *mode.get() != AppMode::Edit { return; }
    if egui_wants.wants_any_pointer_input() { return; }
    if drag.event.button != PointerButton::Primary { return; }

    // dragged entity must be on rails (roots have no Orbit and stay put)
    // This is since dragiing the root would really do nothing, everything would move with it 
    let Ok(mut orbit) = orbits.get_mut(drag.entity) else { return; };
    let Ok(parent_wp) = positions.get(orbit.parent) else { return; };

    let (camera, cam_gt, cam_wp) = *cam;
    let Ok(ray) = camera.viewport_to_world(cam_gt, drag.pointer_location.position) else { return; };

    // intersect the cursor ray with the body's orbital plane through its parent,
    // all in render space (floating origin is a pure translation, so the normal is shared)
    let parent_render = (parent_wp.0 - cam_wp.0).as_vec3();
    let normal = orbit.elements.plane_normal().as_vec3();
    let Some(dist) = ray.intersect_plane(parent_render, InfinitePlane3d::new(normal)) else { return; };
    let point = (ray.get_point(dist) - parent_render).as_dvec3(); // parent-relative offset

    orbit.elements.reshape_through(point, clock.t);
}
