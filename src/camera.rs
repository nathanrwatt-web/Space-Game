use bevy::prelude::*;
use bevy::math::{DVec3, DQuat};
use bevy::input::mouse::AccumulatedMouseScroll;
use bevy_egui::input::EguiWantsInput;

use crate::world_pos::WorldPos;
use crate::body_traits::Focusable;


// ===== SETTINGS ===== 
const ARC_RATE:     f64 = 1.2;      // rads/s for rotating on great circle 
const ZOOM_STEP:    f64 = 0.95;     // multilpier per wheel notch of mouse wheel 
const MIN_DIST:     f64 = 25.0;     // min dist from the body of focus 
const MAX_DIST:     f64 = 5.0e11;   // max dist from the body of focus 
const FOCUS_EASE:   f64 = 20.0;     

#[derive(Component)]
pub struct OrbitCam {
    pub focus: Entity,      // body of focus 
    pub focus_point: DVec3,
    pub orientation: DQuat,
    pub distance: f64,
    pub last_focus: Entity, 
    pub last_focus_pos: DVec3,
}


// Psuedo Code 
//  Change the new orientation of the camera 
//   Calulate the zoom change 
//  Calulate the position of the body being orbited 
//  Calculate the offest of the zoom change 
//  Update the new distance with the offest 
//  Update the new rotation with the new orientation 
pub fn orbit_camera(
    keys: Res<ButtonInput<KeyCode>>,
    scroll: Res<AccumulatedMouseScroll>,
    time: Res<Time>,
    bodies: Query<&WorldPos, Without<Camera>>,
    mut cam: Single<(&mut OrbitCam, &mut WorldPos, &mut Transform), With<Camera>>,
) {
    let (orbit, cam_pos, transform) = &mut *cam;
    let dt = time.delta_secs() as f64;
    let step = ARC_RATE * dt;

    // possible movement directions 
    let arcs = [
        (KeyCode::KeyQ, DQuat::from_rotation_y(step)),
        (KeyCode::KeyE, DQuat::from_rotation_y(-step)),
        (KeyCode::KeyR, DQuat::from_rotation_x(step)),
        (KeyCode::KeyF, DQuat::from_rotation_x(-step)),
    ];

    for (key, delta) in arcs {
        if keys.pressed(key) {
            orbit.orientation = (orbit.orientation * delta).normalize(); // stops drift 
            break; // only do one key 
        }
    }

    let notches = scroll.delta.y as f64;
    if notches != 0.0 {
        // change distance based on amount scrolled
        orbit.distance = (orbit.distance * ZOOM_STEP.powf(-notches)).clamp(MIN_DIST, MAX_DIST);
    }

    // get the world positition of the entity being orbited or default back to focus_point 
    let target = bodies.get(orbit.focus).map(|w| w.0).unwrap_or(orbit.focus_point);

    // if still focusing on the same planet: 
        if orbit.focus == orbit.last_focus {
        let last = orbit.last_focus_pos;        // copy out: can't hold two field-borrows through Mut<>
        orbit.focus_point += target - last;     // displacement since last frame (minus!)
    }
    let a = 1.0 - (-FOCUS_EASE * dt).exp();
    orbit.focus_point = orbit.focus_point.lerp(target, a);

    orbit.last_focus = orbit.focus;
    orbit.last_focus_pos = target;


    let offset = orbit.orientation * (DVec3::Z * orbit.distance);
    cam_pos.0 = orbit.focus_point + offset;
    transform.rotation = orbit.orientation.as_quat();
}

// when a focusable entity is clicked, change the focus 
pub fn focus_on_click(
    click: On<Pointer<Click>>,
    egui_wants: Res<EguiWantsInput>,
    bodies: Query<(&WorldPos, &Focusable)>,
    mut cam: Single<&mut OrbitCam>,
) {
    // if pointer is over the debug window 
    if egui_wants.wants_any_pointer_input() { return; }
    if click.event.button == PointerButton::Primary && bodies.get(click.entity).is_ok() {
        cam.focus = click.entity;
    }
}

