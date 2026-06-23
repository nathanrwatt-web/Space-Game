use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;
use std::f64::consts::{PI, TAU};

use crate::camera::OrbitCam;
use crate::debug::DebugUi;
use crate::game_state::GameState;
use crate::sim::clock::SimClock;
use crate::sim::orbit::Orbit;
use crate::world_pos::WorldPos;


#[derive(Clone, Copy, PartialEq)]
pub enum HandleAxis { Radial, AlongTrack, Normal }

#[derive(Component)]
pub struct EditHandle {
    pub axis: HandleAxis,
    pub sign: f64, // +-1 for the end of the axis orientation
}

// the body the orbit gizmos act on. Set each frame by run_handle_target (Run) from the debug
// selection. None ⇒ handles hidden.
#[derive(Resource, Default)]
pub struct HandleTarget(pub Option<Entity>);

// drag sensitivities 
const K_A: f64      = 0.003; // fractional change
const K_THETA: f64 = 0.005; // radians of phase per pixel 
const K_I: f64     = 0.005; // radians per inclination per pixel

fn axis_color(axis: HandleAxis) -> Color {
    match axis {
        HandleAxis::Normal => Color::srgb(0.9, 0.25, 0.25),
        HandleAxis::AlongTrack => Color::srgb(0.25, 0.9, 0.35),
        HandleAxis::Radial => Color::srgb(0.35, 0.55, 1.0),
    }
}

pub fn spawn_handles(
    mut commands: Commands, 
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mesh = meshes.add(Sphere::new(1.0)); 
    
    // create six handles, pairs with opposit signs for each axis 
    for axis in [HandleAxis::Radial, HandleAxis::AlongTrack, HandleAxis::Normal] {
        let material = materials.add(axis_color(axis));
        for sign in [1.0_f64, -1.0] {
            commands.spawn((
                    Mesh3d(mesh.clone()),
                    MeshMaterial3d(material.clone()),
                    Transform::default(),
                    Visibility::Hidden, 
                    WorldPos::ORIGIN,
                    EditHandle { axis, sign }
            ));
        }
    }
}

// place and show the handles on the HandleTarget body (works for orbit cam + fly cam)
#[allow(clippy::type_complexity)]
pub fn position_handles(
    target: Res<HandleTarget>,
    cam: Single<&WorldPos, (With<Camera>, Without<EditHandle>)>, // disjoint from the handles
    orbits: Query<(&WorldPos, &Orbit)>,
    positions: Query<&WorldPos, Without<EditHandle>>,
    mut handles: Query<(&EditHandle, &mut WorldPos,
        &mut Visibility, &mut Transform), Without<Orbit>>,
    mut gizmos: Gizmos,
) {
    let cam_pos = cam.0;

    let found = target.0
        .and_then(|t| orbits.get(t).ok())
        .and_then(|(body_wp, orbit)| {
            positions.get(orbit.parent).ok().map(|p| (body_wp, orbit, p))
        });

    let Some((body_wp, orbit, parent_wp)) = found else {
        for (_, _, mut vis, _) in &mut handles {
            *vis = Visibility::Hidden;
        }
        return;
    };

    let r_hat = (body_wp.0 - parent_wp.0).normalize_or_zero();
    let n_hat = orbit.elements.plane_normal();
    let theta_hat = n_hat.cross(r_hat).normalize_or_zero();

    let dist = (body_wp.0 - cam_pos).length();  // distance to whichever camera is active
    let l = dist * 0.08;                         // offset, ~constant on screen
    let scale = (dist * 0.01) as f32;            // sphere size, ~constant on screen
    let body_render = (body_wp.0 - cam_pos).as_vec3();

    for (handle, mut wp, mut vis, mut transform) in &mut handles {
        let dir = match handle.axis {
            HandleAxis::Radial => r_hat,
            HandleAxis::AlongTrack => theta_hat,
            HandleAxis::Normal => n_hat,
        };

        wp.0 = body_wp.0 + dir * l * handle.sign;
        *vis = Visibility::Visible;
        transform.scale = Vec3::splat(scale);

        let tip_render = (wp.0 - cam_pos).as_vec3();
        gizmos.line(body_render, tip_render, axis_color(handle.axis));
    }
}

#[allow(clippy::too_many_arguments)]
pub fn drag_handle(
    drag: On<Pointer<Drag>>,
    target: Res<HandleTarget>,
    egui_wants: Res<EguiWantsInput>,
    clock: Res<SimClock>,
    handles: Query<&EditHandle>,
    positions: Query<&WorldPos, Without<EditHandle>>,
    mut orbits: Query<&mut Orbit>,
    cam: Single<(&Camera, &GlobalTransform, &WorldPos), With<Camera>>,
) {
    let Some(target_e) = target.0 else { return; };
    if egui_wants.wants_any_pointer_input() { return; }
    if drag.event.button != PointerButton::Primary { return; }

    let Ok(handle) = handles.get(drag.entity) else { return; };
    let (camera, cam_gt, cam_wp) = *cam;

    let Ok(mut orbit) = orbits.get_mut(target_e) else { return; };
    let Ok(body_wp) = positions.get(target_e) else { return; };
    let Ok(parent_wp) = positions.get(orbit.parent) else { return; };

    // world-space direction this handle points along
    let r_hat = (body_wp.0 - parent_wp.0).normalize_or_zero();
    let n_hat = orbit.elements.plane_normal();
    let axis_dir = match handle.axis {
        HandleAxis::Radial => r_hat,
        HandleAxis::AlongTrack => n_hat.cross(r_hat).normalize_or_zero(),
        HandleAxis::Normal => n_hat,
    };

    // how many pixels the cursor moved along that axis, projected onto the screen
    let body_render = (body_wp.0 - cam_wp.0).as_vec3();
    let tip_render = (body_wp.0 + axis_dir - cam_wp.0).as_vec3();
    let (Ok(p0), Ok(p1)) = (
        camera.world_to_viewport(cam_gt, body_render),
        camera.world_to_viewport(cam_gt, tip_render),
    ) else { return; };
    let screen_axis = p1 - p0;
    if screen_axis.length() < 1e-3 { return; }
    let s = drag.event.delta.dot(screen_axis.normalize()) as f64;

    // reanchor
    let mut el = orbit.elements;
    el.m0 = el.mean_anomaly_at(clock.t);
    el.epoch = clock.t;
    match handle.axis {
        HandleAxis::Radial => el.a = (el.a * (1.0 + K_A * s)).max(1e-3),
        HandleAxis::AlongTrack => el.m0 = (el.m0 + K_THETA * s).rem_euclid(TAU),
        HandleAxis::Normal => el.i = (el.i + K_I * s).clamp(0.0, PI - 1e-6),
    }
    orbit.elements = el;
}

// Run path (run_if AppMode::Run): show handles on the debug selection while in Editing.
pub fn run_handle_target(
    mode: Res<State<GameState>>,
    debug: Res<DebugUi>,
    cam: Single<&OrbitCam, With<Camera>>,
    mut target: ResMut<HandleTarget>,
) {
    target.0 = if *mode.get() == GameState::Editing {
        Some(debug.selected.unwrap_or(cam.focus))
    } else {
        None
    };
}

