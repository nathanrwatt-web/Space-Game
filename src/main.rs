mod world_pos;
mod math;
mod sim;
mod camera;
mod body_traits;
mod debug_ui;
mod edit;

use camera::{OrbitCam, orbit_camera, focus_on_click};
use edit::{AppMode, toggle_mode, reshape_on_drag};
use sim::orbit::{
    Orbit, Maneuvers, Burn, Body, 
    propagate_orbits, draw_orbits, execute_maneuvers,
};
use sim::{
    soi::{draw_soi, update_soi, soi_radius},
    clock::{SimClock, warp_keys, advance_clock},
    mission::plan_mission,
    capture::{ScheduledCapture, execute_capture},
};
use world_pos::WorldPos;
use body_traits::Focusable;
use bevy::{
    math::DQuat,
    prelude::*
};
use debug_ui::{DebugUi, MissionReadout, toggle_debug_ui, debug_panel, debug_is_open};
use bevy_egui::{EguiPlugin, EguiPrimaryContextPass, input::EguiWantsInput};

use crate::math::orbital_elements::OrbitalElements;

fn main() {
   App::new()
       .add_plugins(DefaultPlugins.set(WindowPlugin {
           primary_window: Some(Window {
               title: "Orbital".into(),
               ..default()
           }),
           ..default()
        }))
       .add_plugins(MeshPickingPlugin)
       .add_plugins(EguiPlugin::default())
       .init_resource::<DebugUi>()
       .add_systems(Startup, setup)
       .init_resource::<SimClock>()
       .init_state::<AppMode>()
       // order is important: change time warp -> add time -> calculate orbits -> update camera
       // advance_clock is gated to Run, so Edit mode freezes the whole sim (everything derives from t)
       .add_systems(Update, (
               warp_keys, advance_clock.run_if(in_state(AppMode::Run)), debug_burn_key,
               execute_maneuvers, update_soi, execute_capture, propagate_orbits,
               orbit_camera,
            ).chain())
       .add_systems(Update, (draw_orbits, draw_soi).chain()
            .after(orbit_camera)
            .run_if(debug_is_open))
       .add_systems(Update, apply_focus_request.before(orbit_camera))
       .add_systems(PostUpdate, sync_render_space /* .before(transform-propagation set) */)
       // watches for mouse click primary events on entities with focusable and world_pos
       .add_observer(focus_on_click)
       .add_observer(intercept_transfer)
       .add_observer(reshape_on_drag)
       .add_systems(Update, (toggle_debug_ui, toggle_mode))
       .add_systems(EguiPrimaryContextPass, debug_panel)
       .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {

    let mu_jupiter = 126.687; 

    // sun shines parallel from far away 
    commands.spawn((DirectionalLight {
        illuminance: 8000.0,
        shadows_enabled: false, 
        ..default() },
        Transform::from_rotation(Quat::from_euler(EulerRot::XYZ, -0.7, 0.5, 0.0)),
    ));

    let jupiter = commands.spawn((
        Mesh3d(meshes.add(Sphere::new(699.00))),
        MeshMaterial3d(materials.add(Color::srgb(0.80, 0.60, 0.40))),
        Transform::default(),
        Body {
            mu: mu_jupiter,
            radius: 699.00
        },
        WorldPos::ORIGIN,
        Focusable::default(),
        Name::new("Jupiter")
    )).id();

    // helper to keep the moon spawns short
    let mut moon = |a: f64, m0: f64, radius: f32, mu: f64, color: Color, name: &str| {
        commands.spawn((
            Mesh3d(meshes.add(Sphere::new(radius))),
            MeshMaterial3d(materials.add(color)),
            Transform::default(),
            WorldPos::ORIGIN,
            Focusable::default(),
            Body { mu, radius: radius as f64 },
            Orbit {
                elements: OrbitalElements {
                    a, e: 0.0, i: 0.0, lan: 0.0,
                    arg_pe: 0.0, m0, epoch: 0.0, mu: mu_jupiter,
                },
                parent: jupiter,
            },
            Name::new(name.to_string()),
        ));
    };

    // make bigger so noticable 
    moon(4218.0,  0.0, 18.2 * 2.0, 5.96e-3 * 2.0, Color::srgb(0.90, 0.85, 0.40), "Io");
    moon(6711.0,  2.5, 15.6 * 2.0, 3.20e-3 * 2.0, Color::srgb(0.85, 0.85, 0.90), "Europa");
    moon(10704.0, 4.0, 26.3 * 2.0, 9.89e-3 * 2.0, Color::srgb(0.60, 0.55, 0.50), "Ganymede");
    moon(18827.0, 5.5, 24.1 * 2.0, 7.18e-3 * 2.0, Color::srgb(0.40, 0.40, 0.45), "Callisto");

    // ship 
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(15.0))),
        MeshMaterial3d(materials.add(Color::srgb(1.0, 0.3, 0.3))),
        Transform::default(),
        Focusable::default(),
        WorldPos::ORIGIN,
        Orbit {
            elements: OrbitalElements {
                a: 3000.0, e: 0.0, i: 0.0, lan: 0.0,
                arg_pe: 0.0, m0: 0.0, epoch: 0.0, mu: mu_jupiter,
            },
            parent: jupiter,
        },
        Maneuvers::default(),
        Name::new("Ship"),
    ));

    commands.spawn((
        Camera3d::default(),
        Transform::default(),
        WorldPos::new(0.0, 10000.0, 25000.0),
        OrbitCam {
            focus: jupiter, 
            focus_point: WorldPos::ORIGIN.0,
            orientation: DQuat::from_rotation_x(-0.6),
            distance: 25000.0,
            last_focus: jupiter, 
            last_focus_pos: WorldPos::ORIGIN.0,
        },
    ));
}

// translate f64 math to f32 for rendering 
fn sync_render_space(
    camera: Single<&WorldPos, With<Camera>>,
    mut bodies: Query<(&WorldPos, &mut Transform)>,
) {
    let origin = **camera;                              // the camera's f64 position
    for (pos, mut transform) in &mut bodies {
        transform.translation = pos.to_render_space(origin); // subtract in f64, then cast
    }
}

// helper 
fn mu_for_period(a: f64, period: f64) -> f64 {
    let n = std::f64::consts::TAU / period;
    n * n * a.powi(3)
}

// for testing 
fn debug_burn_key(
    keys: Res<ButtonInput<KeyCode>>,
    clock: Res<SimClock>,
    mut ships: Query<(&Orbit, &mut Maneuvers)>,
) {
    if !keys.just_pressed(KeyCode::KeyB) { return; }
    for (orbit, mut maneuvers) in &mut ships {
        let t = clock.t;
        let v = orbit.elements.velocity_at(t);
        let dv = v * 0.1;
        maneuvers.queue.push_back(Burn { execute_at: t, dv });  
    }
}


fn intercept_transfer(
    click: On<Pointer<Click>>,
    mut debug: ResMut<DebugUi>,
    egui_wants: Res<EguiWantsInput>,
    clock: Res<SimClock>,
    mut commands: Commands,
    mut ships: Query<(&Orbit, &mut Maneuvers)>,
    bodies: Query<(&Orbit, &Body, &Focusable), Without<Maneuvers>>,
) {
    if !debug.open || egui_wants.wants_any_pointer_input() {
        info!("No egui or right clicked on egui panel");
        return; 
    }
    if click.event.button != PointerButton::Secondary { return; }

    let Some(ship_e) = debug.selected else { 
        info!("No ship selected"); 
        return;
    };

    // make sure the selected ship is indeed a ship
    let Ok((ship_orbit, _)) = ships.get(ship_e) else { 
        info!("intercept: focused entity {ship_e:?} is not a ship");
        return; 
    };

    // make sure what is clicked can be targeted 
    let Ok((target_orbit, target_body, _)) = bodies.get(click.entity) else {
        info!("intercept: clicked {:?} is not a targetable body (root/ship/occluder?)", click.entity);
        return;
    };
    
    // check if ship is returning to home body (in reference) or moving in reference frame 
    if (target_orbit.parent != ship_orbit.parent) && (click.entity != ship_orbit.parent)  { 
        info!("The parent of the target ({:?}) is not the target of the ship ({:?})",
            target_orbit.parent, ship_orbit.parent);
        return; 
    }

    let ship_el = ship_orbit.elements;
    let target_el = target_orbit.elements;
    let mu_target = target_body.mu;
    let r_soi = soi_radius(target_el.a, mu_target, ship_el.mu);
    let r_p = debug.capture_rp.unwrap_or((target_body.radius * 1.2).min(0.9 * r_soi));

    let Some(plan) = plan_mission(&ship_el, &target_el, mu_target, clock.t, r_p, &[], 0.0) else {
        info!("no mission found");
        return;
    };

    let t_dep = plan.departure.execute_at;
    ships.get_mut(ship_e).unwrap().1.queue.push_back(plan.departure);
    commands.entity(ship_e).insert(ScheduledCapture {
        execute_at: plan.t_peri,
        parent: click.entity,
        elements: plan.circular,
    });
    info!("mission planned: capture at t = {:.0}", plan.t_peri);

    debug.last_mission = Some(MissionReadout {
        t_dep,
        wait: t_dep - clock.t,
        t_peri: plan.t_peri,
        v_inf: plan.v_inf,
        dep_dv: plan.dep_dv,
        capture_dv: plan.capture_cost,
        total_dv: plan.dep_dv + plan.capture_cost,
        r_p,
        captured_a: plan.circular.a,
        captured_e: plan.circular.e,
    });
}

// double-click in the debug entity list routes here: snap focus + a framing zoom
fn apply_focus_request(
    mut ui: ResMut<DebugUi>,
    bodies: Query<&Body>,
    mut cam: Single<&mut OrbitCam, With<Camera>>,
) {
    let Some(e) = ui.focus_request.take() else { return; };  // consume once
    cam.focus = e;
    cam.distance = bodies.get(e)
        .map(|b| (b.radius * 6.0).max(15.0))  // frame the body by its radius
        .unwrap_or(300.0);                    // ships have no Body → fixed default
}
