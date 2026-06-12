mod world_pos;
mod math;
mod sim;
mod camera;
mod body_traits;
mod debug_ui;

use camera::{OrbitCam, orbit_camera, focus_on_click};
use sim::orbit::{
    Orbit, Maneuvers, Burn, Body, 
    propagate_orbits, draw_orbits, execute_maneuvers,
};
use sim::{
    soi::{draw_soi, update_soi},
    clock::{SimClock, warp_keys, advance_clock},
    transfer::{plan_lambert_intercept, hohmann_tof},
    capture::{CaptureIntent, auto_capture},
};
use world_pos::WorldPos;
use body_traits::Focusable;
use bevy::{
    math::DQuat,
    prelude::*
};
use debug_ui::{DebugUi, toggle_debug_ui, debug_panel, debug_is_open};
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
       // order is important: change time warp -> add time -> calculate orbits -> update camera 
       .add_systems(Update, (
               warp_keys, advance_clock, debug_burn_key,
               execute_maneuvers, update_soi, auto_capture, propagate_orbits,
               orbit_camera,
            ).chain())
       .add_systems(Update, (draw_orbits, draw_soi).chain()
            .after(orbit_camera)
            .run_if(debug_is_open))
       .add_systems(PostUpdate, sync_render_space /* .before(transform-propagation set) */)
       // watches for mouse click primary events on entities with focusable and world_pos 
       .add_observer(focus_on_click)
       .add_observer(intercept_transfer)
       .add_systems(Update, toggle_debug_ui)
       .add_systems(EguiPrimaryContextPass, debug_panel)
       .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let day: f64 = 60.0 * 60.0 * 24.0;
    let mu_star:  f64 = mu_for_period(160.0, 8.0 * day);
    let mu_earth: f64 = mu_for_period(100.0, 8.0 * day);
    let mu_moon:  f64 = mu_for_period(20.0, 8.0 * day);

    commands.spawn((
        PointLight { shadows_enabled: true, ..default() },
        Transform::default(),
        WorldPos::new(120.0, 120.0, 120.0)
        )
    );

    // STAR  - no orbit, fixed position - entity id 
    let star = commands.spawn((
            Mesh3d(meshes.add(Sphere::new(60.0))),
            MeshMaterial3d(materials.add(Color::srgb(1.0, 0.9, 0.4))),
            Transform::default(),
            Body{ mu: mu_star },
            WorldPos::ORIGIN,
            Focusable::default(),
            Name::new("Star"),
    )).id();

    // Orbital Elements {
    //  a: Longest Axis,
    //  e: Eccentricicty, 
    //  i: Inclination,
    //  lan: longitiude of ascending node ,
    //  arg_pe: Argument of periapsis,
    //  m0: Mean anomaly at epoch,
    //  epoch: t_0,
    //  mu: G x M of parent
    // }
    let planet = commands.spawn((
            Mesh3d(meshes.add(Sphere::new(15.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.4, 0.6, 1.0))),
            Transform::default(),
            WorldPos::ORIGIN,
            Focusable{},
            Body { mu: mu_earth },
            Orbit {
                elements: OrbitalElements {
                    a: 400.0,
                    e: 0.0,
                    i: 0.0,
                    lan: 0.0,
                    arg_pe: 0.0,
                    m0: 0.0,
                    epoch: 0.0,
                    mu: mu_star,
                },
                parent: star,
            },
            Name::new("Planet"),
    )).id();

    // moon 
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(5.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.7))),
        Transform::default(),
        WorldPos::ORIGIN,
        Focusable{},
        Body { mu: mu_moon },
        Orbit {
            elements: OrbitalElements {
                a: 50.0, 
                e: 0.0,
                i: 0.0,
                lan: 0.0, 
                arg_pe: 0.0,
                m0: 0.0,
                epoch: 0.0,
                mu: mu_earth,
            },         // mu = PLANET's G·M
            parent: planet,
        },
        Name::new("Moon"),
    ));

    // ship 
    commands.spawn((
            Mesh3d(meshes.add(Sphere::new(5.0))),
            MeshMaterial3d(materials.add(Color::srgb(1.0, 0.3, 0.3))),
            Transform::default(),
            Focusable::default(),
            WorldPos::ORIGIN,
            Orbit {
                elements: OrbitalElements {
                    a: 160.0,
                    e: 0.0, 
                    i: 0.0, 
                    lan: 0.0, 
                    arg_pe: 0.0, 
                    m0: 0.0,
                    epoch: 0.5, 
                    mu: mu_star,
                },
                parent: star,
            },
            Maneuvers::default(),
            Name::new("Ship"),
    ));

    // camera 
    commands.spawn((
            Camera3d::default(),
            Transform::default(),
            WorldPos::new(0.0, 120.0, 450.0),
            OrbitCam {
                focus: star,
                focus_point: WorldPos::ORIGIN.0,
                orientation: DQuat::from_rotation_x(-0.6),
                distance: 600.0,
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

// right clicking while focusing a ship will 
fn intercept_transfer(
    click: On<Pointer<Click>>,
    debug: Res<DebugUi>,
    egui_wants: Res<EguiWantsInput>,
    clock: Res<SimClock>,
    mut commands: Commands,
    mut ships: Query<(&Orbit, &mut Maneuvers)>,
    bodies: Query<(&Orbit, &Focusable), Without<Maneuvers>>,
    cam: Single<&OrbitCam, With<Camera>>,
) {
    // if no debug or pointer on the debug window, or if not right click 
    if !debug.open || egui_wants.wants_any_pointer_input() { return; }
    if click.event.button != PointerButton::Secondary { return; }

    let ship_e = cam.focus;
    let Ok((ship_orbit, _)) = ships.get(ship_e) else { return; };          // focused must be a ship
    let Ok((target_orbit, _)) = bodies.get(click.entity) else { return; }; // clicked must be a body

    // both must be in the same reference frame 
    if target_orbit.parent != ship_orbit.parent { return; }

    let ship_el = ship_orbit.elements;
    let target_el = target_orbit.elements;
    let t = clock.t;
    let tof = hohmann_tof(ship_el.a, target_el.a, ship_el.mu);

    let Some(burn) = plan_lambert_intercept(&ship_el, &target_el, t, tof) else {
        info!("Lambert failed to converge");
        return;
    };

    ships.get_mut(ship_e).unwrap().1.queue.push_back(burn);
    commands.entity(ship_e).insert(CaptureIntent { target: click.entity });
    info!("Intercept planned + capture armed");
}
