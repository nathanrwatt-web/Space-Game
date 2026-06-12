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
       // order is important: change time warp -> add time -> calculate orbits -> update camera 
       .add_systems(Update, (
               warp_keys, advance_clock, debug_burn_key,
               execute_maneuvers, update_soi, execute_capture, propagate_orbits,
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

    let mu_star = mu_for_period(160.0, 8.0 * day);
    let mu_earth = mu_star   * 1.0e-3;              // planet ≈ 1/1000 of the star
    let mu_moon   = mu_earth * 1.0e-2;              // moon  ≈ 1/100 of the planet
    

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
            Body{ mu: mu_star, radius: 60.0 },
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
            Body { mu: mu_earth, radius: 15.0 },
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
        Body { mu: mu_moon, radius: 5.0 },
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


fn intercept_transfer(
    click: On<Pointer<Click>>,
    mut debug: ResMut<DebugUi>,
    egui_wants: Res<EguiWantsInput>,
    clock: Res<SimClock>,
    mut commands: Commands,
    mut ships: Query<(&Orbit, &mut Maneuvers)>,
    bodies: Query<(&Orbit, &Body, &Focusable), Without<Maneuvers>>,
    cam: Single<&OrbitCam, With<Camera>>,
) {
    if !debug.open || egui_wants.wants_any_pointer_input() { return; }
    if click.event.button != PointerButton::Secondary { return; }

    let ship_e = cam.focus;
    let Ok((ship_orbit, _)) = ships.get(ship_e) else { return; };
    let Ok((target_orbit, target_body, _)) = bodies.get(click.entity) else { return; };
    if target_orbit.parent != ship_orbit.parent { return; }

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


