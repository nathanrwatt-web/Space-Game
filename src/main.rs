mod world_pos; 
mod orbital_elements;
mod orbit;
mod clock;
mod camera; 
mod body_traits;
mod transfer;

use camera::{OrbitCam, orbit_camera, focus_on_click};
use orbit::{Orbit, propagate_orbits, draw_orbits, execute_maneuvers, Maneuvers, Burn};
use clock::{SimClock, warp_keys, advance_clock};
use world_pos::WorldPos;
use body_traits::Focusable;
use transfer::plan_hohmann;
use bevy::{
    math::DQuat,
    prelude::*
};

use crate::orbital_elements::OrbitalElements;

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
       .add_systems(Startup, setup)
       .init_resource::<SimClock>()
       // order is important: change time warp -> add time -> calculate orbits -> update camera 
       .add_systems(Update, (
               warp_keys,
               advance_clock,
               debug_burn_key,
               execute_maneuvers,
               propagate_orbits,
               orbit_camera,
               draw_orbits
            ).chain())
       .add_systems(PostUpdate, sync_render_space /* .before(transform-propagation set) */)
       // watches for mouse click primary events on entities with focusable and world_pos 
       .add_observer(focus_on_click)
       .add_observer(hohmann_transfer)
       .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let day: f64 = 60.0 * 60.0 * 24.0;

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
            WorldPos::ORIGIN,
            Focusable::default(),
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
            Focusable{ ships_can_orbit: true },
            Orbit {
                elements: OrbitalElements {
                    a: 400.0,
                    e: 0.0,
                    i: 0.0,
                    lan: 0.0,
                    arg_pe: 0.0,
                    m0: 0.0,
                    epoch: 0.0,
                    mu: mu_for_period(200.0, 100.0 * day),
                },
                parent: star,
            },
    )).id();

    // moon 
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(5.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.7))),
        Transform::default(),
        WorldPos::ORIGIN,
        Focusable{ ships_can_orbit: true },
        Orbit {
            elements: OrbitalElements {
                a: 50.0, 
                e: 0.0,
                i: 0.0,
                lan: 0.0, 
                arg_pe: 0.0,
                m0: 0.0,
                epoch: 0.0,
                mu: mu_for_period(40.0, 10.0 * day),
            },         // mu = PLANET's G·M
            parent: planet,
        },
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
                    mu: mu_for_period(200.0, 8.0 * day),
                },
                parent: star,
            },
            Maneuvers::default(),
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
fn hohmann_transfer(
    click: On<Pointer<Click>>,
    clock: Res<SimClock>,
    mut ships: Query<(&Orbit, &mut Maneuvers)>,
    bodies: Query<(&Orbit, &Focusable), Without<Maneuvers>>,
    cam: Single<&OrbitCam, With<Camera>>,
) {
    let focused = cam.focus;
    
    if click.event.button == PointerButton::Secondary { info!("Clicked Right Mouse Button"); }
    
    if click.event.button == PointerButton::Secondary && bodies.get(click.entity).is_ok() && ships.get(focused).is_ok() {
       let mut ship = ships.get_mut(focused).unwrap();
       let orbit_elements = &ship.0.elements;
       let t = clock.t;
       let r2 = bodies.get(click.entity).unwrap().0.elements.a;

       let burns = plan_hohmann(orbit_elements, r2, t);
       ship.1.queue.push_front(burns.1);
       ship.1.queue.push_front(burns.0);
       info!("Pushed 2 burns");
    }
}
