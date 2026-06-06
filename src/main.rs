mod world_pos; 
mod orbital_elements;
mod orbit;
mod clock;

use orbit::{Orbit, propagate_orbits};
use clock::{SimClock, warp_keys, advance_clock};
use world_pos::WorldPos;
use bevy::{
    math::DVec3,
    prelude::*
};

use crate::orbital_elements::OrbitalElements;

#[derive(Component)]
struct CamLook {
    rate: f32, 
    responsiveness: f32, 
    ang_vel: Vec3,
}

impl Default for CamLook {
    fn default() -> Self {
        Self {
            rate: 1.5,
            responsiveness: 10.0,
            ang_vel: Vec3::ZERO,
        }
    }
}

fn main() {
   App::new()
       .add_plugins(DefaultPlugins.set(WindowPlugin {
           primary_window: Some(Window {
               title: "Orbital".into(),
               ..default()
           }),
           ..default()
       }))
       .add_systems(Startup, setup)
       .init_resource::<SimClock>()
       .add_systems(Update, (warp_keys, advance_clock, propagate_orbits).chain())
       .add_systems(Update, (move_camera, rotate_camera))
       .add_systems(PostUpdate, sync_render_space /* .before(transform-propagation set) */)
       .run();
}

const FAR: f64 = 1.0e13;

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
            Mesh3d(meshes.add(Sphere::new(20.0))),
            MeshMaterial3d(materials.add(Color::srgb(1.0, 0.9, 0.4))),
            Transform::default(),
            WorldPos::ORIGIN,
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
            Mesh3d(meshes.add(Sphere::new(8.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.4, 0.6, 1.0))),
            Transform::default(),
            WorldPos::ORIGIN,
            Orbit {
                elements: OrbitalElements::new(
                    200.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, mu_for_period(200.0, 8.0 * day)),
                parent: star,
            },
    )).id();

    // moon 
    commands.spawn((
        Mesh3d(meshes.add(Sphere::new(3.0))),
        MeshMaterial3d(materials.add(Color::srgb(0.7, 0.7, 0.7))),
        Transform::default(),
        WorldPos::ORIGIN,
        Orbit {
            elements: OrbitalElements::new(
                40.0, 0.0, 0.3, 0.0, 0.0, 0.0, 0.0,
                mu_for_period(40.0, 2.0 * day),         // mu = PLANET's G·M
            ),
            parent: planet,
        },
    ));

    // camera 
    commands.spawn((
            Camera3d::default(),
            Transform::default(),
            WorldPos::new(0.0, 120.0, 450.0),
            CamLook::default(),
    ));
}

fn move_camera(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut cam: Single<(&mut WorldPos, &Transform), With<Camera>>,
) {
    let (world_pos, transform) = &mut *cam;
    let mut dir = DVec3::ZERO;
    if keys.pressed(KeyCode::KeyW) { dir.z -= 1.0; }
    if keys.pressed(KeyCode::KeyS) { dir.z += 1.0; }
    if keys.pressed(KeyCode::KeyA) { dir.x -= 1.0; }
    if keys.pressed(KeyCode::KeyD) { dir.x += 1.0; }
    if keys.pressed(KeyCode::Space) { dir.y += 1.0; }
    if keys.pressed(KeyCode::ShiftLeft) { dir.y -= 1.0; }

    // take the rotation quaternion  and multimply by dir vector to get new direciton 
    let world_dir = transform.rotation.as_dquat() * dir.normalize_or_zero();  
    let speed = 15.0; // m/s — tune wildly later
    // the only f32 is time and casting up will keep accuracy of world_pos 
    world_pos.0 += world_dir * speed * time.delta_secs() as f64; 
}

fn rotate_camera(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut cam: Single<(&mut Transform, &mut CamLook), With<Camera>>,
) {
    let (transform, look) = &mut *cam;
    let dt = time.delta_secs();

    let mut input = Vec3::ZERO;
    if keys.pressed(KeyCode::KeyV) { input.x += 1.0; } // pitch 
    if keys.pressed(KeyCode::KeyC) { input.x -= 1.0; } // pitch 
    if keys.pressed(KeyCode::KeyQ) { input.y += 1.0; } // yaw
    if keys.pressed(KeyCode::KeyE) { input.y -= 1.0; } // yaw
    if keys.pressed(KeyCode::KeyF) { input.z += 1.0; } // roll 
    if keys.pressed(KeyCode::KeyG) { input.z -= 1.0; } // roll

    // where we are going 
    let target = input * look.rate;
    // how much of the gap between heree and target we will close 
    // uses 1 - e^(-k * dt) to make it frame independent 
    let t = 1.0 - (-look.responsiveness * dt).exp();
    // moves a fraction t of the way towards target 
    look.ang_vel = look.ang_vel.lerp(target, t);

    let delta = Quat::from_scaled_axis(look.ang_vel * dt);
    transform.rotation = (transform.rotation * delta).normalize();
}

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
