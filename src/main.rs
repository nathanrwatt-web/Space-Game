mod world_pos; 

use world_pos::WorldPos;
use bevy::{
    math::{DVec3, DQuat},
    prelude::*
};

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
    commands.spawn((
            Mesh3d(meshes.add(Sphere::new(5.0))),
            MeshMaterial3d(materials.add(Color::srgb(0.4, 0.6, 1.0))),
            Transform::default(),
            WorldPos::new(FAR, 0.0, 0.0),
    ));

    commands.spawn((
            PointLight { shadows_enabled: true, ..default() },
            Transform::default(),
            WorldPos::new(FAR, 30.0, 30.0),
    ));

    commands.spawn((
            Camera3d::default(),
            Transform::default(),
            WorldPos::new(FAR, 0.0, 50.0),
            CamLook::default(),
    ));
}

fn move_camera(
    keys: Res<ButtonInput<KeyCode>>,
    time: Res<Time>,
    mut cam: Single<&mut WorldPos, With<Camera>>,
) {
    let mut dir = DVec3::ZERO;
    if keys.pressed(KeyCode::KeyW) { dir.z -= 1.0; }
    if keys.pressed(KeyCode::KeyS) { dir.z += 1.0; }
    if keys.pressed(KeyCode::KeyA) { dir.x -= 1.0; }
    if keys.pressed(KeyCode::KeyD) { dir.x += 1.0; }
    if keys.pressed(KeyCode::Space) { dir.y += 1.0; }
    if keys.pressed(KeyCode::ShiftLeft) { dir.y -= 1.0; }
    let speed = 15.0; // m/s — tune wildly later
    cam.0 += dir * speed * time.delta_secs() as f64;
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
