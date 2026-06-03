mod world_pos; 

use world_pos::WorldPos;
use bevy::prelude::*;
use bevy::math::{DVec3, DQuat};

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
    mut cam: Single<&mut Transform, With<Camera>>,
) {
    let turn_speed = 1.5; // radians/sec — tune to taste
    let mut yaw = 0.0;
    if keys.pressed(KeyCode::KeyQ) { yaw += 1.0; } // turn left
    if keys.pressed(KeyCode::KeyE) { yaw -= 1.0; } // turn right
    cam.rotate_y(yaw * turn_speed * time.delta_secs());
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
