use crate::math::orbital_elements::OrbitalElements;
use crate::world_pos::WorldPos;
use crate::body_traits::Focusable;
use crate::camera::OrbitCam;
use crate::sim::orbit::{Body, Orbit, Maneuvers};
use crate::sim::clock::SimClock;
use crate::worlds::{self, CurrentWorld, WorldMeta};

use bevy::math::DQuat;
use bevy::prelude::*;
use serde::{Serialize, Deserialize};
use std::collections::HashMap;

#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum GameState {
    #[default]
    MainMenu,
    Loading,
    Running,
    Editing,
    Paused,
    Saving,
}

// gameplay systems run only inside a loaded world (not on the start screen / during load)
pub fn in_game(state: Res<State<GameState>>) -> bool {
    !matches!(state.get(), GameState::MainMenu | GameState::Loading)
}

#[derive(Serialize, Deserialize)]
struct SystemFile {
    sim_time: f64,
    bodies: Vec<BodyDescription>,
    camera: Option<CameraDescription>,
}

// each body neads to spawn with:
//  name (since entity is reattributed)
//  parent and orbital elements (to specify the orbit)
//  optional world_pos for the root body
//  if the body is focusable 

#[derive(Serialize, Deserialize)]
struct BodyDescription {
    name: String,
    parent: Option<String>,            // May be the root
    orbital_elements: Option<OrbitalElements>,
    world_pos: Option<WorldPos>,       // May be child with relative position
    mass: Option<BodyMass>,
    focusable: bool,
    appearance: Appearance,
    maneuvers: Option<Maneuvers>,      // Some ⇒ this is a ship (carries its burn queue)
}

impl BodyDescription {
    fn new(name: String, parent: Option<String>, or_els: Option<[f64; 8]>,
        world_pos: Option<WorldPos>, m: Option<(f64, f64)>, focusable: bool, appearance: Appearance)  -> Self {
            Self {
                name,
                parent, 
                orbital_elements: match or_els {
                    Some(e) => { Some( OrbitalElements {
                        a: e[0], e: e[1], i: e[2], lan: e[3],
                        arg_pe: e[4], m0: e[5], epoch: e[6], mu: e[7],
                    })},
                    _ => None
                },
                world_pos,
                mass: match m {
                    Some((m, r)) => { Some(BodyMass {mu: m, radius: r}) },
                    _ => None,
                },
                focusable,
                appearance,
                maneuvers: None, // ships are built as struct literals with Some(..)
            }
    }
}
// orbit cam needs: 
//  Focus entity and point 
//  orientation 
//  distance 
//  last focus and last point *need not be stored 
//
//  additionally needs worldpos 
#[derive(Serialize, Deserialize)]
struct CameraDescription {
    focus_entity: String, 
    orientation: DQuat, 
    distance: f64,
    world_pos: Option<WorldPos>,
}

#[derive(Serialize, Deserialize)]
struct BodyMass {
    mu: f64, 
    radius: f64
}

#[derive(Component, Serialize, Deserialize, Clone)]
pub(crate) enum Appearance {
    Sphere { radius: f32, color: [f32; 3] },
    Mesh { path: String, scale: f32 },
}

// Tab flips between modes
pub fn toggle_mode(
    keys: Res<ButtonInput<KeyCode>>,
    mode: Res<State<GameState>>,
    mut next: ResMut<NextState<GameState>>,
) {
    if keys.just_pressed(KeyCode::Tab) {
        next.set(match mode.get() {     // Resources are singular mode.get returns GameState::
            GameState::Running => GameState::Editing,
            GameState::Editing => GameState::Running,
            keep => *keep,
        });
    }
    if keys.just_pressed(KeyCode::Escape) {
        next.set(match mode.get() {
            GameState::Paused => GameState::Running,                      // close menu / resume
            GameState::Running | GameState::Editing => GameState::Paused, // open menu
            keep => *keep,                                               // ignore mid load/save
        });
    }
}

// OnEnter(MainMenu): tear down the loaded world so we can return to the start screen.
// Uses plain queries (not Single) so it's a harmless no-op the first time MainMenu is entered,
// before Startup has spawned the camera.
pub fn despawn_world(
    mut commands: Commands,
    bodies: Query<Entity, With<Appearance>>,
    mut cams: Query<&mut OrbitCam>,
    mut current: ResMut<CurrentWorld>,
) {
    for e in &bodies {
        commands.entity(e).despawn();
    }
    for mut cam in &mut cams {
        cam.focus = Entity::PLACEHOLDER;
        cam.last_focus = Entity::PLACEHOLDER;
    }
    current.0 = None;
}


// OnEnter(Loading): load the current world's folder (seeding a default if it's a new world),
// spawn it, reconfigure the persistent camera, freeze the clock, then enter Running.
pub fn load_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut clock: ResMut<SimClock>,
    mut next: ResMut<NextState<GameState>>,
    current: Res<CurrentWorld>,
    camera: Single<(&mut OrbitCam, &mut WorldPos)>,
) {
    let Some(name) = current.0.as_deref() else {
        error!("load_scene: no current world set; returning to menu");
        next.set(GameState::MainMenu);
        return;
    };

    let path = worlds::system_path(name);
    let file: SystemFile = match worlds::read_ron::<SystemFile>(&path) {
        Ok(f) => f,
        Err(e) => {
            // new world (or unreadable) → seed from the default scene and persist it
            info!("load '{name}': {e}; seeding default");
            let file = default_system();
            if let Err(e) = worlds::write_ron(&path, &file) {
                error!("load: couldn't write default world: {e}");
            }
            worlds::write_ron(&worlds::meta_path(name), &WorldMeta { sim_time: file.sim_time }).ok();
            file
        }
    };

    let by_name = spawn_system(&file, &mut commands, &mut meshes, &mut materials);
    clock.t = file.sim_time;
    clock.pause(); // worlds always load frozen (warp 0)

    // reconfigure the persistent camera (spawned at Startup) from the saved description
    let (mut orbit_cam, mut cam_wp) = camera.into_inner();
    if let Some(desc) = &file.camera {
        let focus = by_name.get(&desc.focus_entity).copied().unwrap_or(Entity::PLACEHOLDER);
        orbit_cam.focus = focus;
        orbit_cam.last_focus = focus;
        orbit_cam.orientation = desc.orientation;
        orbit_cam.distance = desc.distance;
        if let Some(wp) = desc.world_pos {
            cam_wp.0 = wp.0;
        }
    }

    next.set(GameState::Running);
}

// calls commands.spawn to populated 
fn spawn_system(
    file: &SystemFile,
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) -> HashMap<String, Entity> {
    let mut by_name: HashMap<String, Entity> = HashMap::new();

    // pass 1 — everything that doesn't depend on the parent
    // since enitty doesn't persist, names must be loaded before orbits are 
    for body in &file.bodies {
        let (mesh, material) = match &body.appearance {
            Appearance::Sphere { radius, color } => (
                meshes.add(Sphere::new(*radius)),
                materials.add(Color::srgb(color[0], color[1], color[2])),
            ),
            Appearance::Mesh { .. } => {
                warn!("Appearance::Mesh not handled yet for {}; using a placeholder", body.name);
                (meshes.add(Sphere::new(10.0)), materials.add(Color::WHITE))
            }
        };

        let mut ec = commands.spawn((
            Name::new(body.name.clone()),
            Mesh3d(mesh),
            MeshMaterial3d(material),
            Transform::default(),
            body.world_pos.unwrap_or(WorldPos::ORIGIN),
            body.appearance.clone(), // keep it on the entity so saving can read radius/color back
        ));
        if body.focusable {
            ec.insert(Focusable::default());
        }
        if let Some(m) = &body.mass {
            ec.insert(Body { mu: m.mu, radius: m.radius });
        }
        if let Some(man) = &body.maneuvers {
            ec.insert(man.clone()); // Some ⇒ ship
        }
        by_name.insert(body.name.clone(), ec.id()); // string -> entity
    }

    // pass 2 — attach orbits 
    for body in &file.bodies {
        let Some(elements) = body.orbital_elements else { continue }; // roots have no orbit
        let Some(parent_name) = &body.parent else { continue };
        let Some(&parent) = by_name.get(parent_name) else {
            warn!("body {} references unknown parent {}", body.name, parent_name);
            continue;
        };
        commands.entity(by_name[&body.name]).insert(Orbit { elements, parent });
    }
    by_name
}

// OnEnter(Saving): serialize the live world to RON, then return to the pause menu.
pub fn save_scene(
    clock: Res<SimClock>,
    current: Res<CurrentWorld>,
    bodies: Query<(
        &Name,
        &Appearance,
        &WorldPos,
        Option<&Orbit>,
        Option<&Body>,
        Option<&Maneuvers>,
        Has<Focusable>,
    )>,
    names: Query<&Name>,                       // second lookup: parent Entity -> name
    camera: Single<(&OrbitCam, &WorldPos)>,
    mut next: ResMut<NextState<GameState>>,
) {
    let Some(name) = current.0.as_deref() else {
        warn!("save: no current world; nothing written");
        next.set(GameState::Paused);
        return;
    };

    let mut descs = Vec::new();
    for (name, appearance, world_pos, orbit, body, maneuvers, focusable) in &bodies {
        descs.push(BodyDescription {
            name: name.as_str().to_string(),
            // resolve the parent Entity back to its name (None for roots)
            parent: orbit.and_then(|o| names.get(o.parent).ok())
                .map(|n| n.as_str().to_string()),
            orbital_elements: orbit.map(|o| o.elements),
            world_pos: orbit.is_none().then_some(*world_pos), // roots only; children are derived
            mass: body.map(|b| BodyMass { mu: b.mu, radius: b.radius }),
            focusable,
            appearance: appearance.clone(),
            maneuvers: maneuvers.cloned(),
        });
    }

    let (orbit_cam, cam_wp) = *camera;
    let file = SystemFile {
        sim_time: clock.t,
        bodies: descs,
        camera: Some(CameraDescription {
            focus_entity: names.get(orbit_cam.focus).map(|n| n.as_str().to_string()).unwrap_or_default(),
            orientation: orbit_cam.orientation,
            distance: orbit_cam.distance,
            world_pos: Some(*cam_wp),
        }),
    };

    match worlds::write_ron(&worlds::system_path(name), &file) {
        Ok(()) => {
            worlds::write_ron(&worlds::meta_path(name), &WorldMeta { sim_time: file.sim_time }).ok();
            info!("saved {} bodies to world '{name}'", file.bodies.len());
        }
        Err(e) => error!("save: write failed: {e}"),
    }

    next.set(GameState::Paused);
}


// default setup 
fn default_system() -> SystemFile {
    let jupiter_mu = 126.687;
    SystemFile {
        sim_time: 0.0,
        camera: Some(CameraDescription {
            focus_entity: "Jupiter".into(),
            orientation: DQuat::from_rotation_x(-0.6),
            distance: 25000.0,
            world_pos: Some(WorldPos::new(0.0, 10000.0, 25000.0)),
        }),
        bodies: vec![
            // root: no parent, no orbit, pinned at the origin
            BodyDescription::new(
                "Jupiter".into(), None, None,
                Some(WorldPos::ORIGIN), Some((jupiter_mu, 699.0)), true,
                Appearance::Sphere { radius: 699.0, color: [0.80, 0.60, 0.40] },
            ),
            BodyDescription::new(
                "Io".into(), Some("Jupiter".into()),
                Some([4218.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, jupiter_mu]),
                None, Some((5.96e-3 * 2.0, 36.4)), true,
                Appearance::Sphere { radius: 36.4, color: [0.90, 0.85, 0.40] },
            ),
            BodyDescription::new(
                "Europa".into(), Some("Jupiter".into()),
                Some([6711.0, 0.0, 0.0, 0.0, 0.0, 2.5, 0.0, jupiter_mu]),
                None, Some((3.20e-3 * 2.0, 31.2)), true,
                Appearance::Sphere { radius: 31.2, color: [0.85, 0.85, 0.90] },
            ),
            BodyDescription::new(
                "Ganymede".into(), Some("Jupiter".into()),
                Some([10704.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, jupiter_mu]),
                None, Some((9.89e-3 * 2.0, 52.6)), true,
                Appearance::Sphere { radius: 52.6, color: [0.60, 0.55, 0.50] },
            ),
            BodyDescription::new(
                "Callisto".into(), Some("Jupiter".into()),
                Some([18827.0, 0.0, 0.0, 0.0, 0.0, 5.5, 0.0, jupiter_mu]),
                None, Some((7.18e-3 * 2.0, 48.2)), true,
                Appearance::Sphere { radius: 48.2, color: [0.40, 0.40, 0.45] },
            ),
            // ship: massless, on rails around Jupiter, marked by Some(maneuvers)
            BodyDescription {
                name: "Ship".into(),
                parent: Some("Jupiter".into()),
                orbital_elements: Some(OrbitalElements {
                    a: 3000.0, e: 0.0, i: 0.0, lan: 0.0,
                    arg_pe: 0.0, m0: 0.0, epoch: 0.0, mu: jupiter_mu,
                }),
                world_pos: None,
                mass: None,
                focusable: true,
                appearance: Appearance::Sphere { radius: 15.0, color: [1.0, 0.3, 0.3] },
                maneuvers: Some(Maneuvers::default()),
            },
        ],
    }
}

