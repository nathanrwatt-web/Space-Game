use crate::math::orbital_elements::OrbitalElements;
use crate::world_pos::WorldPos;
use crate::body_traits::Focusable;
use crate::camera::OrbitCam;
use crate::edit::HandleTarget;
use crate::sim::entity::{SimEntity, SimulationTier};
use crate::sim::guidance::Guidance;
use crate::sim::integrate::{Propulsion, StateVec, ThrustCommand};
use crate::sim::orbit::{Body, Maneuvers, Orbit, OrbitPropagationCache};
use crate::sim::clock::SimClock;
use crate::worlds::{self, CurrentWorld, WorldMeta};

use bevy::math::{DQuat, DVec3};
use bevy::prelude::*;
use serde::{Serialize, Deserialize};
use std::collections::HashMap;

// main menu, game running
#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum AppMode {
    #[default]
    Menu,
    Run,
}

// AppMode::Run sub branches 
#[derive(SubStates, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[source(AppMode = AppMode::Run)]
pub enum GameState {
    #[default]
    Loading,
    Running,
    Editing,
    Paused,
    Saving,
}

pub fn not_menu(mode: Res<State<AppMode>>) -> bool {
    *mode.get() != AppMode::Menu
}

#[derive(Serialize, Deserialize)]
struct SystemFile {
    sim_time: f64,
    bodies: Vec<BodyDescription>,
    camera: Option<CameraDescription>,
}

// for correctly saving the state of a moving entity
#[derive(Serialize, Deserialize, Clone, Debug)]
#[serde(tag = "kind", rename_all = "snake_case")]
pub(crate) enum MotionDescription {
    Root {
        world_pos: WorldPos,
    },
    Orbit {
        parent: String,
        orbital_elements: OrbitalElements,
    },
    Powered {
        frame: String,
        pos: [f64; 3],
        vel: [f64; 3],
        guidance: Guidance,
    },
}

fn vec3_to_array(v: DVec3) -> [f64; 3] {
    [v.x, v.y, v.z]
}

fn array_to_vec3(v: [f64; 3]) -> DVec3 {
    DVec3::new(v[0], v[1], v[2])
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::world::CommandQueue;

    fn powered_ship_desc() -> Vec<BodyDescription> {
        vec![
            BodyDescription {
                name: "Root".into(),
                motion: Some(MotionDescription::Root {
                    world_pos: WorldPos::ORIGIN,
                }),
                parent: None,
                orbital_elements: None,
                world_pos: None,
                mass: Some(BodyMass { mu: 25.0, radius: 5.0 }),
                focusable: true,
                tier: SimulationTier::Background,
                appearance: None,
                maneuvers: None,
                propulsion: None,
            },
            BodyDescription {
                name: "Ship".into(),
                motion: Some(MotionDescription::Powered {
                    frame: "Root".into(),
                    pos: [10.0, 2.0, -1.0],
                    vel: [0.5, 1.5, 0.25],
                    guidance: Guidance::Hold,
                }),
                parent: None,
                orbital_elements: None,
                world_pos: None,
                mass: None,
                focusable: true,
                tier: SimulationTier::Local,
                appearance: None,
                maneuvers: Some(Maneuvers::default()),
                propulsion: Some(Propulsion {
                    max_accel: 3.0,
                    throttle: 0.75,
                }),
            },
        ]
    }

    #[test]
    fn powered_motion_round_trips_through_ron() {
        let motion = MotionDescription::Powered {
            frame: "Root".into(),
            pos: [10.0, 2.0, -1.0],
            vel: [0.5, 1.5, 0.25],
            guidance: Guidance::Hold,
        };
        let text = ron::ser::to_string(&motion).expect("serialize");
        let restored: MotionDescription = ron::from_str(&text).expect("deserialize");
        match restored {
            MotionDescription::Powered { frame, pos, vel, guidance } => {
                assert_eq!(frame, "Root");
                assert_eq!(pos, [10.0, 2.0, -1.0]);
                assert_eq!(vel, [0.5, 1.5, 0.25]);
                assert!(matches!(guidance, Guidance::Hold));
            }
            other => panic!("expected powered motion, got {other:?}"),
        }
    }

    #[test]
    fn spawn_system_restores_powered_ship_components() {
        let mut world = World::new();
        world.init_resource::<Assets<Mesh>>();
        world.init_resource::<Assets<StandardMaterial>>();

        let mut queue = CommandQueue::default();
        world.resource_scope(|world, mut meshes: Mut<Assets<Mesh>>| {
            world.resource_scope(|world, mut materials: Mut<Assets<StandardMaterial>>| {
                let mut commands = Commands::new(&mut queue, world);
                spawn_system(&powered_ship_desc(), &mut commands, &mut meshes, &mut materials);
            });
        });
        queue.apply(&mut world);

        let mut roots = world.query::<(Entity, &Name)>();
        let root = roots
            .iter(&world)
            .find(|(_, name)| name.as_str() == "Root")
            .map(|(entity, _)| entity)
            .expect("root spawned");

        let mut ships = world.query::<(&Name, &StateVec, &Guidance, &Propulsion, &SimulationTier)>();
        let (_, state, guidance, propulsion, tier) = ships
            .iter(&world)
            .find(|(name, ..)| name.as_str() == "Ship")
            .expect("ship spawned");

        assert_eq!(state.frame, root);
        assert_eq!(state.pos, DVec3::new(10.0, 2.0, -1.0));
        assert_eq!(state.vel, DVec3::new(0.5, 1.5, 0.25));
        assert!(matches!(guidance, Guidance::Hold));
        assert_eq!(propulsion.max_accel, 3.0);
        assert_eq!(propulsion.throttle, 0.75);
        assert_eq!(*tier, SimulationTier::Local);
    }
}

// each body neads to spawn with:
//  name (since entity is reattributed)
//  parent and orbital elements (to specify the orbit)
//  optional world_pos for the root body
//  if the body is focusable 

#[derive(Serialize, Deserialize)]
pub(crate) struct BodyDescription {
    pub(crate) name: String,
    #[serde(default)]
    pub(crate) motion: Option<MotionDescription>,
    #[serde(default)]
    pub(crate) parent: Option<String>,            // May be the root
    #[serde(default)]
    pub(crate) orbital_elements: Option<OrbitalElements>,
    #[serde(default)]
    pub(crate) world_pos: Option<WorldPos>,       // May be child with relative position
    pub(crate) mass: Option<BodyMass>,
    pub(crate) focusable: bool,
    #[serde(default)]
    pub(crate) tier: SimulationTier,
    #[serde(default)]
    pub(crate) appearance: Option<Appearance>,
    pub(crate) maneuvers: Option<Maneuvers>,      // Some ⇒ this is a ship (carries its burn queue)
    #[serde(default)]
    pub(crate) propulsion: Option<Propulsion>,    // Some ⇒ ship has engine 
}

impl BodyDescription {
    // helper for making new BodyDescriptions 
    pub(crate) fn new(name: String, parent: Option<String>, or_els: Option<[f64; 8]>,
        world_pos: Option<WorldPos>, m: Option<(f64, f64)>, focusable: bool, appearance: Appearance)  -> Self {
            Self {
                name,
                motion: None,
                parent, 
                orbital_elements: or_els.map(|e| OrbitalElements {
                        a: e[0], e: e[1], i: e[2], lan: e[3],
                        arg_pe: e[4], m0: e[5], epoch: e[6], mu: e[7],
                    }),
                world_pos,
                mass: match m {
                    Some((m, r)) => { Some(BodyMass {mu: m, radius: r}) },
                    _ => None,
                },
                focusable,
                tier: SimulationTier::Rendered,
                appearance: Some(appearance),
                maneuvers: None,  // ships are built as struct literals with Some(..)
                propulsion: None, // ditto — only ships carry propulsion
            }
    }

    fn resolved_motion(&self) -> MotionDescription {
        if let Some(motion) = &self.motion {
            return motion.clone();
        }
        if let (Some(parent), Some(orbital_elements)) = (&self.parent, self.orbital_elements) {
            return MotionDescription::Orbit {
                parent: parent.clone(),
                orbital_elements,
            };
        }
        MotionDescription::Root {
            world_pos: self.world_pos.unwrap_or(WorldPos::ORIGIN),
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
pub(crate) struct BodyMass {
    pub(crate) mu: f64,
    pub(crate) radius: f64,
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
        next.set(match mode.get() {
            GameState::Running => GameState::Editing,
            GameState::Editing => GameState::Running,
            keep => *keep,
        });
    }
    if keys.just_pressed(KeyCode::Escape) {
        next.set(match mode.get() {
            GameState::Paused => GameState::Running,                      // close menu / resume
            GameState::Running | GameState::Editing => GameState::Paused, // open menu
            keep => *keep,                                                // ignore mid load/save
        });
    }
}

// OnExit(AppMode::Run): tear down the loaded world when leaving a game (back to menu).
pub fn despawn_world(
    mut commands: Commands,
    bodies: Query<Entity, With<SimEntity>>,
    mut cams: Query<&mut OrbitCam>,
    mut current: ResMut<CurrentWorld>,
    mut target: ResMut<HandleTarget>,
    mut orbit_cache: ResMut<OrbitPropagationCache>,
) {
    for e in &bodies {
        commands.entity(e).despawn();
    }
    for mut cam in &mut cams {
        cam.focus = Entity::PLACEHOLDER;
        cam.last_focus = Entity::PLACEHOLDER;
    }
    current.0 = None;
    target.0 = None;
    *orbit_cache = OrbitPropagationCache::default();
}


// OnEnter(Loading): load the current worlds folder seeding a default if it's a new world,
// spawn it, reconfigure the camera, freeze the clock, then enter Running.
#[allow(clippy::too_many_arguments)]
pub fn load_scene(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut clock: ResMut<SimClock>,
    mut orbit_cache: ResMut<OrbitPropagationCache>,
    mut next: ResMut<NextState<GameState>>,
    mut app_next: ResMut<NextState<AppMode>>,
    current: Res<CurrentWorld>,
    camera: Single<(&mut OrbitCam, &mut WorldPos)>,
) {
    let Some(name) = current.0.as_deref() else {
        error!("load_scene: no current world set; returning to menu");
        app_next.set(AppMode::Menu);
        return;
    };

    let path = worlds::system_path(name);
    let file: SystemFile = match worlds::read_ron::<SystemFile>(&path) {
        Ok(f) => f,
        Err(e) => {
            // new world, seed from default 
            info!("load '{name}': {e}; seeding default");
            let file = default_system();
            if let Err(e) = worlds::write_ron(&path, &file) {
                error!("load: couldn't write default world: {e}");
            }
            worlds::write_ron(&worlds::meta_path(name), &WorldMeta { sim_time: file.sim_time }).ok();
            file
        }
    };

    let by_name = spawn_system(&file.bodies, &mut commands, &mut meshes, &mut materials);
    clock.t = file.sim_time;
    clock.pause(); // worlds always load frozen (warp 0)
    orbit_cache.dirty = true;

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

// returns hashmap of name of entity -> entity id on Loading
// handles the commands.spawn initialization 
pub(crate) fn spawn_system(
    bodies: &[BodyDescription],
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
) -> HashMap<String, Entity> {
    let mut by_name: HashMap<String, Entity> = HashMap::new();

    // pass 1 — everything that doesn't depend on parent lookup
    for body in bodies {
        let initial_wp = match body.resolved_motion() {
            MotionDescription::Root { world_pos } => world_pos,
            MotionDescription::Orbit { .. } | MotionDescription::Powered { .. } => WorldPos::ORIGIN,
        };

        let mut ec = commands.spawn((
            SimEntity,
            body.tier,
            Name::new(body.name.clone()),
            initial_wp,
        ));
        if let Some(appearance) = &body.appearance {
            ec.insert(appearance.clone());
            if body.tier != SimulationTier::Background {
                let (mesh, material) = match appearance {
                    Appearance::Sphere { radius, color } => (
                        meshes.add(Sphere::new(*radius)),
                        materials.add(Color::srgb(color[0], color[1], color[2])),
                    ),
                    Appearance::Mesh { .. } => {
                        warn!("Appearance::Mesh not handled yet for {}; using a placeholder", body.name);
                        (meshes.add(Sphere::new(10.0)), materials.add(Color::WHITE))
                    }
                };
                ec.insert((Mesh3d(mesh), MeshMaterial3d(material), Transform::default()));
            }
        }
        if body.focusable {
            ec.insert(Focusable::default());
        }
        if let Some(m) = &body.mass {
            ec.insert(Body { mu: m.mu, radius: m.radius });
        }
        if let Some(man) = &body.maneuvers {
            ec.insert(man.clone());
            ec.insert(body.propulsion.unwrap_or_default());
        }
        by_name.insert(body.name.clone(), ec.id());
    }

    // pass 2 — attach motion state once all names can resolve to entities
    for body in bodies {
        let entity = by_name[&body.name];
        match body.resolved_motion() {
            MotionDescription::Root { world_pos } => {
                commands.entity(entity).insert(world_pos);
            }
            MotionDescription::Orbit { parent, orbital_elements } => {
                let Some(&parent_entity) = by_name.get(&parent) else {
                    warn!("body {} references unknown parent {}", body.name, parent);
                    continue;
                };
                commands.entity(entity).insert(Orbit {
                    elements: orbital_elements,
                    parent: parent_entity,
                });
            }
            MotionDescription::Powered { frame, pos, vel, guidance } => {
                let Some(&frame_entity) = by_name.get(&frame) else {
                    warn!("body {} references unknown frame {}", body.name, frame);
                    continue;
                };
                commands.entity(entity).insert((
                    StateVec {
                        pos: array_to_vec3(pos),
                        vel: array_to_vec3(vel),
                        frame: frame_entity,
                    },
                    ThrustCommand::default(),
                    guidance,
                ));
            }
        }
    }
    by_name
}

// OnEnter(Saving): serialize the live world to RON, then return to the pause menu.
#[allow(clippy::type_complexity)]
pub fn save_scene(
    clock: Res<SimClock>,
    current: Res<CurrentWorld>,
    bodies: Query<(
        &Name,
        &WorldPos,
        Option<&Orbit>,
        Option<&StateVec>,
        Option<&Guidance>,
        Option<&Body>,
        Option<&Maneuvers>,
        Option<&Propulsion>,
        Option<&Appearance>,
        Option<&SimulationTier>,
        Has<Focusable>,
    ), With<SimEntity>>,
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
    for (name, world_pos, orbit, statevec, guidance, body, maneuvers, propulsion, appearance, tier, focusable) in &bodies {
        let motion = if let Some(orbit) = orbit {
            names.get(orbit.parent).ok().map(|parent_name| MotionDescription::Orbit {
                parent: parent_name.as_str().to_string(),
                orbital_elements: orbit.elements,
            })
        } else if let Some(statevec) = statevec {
            names.get(statevec.frame).ok().map(|frame_name| MotionDescription::Powered {
                frame: frame_name.as_str().to_string(),
                pos: vec3_to_array(statevec.pos),
                vel: vec3_to_array(statevec.vel),
                guidance: guidance.copied().unwrap_or_default(),
            })
        } else {
            Some(MotionDescription::Root { world_pos: *world_pos })
        };

        descs.push(BodyDescription {
            name: name.as_str().to_string(),
            motion,
            parent: None,
            orbital_elements: None,
            world_pos: None,
            mass: body.map(|b| BodyMass { mu: b.mu, radius: b.radius }),
            focusable,
            tier: tier.copied().unwrap_or_default(),
            appearance: appearance.cloned(),
            maneuvers: maneuvers.cloned(),
            propulsion: propulsion.copied(),
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
    // Scaled-up Jupiter system
    let jupiter_mu = 126_687.0; // 126.687 × 1000
    SystemFile {
        sim_time: 0.0,
        camera: Some(CameraDescription {
            focus_entity: "Jupiter".into(),
            orientation: DQuat::from_rotation_x(-0.6),
            distance: 250_000.0,
            world_pos: Some(WorldPos::new(0.0, 100_000.0, 250_000.0)),
        }),
        bodies: vec![
            // root: no parent, no orbit, pinned at the origin
            BodyDescription::new(
                "Jupiter".into(), None, None,
                Some(WorldPos::ORIGIN), Some((jupiter_mu, 6990.0)), true,
                Appearance::Sphere { radius: 6990.0, color: [0.80, 0.60, 0.40] },
            ),
            BodyDescription::new(
                "Io".into(), Some("Jupiter".into()),
                Some([42180.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, jupiter_mu]),
                None, Some((5.96 * 2.0, 364.0)), true,
                Appearance::Sphere { radius: 364.0, color: [0.90, 0.85, 0.40] },
            ),
            BodyDescription::new(
                "Europa".into(), Some("Jupiter".into()),
                Some([67110.0, 0.0, 0.0, 0.0, 0.0, 2.5, 0.0, jupiter_mu]),
                None, Some((3.20 * 2.0, 312.0)), true,
                Appearance::Sphere { radius: 312.0, color: [0.85, 0.85, 0.90] },
            ),
            BodyDescription::new(
                "Ganymede".into(), Some("Jupiter".into()),
                Some([107040.0, 0.0, 0.0, 0.0, 0.0, 4.0, 0.0, jupiter_mu]),
                None, Some((9.89 * 2.0, 526.0)), true,
                Appearance::Sphere { radius: 526.0, color: [0.60, 0.55, 0.50] },
            ),
            BodyDescription::new(
                "Callisto".into(), Some("Jupiter".into()),
                Some([188270.0, 0.0, 0.0, 0.0, 0.0, 5.5, 0.0, jupiter_mu]),
                None, Some((7.18 * 2.0, 482.0)), true,
                Appearance::Sphere { radius: 482.0, color: [0.40, 0.40, 0.45] },
            ),
            // ship: massless, on rails around Jupiter, marked by Some(maneuvers).
            // radius kept small (≈ 1/140 of Jupiter) for a realistic ratio; visible when zoomed in.
            BodyDescription {
                name: "Ship".into(),
                motion: None,
                parent: Some("Jupiter".into()),
                orbital_elements: Some(OrbitalElements {
                    a: 30000.0, e: 0.0, i: 0.0, lan: 0.0,
                    arg_pe: 0.0, m0: 0.0, epoch: 0.0, mu: jupiter_mu,
                }),
                world_pos: None,
                mass: None,
                focusable: true,
                tier: SimulationTier::Rendered,
                appearance: Some(Appearance::Sphere { radius: 50.0, color: [1.0, 0.3, 0.3] }),
                maneuvers: Some(Maneuvers::default()),
                // engines: max_accel ≫ local gravity (μ/r² ≈ 1.4e-4 at this orbit) so Hold/StationKeep have authority
                propulsion: Some(Propulsion { max_accel: 0.1, throttle: 1.0 }),
            },
        ],
    }
}
