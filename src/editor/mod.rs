// level editor has own EditorBody tag as well as own folder for files 
// code is seperated for future purposes

mod camera;
mod ui;

pub use camera::{EditorCamera, fly_camera};
pub use ui::editor_panel;

use bevy::prelude::*;
use bevy_egui::input::EguiWantsInput;
use serde::{Serialize, Deserialize};
use std::path::{Path, PathBuf};

use crate::game_state::{Appearance, BodyDescription, BodyMass, spawn_system};
use crate::sim::clock::SimClock;
use crate::sim::orbit::{Body, Orbit};
use crate::world_pos::WorldPos;
use crate::worlds;

pub const LEVELS_ROOT: &str = "Levels";
pub const LEVEL_FILE: &str = "level.ron";
const STEP: f64 = 3600.0; // single-step size: 1 hour

// distinct level format, which is just bodies for now 
#[derive(Serialize, Deserialize, Default)]
pub struct Level {
    pub bodies: Vec<BodyDescription>,
}

#[derive(Resource, Default)]
pub struct CurrentLevel(pub Option<String>);

#[derive(Resource, Default)]
pub struct EditorSaveRequest(pub bool);

// marks editor-spawned bodies (kept distinct from run-world bodies)
#[derive(Component)]
pub struct EditorBody;

pub fn level_dir(name: &str) -> PathBuf { Path::new(LEVELS_ROOT).join(name) }
pub fn level_path(name: &str) -> PathBuf { level_dir(name).join(LEVEL_FILE) }

pub(crate) fn fmt_time(t: f64) -> String {
    let total = t.max(0.0) as u64;
    format!(
        "{}d {:02}:{:02}:{:02}",
        total / 86400, (total % 86400) / 3600, (total % 3600) / 60, total % 60
    )
}

// a minimal starting level: one central sphere (root)
fn default_level() -> Level {
    Level {
        bodies: vec![BodyDescription::new(
            "Center".into(), None, None,
            Some(WorldPos::ORIGIN), Some((100.0, 200.0)), true,
            Appearance::Sphere { radius: 200.0, color: [0.7, 0.7, 0.8] },
        )],
    }
}

// OnEnter(AppMode::Edit): load (or seed) the current level, spawn it, reset the clock + fly cam.
pub fn editor_setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut clock: ResMut<SimClock>,
    mut editor_cam: ResMut<EditorCamera>,
    current: Res<CurrentLevel>,
) {
    let Some(name) = current.0.as_deref() else {
        warn!("editor: no current level set");
        return;
    };
    let path = level_path(name);
    let level: Level = match worlds::read_ron::<Level>(&path) {
        Ok(l) => l,
        Err(e) => {
            info!("editor '{name}': {e}; seeding default level");
            let l = default_level();
            if let Err(e) = worlds::write_ron(&path, &l) {
                error!("editor: write default: {e}");
            }
            l
        }
    };
    let by_name = spawn_system(&level.bodies, &mut commands, &mut meshes, &mut materials);
    for e in by_name.values() {
        commands.entity(*e).insert(EditorBody);
    }
    clock.t = 0.0;
    clock.pause();
    *editor_cam = EditorCamera::default();
}

// OnExit(AppMode::Edit): despawn editor bodies.
pub fn editor_teardown(mut commands: Commands, bodies: Query<Entity, With<EditorBody>>) {
    for e in &bodies {
        commands.entity(e).despawn();
    }
}

// editor time: continuous at the current rate + [ / ] single steps (analytic, so exact).
pub fn editor_time(
    keys: Res<ButtonInput<KeyCode>>,
    egui_wants: Res<EguiWantsInput>,
    time: Res<Time>,
    mut clock: ResMut<SimClock>,
) {
    clock.t += clock.warp() * time.delta_secs() as f64;
    if egui_wants.wants_any_keyboard_input() {
        return;
    }
    if keys.just_pressed(KeyCode::BracketRight) { clock.t += STEP; }
    if keys.just_pressed(KeyCode::BracketLeft) { clock.t -= STEP; }
}

// serialize the live editor bodies back to the level file (on a UI request).
#[allow(clippy::type_complexity)]
pub fn save_level(
    mut req: ResMut<EditorSaveRequest>,
    current: Res<CurrentLevel>,
    bodies: Query<(&Name, &Appearance, &WorldPos, Option<&Orbit>, Option<&Body>), With<EditorBody>>,
    names: Query<&Name>,
) {
    if !req.0 {
        return;
    }
    req.0 = false;
    let Some(name) = current.0.as_deref() else { return; };

    let mut descs = Vec::new();
    for (n, appearance, world_pos, orbit, body) in &bodies {
        descs.push(BodyDescription {
            name: n.as_str().to_string(),
            parent: orbit.and_then(|o| names.get(o.parent).ok()).map(|x| x.as_str().to_string()),
            orbital_elements: orbit.map(|o| o.elements),
            world_pos: orbit.is_none().then_some(*world_pos),
            mass: body.map(|b| BodyMass { mu: b.mu, radius: b.radius }),
            focusable: true,
            appearance: appearance.clone(),
            maneuvers: None,
        });
    }
    let level = Level { bodies: descs };
    match worlds::write_ron(&level_path(name), &level) {
        Ok(()) => info!("saved level '{name}' ({} bodies)", level.bodies.len()),
        Err(e) => error!("save level: {e}"),
    }
}
